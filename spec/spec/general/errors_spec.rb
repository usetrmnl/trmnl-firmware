# frozen_string_literal: true

require "json"

# Error handling on the device under test: the error screens (and their weak-WiFi variants),
# quiet retries on timer wakes and /api/setup failures during onboarding.

# WIFI_CONNECTION_RSSI is -100: at or below it the device blames the WiFi signal.
weak = [{ ssid: "TRMNL-Sim", rssi: -100 }]

General.describe "Errors" do
  fixture(:dev) { ProvisionedDevice.new(build) }

  # Wait until the device sleeps or restarts (a crash); returns its status.
  def settle(s, timeout: 120)
    deadline = Process.clock_gettime(Process::CLOCK_MONOTONIC) + timeout
    loop do
      st = s.status
      return st if st["boot_count"] > 1 || st["state"] == "halted"
      return s.wait(state: "deep_sleep", timeout: 30, settle_ms: 200)["status"] if st["state"] == "deep_sleep"
      if Process.clock_gettime(Process::CLOCK_MONOTONIC) > deadline
        raise TrmnlSim::TimeoutError, "device neither slept nor restarted: #{st}"
      end

      sleep 0.2
    end
  end

  # Power on (an error is shown right away), wait for the error screen and yield the simulator.
  def boot_with_error(**kw)
    dev.boot(**kw) do |s|
      st = settle(s)
      expect([st["boot_count"], st["state"]]).to eq([1, "deep_sleep"])
      yield s if block_given?
    end
  end

  describe "ErrorScreens" do
    before { dev.reset }

    it "api unreachable", :smoke do
      dev.mock.set_fault("/api/display", close: true)
      boot_with_error { |s| expect(s.status["display_refreshes"]).to be > 1 }
    end

    it "api unreachable on weak wifi" do
      dev.mock.set_fault("/api/display", close: true)
      boot_with_error(networks: weak)
    end

    it "image cut short on weak wifi" do
      dev.mock.set_fault("/images/*", truncate: 1000)
      boot_with_error(networks: weak)
    end

    it "image host unreachable" do
      dev.mock.set_fault("/images/*", close: true)
      boot_with_error
    end

    it "image url that can't be fetched" do
      # HTTPClient refuses the URL, so the request never starts: HTTPS_UNABLE_TO_CONNECT.
      # The screen blames the API although it answered (only HTTP errors from the image
      # host get the "image download failed" screen).
      dev.mock.display = { image_url: "ftp://10.0.2.2/image.bmp", filename: "plugin-bbbbbb-1", refresh_rate: 300 }
      boot_with_error do |s|
        # "WiFi connected, unable connect to API." and how to retry (on the 960x540 parallel
        # panels the logo, centred, runs into the text's fixed rows)
        expect(s).to show_message("api_unable_to_connect.png", [200, 320, 400, 64])
      end
      log = dev.mock.wait_for_request("/api/log", timeout: 10)
      expect(log.body).to include("HTTPS_UNABLE_TO_CONNECT - Unable to create WiFiClient")
    end

    it "image too large" do
      dev.mock.set_file("/huge.bmp", "image/bmp", "\0".b * 100_000)
      dev.mock.display = { image_url: "#{dev.mock.device_url}/huge.bmp", filename: "plugin-aaaaaa-1",
                           refresh_rate: 300 }
      boot_with_error
    end
  end

  describe "Retries" do
    before { dev.reset }

    it "timer wakes retry quietly then show the error" do
      dev.boot_asleep do |s|
        s.wait_for_deep_sleep
        screen = s.screenshot
        dev.mock.set_fault("/api/display", status: 500)
        quiet = true
        attempt = 0
        5.times do |i|
          attempt = i
          dev.mock.next_request("/api/display", timeout: 90) { s.wake }
          s.wait(state: "deep_sleep", timeout: 180, settle_ms: 200)
          quiet = s.compare_screen(screen, tolerance: 0, max_ratio: 0)["match"]
          break unless quiet
        end
        expect(quiet).to be(false), "the error was never shown"
        expect(attempt).to be >= 2, "retried quietly first"
        dev.mock.clear_faults
        dev.mock.next_request(dev.mock.image_path("default"), timeout: 90) { s.wake }
        s.wait_for_deep_sleep
      end
    end

    # WiFi connect failures (wifiErrorDeepSleep, src/sleep_session.cpp:192): the attempt number
    # is kept in NVS ("wifi_retry", reset to 1 on every successful connect, src/wifi_session.cpp:118), and
    # RefreshInterval::applyWifiRetry (lib/trmnl/src/refresh_interval.cpp:31) stores the sleep:
    # SHORT_TERM_SLOW_RETRY_INTERVAL (300 s) below MAX_QUIET_SLOW_RETRIES (12), else
    # LONG_TERM_SLOW_RETRY_INTERVAL (900 s). Timer wakes don't show the error right away
    # (should_show_error_now, src/bl.cpp:180).

    # Wake on the timer with WiFi out of range; returns the status once asleep again, after
    # checking the attempt number the device logged.
    def wifi_failure(s, attempt)
      s.wake
      s.wait(console: /WIFI connection failed! Retry count: #{attempt}\b/, timeout: 240)
      s.wait(state: "deep_sleep", timeout: 240, settle_ms: 200)["status"]
    end

    def sleep_s(status) = status["wake_at_s"] - status["sim_time_s"]

    it "wifi failures on timer wakes retry quietly every 5 minutes" do
      dev.boot_asleep do |s|
        s.wait_for_deep_sleep
        # a server rate other than the retry interval, so the retries must replace it
        dev.mock.display = { image: "default", refresh_rate: 600 }
        dev.mock.next_request("/api/display", timeout: 90) { s.wake }
        expect(sleep_s(s.wait_for_deep_sleep)).to be_within(15).of(600)
        screen = s.screenshot
        s.set_wifi(false)
        expect do
          (1..3).each do |attempt|
            expect(sleep_s(wifi_failure(s, attempt))).to be_within(15).of(300), "attempt #{attempt}"
            expect(s).to show_image(screen, tolerance: 0, max_ratio: 0), "attempt #{attempt} redrew the screen"
          end
        end.not_to(change { dev.mock.requests.size })
      end
    end

    it "wifi failures at the retry limit show the wifi error and sleep longer" do
      dev.boot_asleep do |s|
        s.wait_for_deep_sleep
        screen = s.screenshot
        s.set_wifi(false)
        (1..11).each do |attempt|
          expect(sleep_s(wifi_failure(s, attempt))).to be_within(15).of(300), "attempt #{attempt}"
          expect(s).to show_image(screen, tolerance: 0, max_ratio: 0), "attempt #{attempt} redrew the screen"
        end
        # The 12th shows WIFI_FAILED (not WIFI_RETRY_LIMIT's "press button" screen) and still
        # arms the timer, for 15 minutes (src/sleep_session.cpp:203).
        st = wifi_failure(s, 12)
        expect(st["wake_at_s"]).not_to be_nil, "the timer must still wake it"
        expect(sleep_s(st)).to be_within(15).of(900)
        expect(s).not_to show_image(screen, tolerance: 0, max_ratio: 0)
        # "Can't establish WiFi connection. Will keep trying." (only the X has a golden of it)
        expect(s).to match_golden("wifi_failed_message.png") if Golden.own_region("wifi_failed_message.png", device)
        # the count starts over, back on the 5-minute retries
        expect(sleep_s(wifi_failure(s, 1))).to be_within(15).of(300)
        s.set_wifi(true)
        dev.mock.next_request("/api/display", timeout: 120) { s.wake }
        s.wait_for_deep_sleep(timeout: 120)
      end
    end
  end

  # Onboarding against a server whose /api/setup misbehaves.
  describe "SetupErrors" do
    let(:mock) { TrmnlSim::MockTrmnl.new }

    after { mock.close }

    # Onboard a fresh device against `mock` until it asks /api/setup; yields the simulator.
    def onboard(**kw)
      sim(erase: true, extra_args: ["--offline"], **kw) do |s|
        s.wait(portal: true, timeout: 90)
        s.portal_connect("TRMNL-Sim", "password", server: mock.device_url)
        mock.wait_for_request("/api/setup", timeout: 120)
        yield s
      end
    end

    # Onboard, and expect the device to go to sleep on the error screen without restarting.
    def onboard_and_sleep(**kw)
      onboard(**kw) do |s|
        yield s if block_given?
        expect(settle(s)["boot_count"]).to eq(1)
      end
    end

    # The message's lines (on the rows from 340 down, under the logo) are each centred.
    def expect_centred_lines(s, at_least)
      width = device.width
      lines = text_lines(s, 300)
      expect(lines.size).to be >= at_least, "message lines: #{lines}"
      lines.each do |top, bottom, left, right|
        # a glyph's side bearings can differ by a pixel or two
        expect((left - (width - 1 - right)).abs).to be <= 3,
                                                    "rows #{top}-#{bottom}: ink from x=#{left} to #{right} on a " \
                                                    "#{width}-pixel panel"
      end
    end

    it "unregistered mac shows the signup message" do
      mock.setup = nil # 404 "MAC Address not registered"
      onboard do |s|
        st = settle(s)
        expect([st["boot_count"], st["state"]]).to eq([1, "deep_sleep"])
        expect(mock.count("/api/display")).to eq(0)
      end
    end

    it "long unregistered message is wrapped" do
      message = "Your device #{'is not yet registered with any account, ' * 3}" \
                "visit https://usetrmnl.com/signup/with-a-very-long-link-that-cannot-be-wrapped-anywhere-at-all " \
                "and enter Device ID SIMTST"
      mock.setup = nil
      mock.set_fault("/api/setup", body: JSON.generate({ status: 404, api_key: nil, friendly_id: nil, image_url: nil,
                                                         message: }))
      onboard do |s|
        st = settle(s, timeout: 30)
        expect([st["boot_count"], st["state"]]).to eq([1, "deep_sleep"])
        # wrapped at spaces, the link broken inside, every line centred
        expect_centred_lines(s, 4)
      end
    end

    it "unregistered message is centred" do
      # trmnl.app's "MAC ... not registered - send to support@trmnl.com to activate your TRMNL"
      # (issue #650: drawn from the left edge)
      mock.setup = nil
      onboard do |s|
        settle(s)
        expect_centred_lines(s, 2)
      end
    end

    it "setup server error" do
      mock.set_fault("/api/setup", status: 500)
      onboard_and_sleep
    end

    it "setup server error on weak wifi" do
      mock.set_fault("/api/setup", status: 500)
      onboard_and_sleep(networks: weak)
    end

    it "setup malformed json" do
      mock.set_fault("/api/setup", body: '{"status": 200, "api_key": ')
      onboard_and_sleep
    end

    { "missing" => { status: 404 }, "not an image" => { body: "not an image at all" },
      "host unreachable" => { close: true } }.each do |what, fault|
      it "setup logo #{what}" do
        mock.set_fault("/images/*", **fault)
        onboard_and_sleep { mock.wait_for_request("/images/default.bmp", timeout: 120) }
      end
    end
  end
end
