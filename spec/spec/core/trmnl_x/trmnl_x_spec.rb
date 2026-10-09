# frozen_string_literal: true

require "json"
require "tmpdir"

# TRMNL X: factory flow, shipment mode and the dock, onboarding over 2.4 GHz (S3 WiFi) and
# 5 GHz (ESP32-C5 modem), the 1872x1404 parallel panel, the touch bar, and charging headers.

RSpec.describe "TRMNL X", env: "TRMNL_X" do
  fixture(:shipped) { TrmnlX::ShippedX.new }
  fixture(:dev) { TrmnlX::ProvisionedX.new(shipped) } # onboarded on the 5 GHz network, through the modem

  def monotonic = Process.clock_gettime(Process::CLOCK_MONOTONIC)

  describe "FactoryAndDock" do
    it "factory flow flashes the modem then ships" do
      log = shipped.factory_console.join("\n")
      expect(log).to include("[MODEM] FLASH COMPLETE!")
      expect(log).to include("Entering shipment mode light sleep loop")
    end

    it "factory flow waits until taken off the dock to ship" do
      # Still on USB power when QA is done: "ready to ship" until it comes off the dock.
      TrmnlX.sim(erase: true) do |s|
        s.dock(true)
        s.wait_for_console(/USB power still detected/, timeout: 30)
        s.dock(false)
        s.wait_for_console(/Entering shipment mode light sleep loop/, timeout: 30)
      end
    end

    it "shipment mode waits for the dock" do
      shipped.boot do |s|
        s.wait_for_console(/Entering shipment mode light sleep loop/, timeout: 180)
        st = s.wait(state: "light_sleep", timeout: 60)["status"]
        expect(st["docked"]).to be(false)
        expect(st["portal_url"]).to be_nil
        s.dock(true)
        st = s.wait(portal: true, timeout: 180)["status"]
        expect(st["docked"]).to be(true)
        expect(st["boot_count"]).to be >= 2 # restarted out of shipment mode
      end
    end

    it "portal soft reset restarts from core 0" do
      # /soft-reset calls ESP.restart() on async_tcp (core 0), which resets the APP core
      # first and must not see it come back before both cores restart.
      shipped.boot do |s|
        s.wait_for_console(/Entering shipment mode light sleep loop/, timeout: 180)
        s.dock(true)
        boots = s.wait(portal: true, timeout: 180)["status"]["boot_count"]
        begin
          s.portal_request("/soft-reset", retry_for: 0)
        rescue *TrmnlSim::Simulator::CONNECTION_ERRORS
          nil # the device may restart before it answers
        end
        st = s.wait(min_boots: boots + 1, portal: true, timeout: 180)["status"]
        expect(st["state"]).not_to eq("halted")
      end
    end
  end

  describe "Provisioned" do
    before { dev.reset }

    it "5 GHz requests go through the modem" do
      dev.boot do |s|
        req = dev.mock.wait_for_request("/api/display", timeout: 120)
        expect(req).to have_header("RSSI", "-48") # the 5 GHz AP, seen by the modem
        %w[ID FW-Version Model Battery-Voltage Width Height].each { |h| expect(req).to have_header(h) }
        expect(req.headers.values_at("Width", "Height")).to eq(%w[1872 1404])
        expect(req).to have_header("Model", "x")
        expect(req).not_to have_header("Panel-Rev") # FastEPD panels aren't read
        s.wait(state: "deep_sleep", timeout: 120)
      end
    end

    it "renders a 1-bit png exactly" do
      expected = dev.mock.set_png("seven", TrmnlX.digits("7"))
      dev.mock.display = { image: "seven", refresh_rate: 300 }
      dev.boot do |s|
        dev.mock.wait_for_request("/images/seven.png", timeout: 120)
        s.wait(state: "deep_sleep", timeout: 120)
        expect(s).to show_image(expected, tolerance: 64, max_ratio: 0)
      end
    end

    it "shows 16 gray levels from a 4-bit png" do
      expected = dev.mock.set_png("ramp", ->(x, _y) { [15, x * 16 / 1872].min }, bits: 4)
      dev.mock.display = { image: "ramp", refresh_rate: 300 }
      dev.boot do |s|
        dev.mock.wait_for_request("/images/ramp.png", timeout: 120)
        s.wait(state: "deep_sleep", timeout: 120)
        expect(s).to show_image(expected, tolerance: 40, max_ratio: 0.001)
      end
    end

    it "sleeps for the refresh rate" do
      dev.mock.display = { image: "default", refresh_rate: 600 }
      dev.boot do |s|
        st = s.wait(state: "deep_sleep", timeout: 120)["status"]
        expect(st["wake_at_s"] - st["sim_time_s"]).to be_within(15).of(600)
      end
    end

    it "restored X shows the same screen and wakes by touch" do
      seven = dev.mock.set_png("seven", TrmnlX.digits("7"))
      dev.mock.display = { image: "seven", refresh_rate: 300 }
      Dir.mktmpdir do |tmp|
        path = File.join(tmp, "x.trmnlsave")
        saved, screen, boots = dev.boot do |s|
          dev.mock.wait_for_request("/images/seven.png", timeout: 120)
          s.wait(state: "deep_sleep", timeout: 120)
          s.dock(true)
          [s.save_point(path), s.screenshot, s.status["boot_count"]]
        end
        expect(saved["deep_sleep"]).to be(true)
        eight = dev.mock.set_png("eight", TrmnlX.digits("8"))
        dev.mock.display = { image: "eight", refresh_rate: 300 }
        dev.mock.requests.clear
        dev.restore(path) do |s|
          st = s.wait(state: "deep_sleep", timeout: 30)["status"]
          expect(st.values_at("boot_count", "docked", "charging")).to eq([boots, true, true])
          expect(s).to show_image(screen, tolerance: 0, max_ratio: 0)
          expect(s).to show_image(seven, tolerance: 64, max_ratio: 0)
          # Touch wake needs the IQS323 configuration and RTC state from before the save.
          s.touch("center", ms: 150)
          req = dev.mock.wait_for_request("/api/display", timeout: 120)
          expect(req).to have_header("Update-Source", "EXT0")
          expect(req).to have_header("RSSI", "-48") # still on 5 GHz, through the modem
          expect(req).to have_header("USB-Connected", "true")
          dev.mock.wait_for_request("/images/eight.png", timeout: 120)
          s.wait(state: "deep_sleep", timeout: 120)
          expect(s).to show_image(eight, tolerance: 64, max_ratio: 0)
          expect(dev.mock.paths).not_to include("/api/setup")
        end
      end
    end
  end

  describe "TouchAndDock" do
    before { dev.reset }

    it "center tap wakes and refreshes" do
      dev.boot_asleep do |s|
        s.wait(state: "deep_sleep", timeout: 120)
        req = dev.mock.next_request("/api/display", timeout: 120) { s.touch("center", ms: 150) }
        expect(req).to have_header("Update-Source", "EXT0")
        s.wait(state: "deep_sleep", timeout: 120)
      end
    end

    it "left tap shows the previous image offline" do
      one = dev.mock.set_png("one", TrmnlX.digits("1"))
      two = dev.mock.set_png("two", TrmnlX.digits("2"))
      dev.mock.display_queue << { image: "one", refresh_rate: 300 }
      dev.mock.display = { image: "two", refresh_rate: 300 }
      dev.boot do |s|
        dev.mock.wait_for_request("/images/one.png", timeout: 120)
        s.wait(state: "deep_sleep", timeout: 120)
        s.wake
        dev.mock.wait_for_request("/images/two.png", timeout: 120)
        s.wait(state: "deep_sleep", timeout: 120, settle_ms: 500)
        expect(s).to show_image(two, tolerance: 64, max_ratio: 0)
        n = dev.mock.cursor
        c = s.status["console_total"]
        s.touch("left", ms: 150)
        s.wait(console: /Playlist browse/, since: c, timeout: 60)
        s.wait(state: "deep_sleep", timeout: 120)
        expect(s).to show_image(one, tolerance: 64, max_ratio: 0)
        expect(dev.mock.paths.drop(n)).to eq([]) # no network needed
      end
    end

    it "dock reports usb power and charging" do
      dev.boot do |s|
        req = dev.mock.wait_for_request("/api/display", timeout: 120)
        expect(req.headers.values_at("USB-Connected", "Battery-Charging")).to eq(%w[false 0])
        s.wait(state: "deep_sleep", timeout: 120)
        s.dock(true)
        expect(s.status["charging"]).to be(true)
        req = dev.mock.next_request("/api/display", timeout: 120) { s.wake }
        expect(req.headers.values_at("USB-Connected", "Battery-Charging")).to eq(%w[true 1])
        s.wait(state: "deep_sleep", timeout: 120)
      end
    end

    it "reports the fuel gauge next to its voltage estimate" do
      # Production X builds estimate the charge from the voltage (BYPASS_BQ27427_SOC) and
      # send the gauge's own readings in the Gauge-* headers for comparison. At 4.06 V the
      # estimate sits on its 100 % plateau; the simulated gauge says 88 %.
      req = dev.boot_asleep do |s|
        s.wait(state: "deep_sleep", timeout: 120)
        s.set_battery(4060)
        display = dev.mock.next_request("/api/display", timeout: 120) { s.wake }
        s.wait(state: "deep_sleep", timeout: 120)
        display
      end
      h = req.headers
      expect(req).to have_header("Battery-Voltage", a_value_within(0.05).of(4.06))
      expect(req).to have_header("Battery-Count", "1")
      # the voltage estimate, with a nominal 6000 mAh per cell and no state of health
      expect(h.values_at("Percent-Charged", "Battery-Capacity", "Battery-Health")).to eq(%w[100 6000/6000 -1])
      # the gauge's own view
      expect(h.values_at("Gauge-SOC", "Gauge-Capacity", "Gauge-Health")).to eq(%w[88 5280/6000 100])
      expect(h.values_at("Battery-Current", "Battery-Temp")).to eq(%w[-50 25.00]) # discharging
    end
  end

  # Nobody joins the setup portal: after 15 minutes the X goes back to shipment mode.
  describe "PortalTimeout" do
    def time_out(s, docked: false)
      s.wait_for_console(/Entering shipment mode light sleep loop/, timeout: 30)
      s.dock(true)
      s.wait(portal: true, timeout: 30)
      s.dock(docked)
      s.set_portal_client(false) # so turbo can run to the timeout
      # Serial isn't running when enter_shipment_sleep() announces itself on a production
      # build, so go by the clock: the portal is gone once 15 minutes have passed.
      t0 = s.status["sim_time_s"]
      st = nil
      # about 40 s alone, longer next to other tests
      eventually("the portal did not time out", within: 150) { (st = s.status)["sim_time_s"] >= t0 + (15 * 60) + 5 }
      expect(st["portal_url"]).to be_nil
      st["console_total"]
    end

    it "unattended portal goes back to shipment mode" do
      shipped.boot do |s|
        # Still docked: "ready to ship" until it comes off USB power, then shipment mode.
        time_out(s, docked: true)
        g = s.status["display_generation"]
        s.dock(false) # checked every 2 s: then the shipping screen
        # (wall-clock limits allow for a machine busy with other simulators)
        eventually("the shipping screen was not drawn", within: 90, every: 0.2) do
          s.status["display_generation"] != g
        end
        s.wait(display_idle: true, timeout: 90)
        c = s.status["console_total"]
        s.dock(true) # the charger ends shipment mode
        s.wait(console: /CHARGER DETECTED - Exiting shipment mode/, since: c, timeout: 90)
      end
    end

    # The setup screen (WIFI_CONNECT) ends with display_sleep(1000), which arms a 1 s
    # light-sleep timer that is never disarmed. enter_shipment_sleep() only adds the
    # charger's GPIO wakeup, so the device wakes about every 1.2 s ("Unexpected wakeup
    # cause: 4") instead of sleeping until it is docked, draining the battery in the box.
    it "shipment mode after the timeout stays asleep" do
      shipped.boot do |s|
        c = time_out(s)
        t0 = s.status["sim_time_s"]
        sleep 0.2 while s.status["sim_time_s"] < t0 + 10
        wakes = s.console(c).grep(/WAKEUP from light sleep/)
        expect(wakes).to eq([])
      end
    end
  end

  describe "Onboarding" do
    it "portal rescans both radios and joins the chosen band" do
      TrmnlSim::MockTrmnl.open do |mock|
        shipped.boot do |s|
          s.wait_for_console(/Entering shipment mode light sleep loop/, timeout: 30)
          s.dock(true)
          s.wait(portal: true, timeout: 30)
          s.dock(false)
          c = s.status["console_total"]
          s.portal_request("/scan?force=1")
          s.wait(console: /Modem re-scan found/, since: c, timeout: 30)
          deadline = monotonic + 30
          sleep 0.5 while s.portal_request("/scan")[0] != 200 && monotonic < deadline
          code, body = s.portal_request("/scan")
          expect(JSON.parse(body)).to include("mac_5ghz") if code == 200
          body = { ssid: TrmnlX::SSID_24, pswd: "password", server: mock.device_url, band: "2.4GHz" }
          expect(s.portal_request("/connect", body)[0]).to eq(200)
          req = mock.wait_for_request("/api/display", timeout: 30)
          expect(req).to have_header("RSSI", "-54") # the S3's own radio
        end
      end
    end

    it "onboarding on 2.4 GHz uses the S3 radio" do
      TrmnlSim::MockTrmnl.open do |mock|
        shipped.boot do |s|
          TrmnlX.onboard(s, mock, TrmnlX::SSID_24)
          req = mock.wait_for_request("/api/display")
          expect(req).to have_header("RSSI", "-54") # the 2.4 GHz AP, seen by the S3
        end
      end
    end

    it "onboarding on 5 GHz reports the panel size and signal through the modem" do
      # /api/setup goes through the modem (DeviceSetupX::callSetupApi) with the same headers
      TrmnlSim::MockTrmnl.open do |mock|
        shipped.boot do |s|
          TrmnlX.onboard(s, mock, TrmnlX::SSID_5)
          req = mock.wait_for_request("/api/setup")
          expect(req).to have_header("Width", "1872")
          expect(req).to have_header("Height", "1404")
          expect(req).to have_header("RSSI", "-48") # the 5 GHz AP, seen by the modem
        end
      end
    end
  end

  # A join that fails leaves the portal up, so the password can be corrected.
  describe "FailedJoin" do
    def portal(s)
      s.wait_for_console(/Entering shipment mode light sleep loop/, timeout: 180)
      s.dock(true)
      s.wait(portal: true, timeout: 180)
      s.dock(false)
      s.set_networks([{ ssid: TrmnlX::SSID_24, password: "password", rssi: -54, channel: 6 },
                      { ssid: TrmnlX::SSID_5, password: "password", rssi: -48, channel: 36 }])
    end

    def join(s, ssid, band)
      c = s.status["console_total"]
      code, data = s.portal_request("/connect", { ssid:, pswd: "wrong", server: "http://x", band: })
      expect(code).to eq(200), data
      c
    end

    # For a minute of simulated time the device stays in setup mode (whether its AP stayed up
    # or it brought the portal back), and the portal answers.
    def expect_portal_to_stay(s)
      t0 = s.status["sim_time_s"]
      eventually("the simulation stalled", within: 120) do
        st = s.status
        expect(st["state"]).not_to eq("deep_sleep"), "went to sleep instead of keeping the portal"
        st["sim_time_s"] >= t0 + 60
      end
      s.wait(portal: true, timeout: 30)
      expect(s.portal_request("/")[0]).to eq(200)
    end

    it "wrong password on 2.4 GHz keeps the portal" do
      shipped.boot do |s|
        portal(s)
        c = join(s, TrmnlX::SSID_24, "2.4GHz")
        s.wait(console: /connect attempt failed/, since: c, timeout: 60)
        expect_portal_to_stay(s)
      end
    end

    it "wrong password on 5 GHz keeps the portal" do
      shipped.boot do |s|
        portal(s)
        join(s, TrmnlX::SSID_5, "5GHz") # the modem's join fails silently on a production build
        expect_portal_to_stay(s)
      end
    end
  end
end
