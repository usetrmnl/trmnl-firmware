# frozen_string_literal: true

require "json"
require "net/http"

# The captive portal of the device under test in setup mode: what phones and laptops probe, the
# settings page, the advanced join options (static IP, WPA2 Enterprise, NTP server, hostname) and
# several saved networks.

General.describe "Portal" do
  fixture(:dev) { ProvisionedDevice.new(build) }

  # GET from the portal without following redirects: [status, headers (lowercase names), body].
  # Like Python's http.client, it asks for no content coding (so a gzip body stays gzipped).
  def raw_get(s, path, timeout: 30)
    url = URI(s.portal_url)
    Net::HTTP.start(url.host, url.port, open_timeout: timeout, read_timeout: timeout) do |http|
      r = http.get(path, "Accept-Encoding" => "identity")
      [r.code.to_i, r.to_hash.transform_values { _1.join(", ") }, r.body.to_s]
    end
  end

  # Ask for the scan results (the block) until the scan is done: /scan answers 202 while it runs
  # (for at most 60 s). Returns the last answer ([code, ...]).
  def scanned
    deadline = Time.now + 60
    loop do
      answer = yield
      return answer if answer[0] != 202 || Time.now > deadline

      sleep 0.5 # 202: still scanning (a 200 starts the next scan)
    end
  end

  describe "Probes" do
    fixture(:portal) { fresh_device }

    it "redirects captive portal checks to the setup page" do
      aggregate_failures do
        %w[/generate_204 /redirect /hotspot-detect.html /canonical.html /ncsi.txt /no/such/page].each do |path|
          code, headers, = raw_get(portal, path)
          expect(code).to eq(302), path
          expect(headers).to include("location"), path
        end
      end
    end

    it "answers the windows and firefox checks" do
      code, headers, = raw_get(portal, "/connecttest.txt")
      expect([code, headers["location"]]).to eq([302, "http://logout.net"])
      expect(raw_get(portal, "/wpad.dat")[0]).to eq(404)
      expect(raw_get(portal, "/success.txt")[0]).to eq(200)
      expect(raw_get(portal, "/favicon.ico")[0]).to eq(404)
    end

    it "serves the setup page gzipped" do
      code, headers, body = raw_get(portal, "/")
      expect(code).to eq(200)
      expect(headers["content-encoding"]).to eq("gzip")
      expect(body.bytesize).to be > 1000
      code, headers, = raw_get(portal, "/advanced")
      expect([code, headers["location"]]).to eq([302, "/#advanced"])
    end

    it "device settings report the api url" do
      code, _, body = raw_get(portal, "/device-settings")
      expect(code).to eq(200)
      expect(JSON.parse(body)).to include("api_url")
    end

    it "runs the sensor self test" do
      # two averages of 1000 chip temperature readings, 7 s apart: over 9 s of device time,
      # more on the clock where the simulation runs slower than real time
      code, _, body = raw_get(portal, "/run-test", timeout: 120)
      expect(code).to eq(200)
      expect { JSON.parse(body) }.not_to raise_error
    end

    it "does a forced rescan" do
      code, = raw_get(portal, "/scan?force=1")
      expect(code).to eq(200).or eq(202)
      code, _, body = scanned { raw_get(portal, "/scan") }
      expect(code).to eq(200)
      expect(JSON.parse(body)["networks"].map { _1["name"] }).to include("TRMNL-Sim")
    end
  end

  describe "ScanList" do
    it "access points are merged by ssid", :smoke do
      nets = [{ ssid: "TRMNL-Sim", rssi: -75 }, { ssid: "TRMNL-Sim", rssi: -45, channel: 11 },
              { ssid: "Cafe", open: true, rssi: -60 }, { ssid: "TRMNL", rssi: -30 }]
      fresh_device(networks: nets) do |s|
        scan = s.portal_scan["networks"].to_h { [_1["name"], _1] }
        expect(scan["TRMNL-Sim"]["rssi"]).to eq("-45") # the strongest of the two
        expect(scan["Cafe"]["open"]).to be(true)
        expect(scan).not_to include("TRMNL") # another device's setup network
      end
    end
  end

  describe "JoinOptions" do
    # Join TRMNL-Sim through the portal with the setup page's extra `fields`, towards a mock
    # server; yields the simulator.
    def join(**fields)
      device_mock do |mock|
        fresh_device do |s|
          body = { ssid: "TRMNL-Sim", pswd: "password", server: mock.device_url, **fields }
          code, data = s.portal_request("/connect", body)
          expect(code).to eq(200), data
          yield s, mock
        end
      end
    end

    it "static ip" do
      join(useStaticIP: true, staticIP: "192.168.4.50", gateway: "192.168.4.1", subnet: "255.255.255.0",
           dns1: "1.1.1.1", dns2: "8.8.8.8") do |s|
        s.wait_for_console(/Static IP configured/, timeout: 90)
        s.wait_for_deep_sleep(timeout: 180)
      end
    end

    it "static ip with defaults" do
      join(useStaticIP: true, staticIP: "192.168.4.50") do |s|
        s.wait_for_console(/Static IP configured/, timeout: 90)
        s.wait_for_deep_sleep(timeout: 180)
      end
    end

    it "invalid static ip falls back to dhcp" do
      join(useStaticIP: true, staticIP: "not-an-ip") do |s, mock|
        s.wait_for_console(/Invalid static IP address/, timeout: 90)
        mock.wait_for_request("/api/setup", timeout: 120)
        s.wait_for_deep_sleep(timeout: 300)
      end
    end

    it "wpa2 enterprise" do
      join(isEnterprise: true, identity: "alice@example.com", username: "alice") do |s, mock|
        s.wait_for_console(/WPA2 Enterprise/, timeout: 90)
        mock.wait_for_request("/api/setup", timeout: 120)
        s.wait_for_deep_sleep(timeout: 180)
      end
    end

    it "wpa2 enterprise identity only" do
      join(isEnterprise: true, identity: "alice@example.com", pswd: "") do |s|
        s.wait_for_console(/WPA2 Enterprise/, timeout: 90)
        s.wait_for_deep_sleep(timeout: 300)
      end
    end

    it "ntp server and hostname are saved" do
      join(ntpServer1: "time.example.com", hostname: "kitchen-trmnl") do |s|
        s.wait_for_console(/Saved NTP server: time.example.com/, timeout: 60)
        s.wait_for_console(/Saved hostname: kitchen-trmnl/, timeout: 60)
        s.wait_for_deep_sleep(timeout: 180)
      end
    end
  end

  # A join that fails leaves the portal up, so the details can be corrected.
  describe "FailedJoin", skip_if: :shipment, why: "its portal comes from shipment mode, and 5 GHz joins go through " \
                                                  "the modem (core/trmnl_x/trmnl_x_spec FailedJoin)" do
    # Submit the setup page (to a server that doesn't matter) with `fields` changed; returns
    # the console cursor from before it.
    def join(s, **fields)
      c = s.status["console_total"]
      body = { ssid: "TRMNL-Sim", pswd: "password", server: "http://x", **fields }
      code, data = s.portal_request("/connect", body)
      expect(code).to eq(200), data
      c
    end

    # From the failed join on, the device stays in setup mode (whether its AP stayed up or it
    # brought the portal back) and the portal answers.
    def expect_portal_stays(s)
      t0 = s.status["sim_time_s"]
      deadline = Time.now + 90
      while (st = s.status)["sim_time_s"] < t0 + 30
        expect(st["state"]).not_to eq("deep_sleep"), "went to sleep instead of keeping the portal"
        expect(Time.now).to be < deadline, "the simulation stalled"
        sleep 0.5
      end
      s.wait(portal: true, timeout: 30)
      expect(s.portal_scan["networks"].map { _1["name"] }).to include("TRMNL-Sim")
    end

    it "wrong password keeps the portal", :smoke do
      fresh_device(networks: [{ ssid: "TRMNL-Sim", password: "password" }]) do |s|
        c = join(s, pswd: "wrong")
        s.wait(console: /connect attempt failed/, since: c, timeout: 60)
        expect_portal_stays(s)
      end
    end

    it "unknown network keeps the portal" do
      fresh_device do |s|
        c = join(s, ssid: "No Such Network")
        s.wait(console: /connect attempt failed/, since: c, timeout: 60)
        expect_portal_stays(s)
      end
    end

    it "wpa2 enterprise without identity keeps the portal" do
      fresh_device do |s|
        c = join(s, isEnterprise: true)
        s.wait(console: /requires an identity/, since: c, timeout: 90)
        expect_portal_stays(s)
      end
    end
  end

  describe "SavedNetworks" do
    before { dev.reset }

    # Join another network through the add_wifi special function (which, unlike a long press,
    # keeps the saved networks): the setup portal comes up.
    def add_network(s)
      m = dev.mock
      m.display = { image: "default", refresh_rate: 300, special_function: "add_wifi" }
      m.next_request("/api/display", timeout: 120) { s.wake }
      s.wait(state: "deep_sleep", timeout: 120, settle_ms: 200)
      m.display = { image: "default", refresh_rate: 300, action: "add_wifi" }
      s.press(1500)
      s.wait(portal: true, timeout: 120)
      s
    end

    # add_network: the add_wifi special function
    it "second network is used when the first is gone", needs: %i[double_click button] do
      nets = [{ ssid: "TRMNL-Sim" }, { ssid: "Office", password: "office-pass", rssi: -60 }]
      dev.boot_asleep(networks: nets) do |s|
        s.wait_for_deep_sleep
        add_network(s)
        s.portal_connect("Office", "office-pass", server: dev.mock.device_url)
        dev.mock.next_request("/api/display", timeout: 120)
        s.wait(state: "deep_sleep", timeout: 120, settle_ms: 200)
        s.set_networks([{ ssid: "TRMNL-Sim" }]) # the office (last used) is out of range
        n = dev.mock.cursor
        c = s.status["console_total"]
        s.wake
        s.wait(console: /Trying to connect to saved network TRMNL-Sim/, since: c, timeout: 240)
        dev.mock.wait_for_request("/api/display", after: n, timeout: 240)
        s.wait_for_deep_sleep(timeout: 120)
      end
    end

    # add_network: the add_wifi special function
    it "joining a saved network again is not saved twice", needs: %i[double_click button] do
      dev.boot_asleep do |s|
        s.wait_for_deep_sleep
        add_network(s)
        c = s.status["console_total"]
        s.portal_connect("TRMNL-Sim", "password", server: dev.mock.device_url)
        s.wait(console: /Duplicate regular network found/, since: c, timeout: 120)
        s.wait_for_deep_sleep(timeout: 120)
      end
    end

    # add_network: the add_wifi special function
    it "portal lists saved networks out of range", needs: %i[double_click button] do
      dev.boot_asleep do |s|
        s.wait_for_deep_sleep
        add_network(s)
        s.set_networks([{ ssid: "Somewhere Else", rssi: -70 }])
        s.portal_request("/scan?force=1")
        _, body = scanned { s.portal_request("/scan") }
        saved = JSON.parse(body)["networks"].select { _1["saved"] }.to_h { [_1["name"], _1] }
        expect(saved).to include("TRMNL-Sim")
      end
    end

    it "wifi failures on timer wakes end with the wifi error" do
      dev.boot_asleep do |s|
        s.wait_for_deep_sleep
        screen = s.screenshot
        s.set_wifi(false)
        shown = 14.times.any? do
          s.wake
          s.wait(state: "deep_sleep", timeout: 240, settle_ms: 200)
          !s.compare_screen(screen, tolerance: 0, max_ratio: 0)["match"]
        end
        expect(shown).to be(true), "the WiFi error was never shown"
      end
    end
  end
end
