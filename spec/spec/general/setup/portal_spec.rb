# frozen_string_literal: true

require "json"
require "net/http"

# The captive portal of the device under test in setup mode: what phones and laptops probe, the
# settings page and the advanced join options (static IP, WPA2 Enterprise, NTP server, hostname).

General.describe "Portal" do
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

    # The TRMNL X lists what its modem's AT+CWLAP prints, unescaped; parentheses, quotes and
    # commas in an SSID must not end it early (lib/trmnl/src/modem_scan_parse.cpp).
    it "ssids keep their punctuation", :smoke do
      nets = [{ ssid: "Foo (Bar)", rssi: -50 }, { ssid: "Say \"hi\", (5G)", rssi: -60 }]
      fresh_device(networks: nets) do |s|
        scan = s.portal_scan["networks"].to_h { [_1["name"], _1["rssi"]] }
        expect(scan).to eq("Foo (Bar)" => "-50", "Say \"hi\", (5G)" => "-60")
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
      s.set_portal_client(false) # nobody uses the portal while we watch, so turbo can run
      while (st = s.status)["sim_time_s"] < t0 + 30
        expect(st["state"]).not_to eq("deep_sleep"), "went to sleep instead of keeping the portal"
        expect(Time.now).to be < deadline, "the simulation stalled"
        sleep 0.5
      end
      s.set_portal_client(true)
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
end
