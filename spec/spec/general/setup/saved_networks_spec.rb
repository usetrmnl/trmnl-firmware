# frozen_string_literal: true

# Several saved networks: the device falls back to the next one, never saves one twice, lists the
# saved ones in its portal, and shows the WiFi error after failing on timer wakes. Kept apart from
# portal_spec.rb so that parallel_rspec, which hands out whole files, runs them on another worker.

General.describe "Portal" do
  fixture(:dev) { ProvisionedDevice.new(build) }

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
        saved = s.portal_scan["networks"].select { _1["saved"] }.to_h { [_1["name"], _1] }
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
