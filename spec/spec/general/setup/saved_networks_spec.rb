# frozen_string_literal: true

require "json"

# Several saved networks: the device falls back to the next one, never saves one twice, lists the
# saved ones in its portal (which manages up to five and opens without forgetting them), and shows
# the WiFi error after failing on timer wakes. Kept apart from
# portal_spec.rb so that parallel_rspec, which hands out whole files, runs them on another worker.

General.describe "Portal" do
  fixture(:dev) { ProvisionedDevice.new(build) }

  describe "SavedNetworks" do
    before { dev.reset }

    # Join another network through the add_wifi special function: the setup portal comes up.
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

    # A long press opens the settings portal and keeps the saved networks (button_wake_act,
    # src/button.cpp; on the X, Settings > Reset Wi-Fi in the touch menu).
    def open_settings(s)
      s.press(6000)
      s.wait(portal: true, timeout: 120)
      s
    end

    # GET /saved-networks (lib/wificaptive/src/WebServer.cpp): the slots in use, without passwords.
    def saved_networks(s)
      code, data = s.portal_request("/saved-networks")
      expect(code).to eq(200), data
      JSON.parse(data)
    end

    def saved_ssids(s) = saved_networks(s)["networks"].map { _1["ssid"] }

    it "a long press opens the portal with the saved networks kept", needs: :button do
      dev.boot_asleep do |s|
        s.wait_for_deep_sleep
        open_settings(s)
        list = saved_networks(s)
        expect(list["max"]).to eq(5)
        expect(list["networks"].length).to eq(1)
        expect(list["networks"][0]).to include("index" => 0, "ssid" => "TRMNL-Sim", "last_used" => true)
        expect(list["networks"][0].keys).not_to include("pswd", "password")
      end
    end

    # POST /exit is the settings page's Done: the device restarts and joins a saved network.
    it "closing the portal goes back to the saved network", needs: :button do
      dev.boot_asleep do |s|
        s.wait_for_deep_sleep
        open_settings(s)
        n = dev.mock.cursor
        setups = dev.mock.count("/api/setup")
        code, data = s.portal_request("/exit", {})
        expect(code).to eq(200), data
        dev.mock.wait_for_request("/api/display", after: n, timeout: 240)
        expect(dev.mock.count("/api/setup")).to eq(setups), "should not re-register"
        s.wait_for_deep_sleep(timeout: 120)
      end
    end

    # POST /forget-network names the slot and its SSID. With nothing left to go back to, the
    # portal can't be closed (POST /exit) and the device stays in setup mode.
    it "a saved network is forgotten", needs: :button do
      dev.boot_asleep do |s|
        s.wait_for_deep_sleep
        open_settings(s)
        expect(s.portal_request("/forget-network", { index: 0, ssid: "Office" })[0]).to eq(404) # not its SSID
        expect(s.portal_request("/forget-network", { index: 1, ssid: "TRMNL-Sim" })[0]).to eq(404) # not its slot
        expect(saved_ssids(s)).to eq(["TRMNL-Sim"])
        expect(s.portal_request("/forget-network", { index: 0, ssid: "TRMNL-Sim" })[0]).to eq(200)
        expect(saved_ssids(s)).to eq([])
        expect(s.portal_request("/exit", {})[0]).to eq(409)
      end
    end

    # A network is only saved once the device has joined it (/connect): a wrong password leaves
    # the saved networks as they were, and the portal comes back to say why.
    it "a join that fails is not saved", needs: :button do
      nets = [{ ssid: "TRMNL-Sim" }, { ssid: "Office", password: "office-pass", rssi: -60 }]
      dev.boot_asleep(networks: nets) do |s|
        s.wait_for_deep_sleep
        open_settings(s)
        c = s.status["console_total"]
        s.portal_request("/connect", { ssid: "Office", pswd: "wrong" })
        s.wait(console: /connect attempt failed/, since: c, timeout: 60)
        s.wait(portal: true, timeout: 120)
        expect(saved_ssids(s)).to eq(["TRMNL-Sim"])
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
