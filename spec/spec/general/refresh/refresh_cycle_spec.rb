# frozen_string_literal: true

# A provisioned device talking to a mock TRMNL server: images, sleep, wake sources, headers.

General.describe "Refresh cycle" do
  fixture(:dev) { ProvisionedDevice.new(build) }

  describe "RefreshCycle" do
    before { dev.reset }

    it "renders the image exactly", :smoke do
      path, expected = device_image(dev.mock, "one", device_number("1"))
      dev.mock.display = { image: "one", refresh_rate: 300 }
      dev.boot do |s|
        dev.mock.wait_for_request(path, timeout: 90)
        s.wait(state: "deep_sleep", display_idle: true, timeout: 120)
        expect(s).to show_image(expected, tolerance: 64, max_ratio: 0)
      end
    end

    it "sleeps for the refresh rate" do
      dev.mock.display = { image: "default", refresh_rate: 600 }
      dev.boot do |s|
        st = s.wait_for_deep_sleep
        expect(st["wake_at_s"] - st["sim_time_s"]).to be_within(15).of(600)
      end
    end

    # One answer's image, special function and refresh rate all land: handleApiDisplayResponse
    # (src/display_session.cpp:41-112) saves the function for the next button wake (sf, an
    # SPECIAL_FUNCTION in lib/trmnl/include/special_function.h) and the rate as refresh_rate.
    it "applies the image, special function and refresh rate of an answer", :smoke do
      path, expected = device_image(dev.mock, "three", device_number("3"))
      dev.mock.display = { image: "three", refresh_rate: 900, special_function: "identify" }
      dev.boot do |s|
        dev.mock.wait_for_request(path, timeout: 90)
        st = s.wait(state: "deep_sleep", display_idle: true, timeout: 120)["status"]
        expect(s).to show_image(expected, tolerance: 64, max_ratio: 0)
        expect(st["wake_at_s"] - st["sim_time_s"]).to be_within(15).of(900)
        expect(s.preferences.values_at("sf", "refresh_rate")).to eq(%w[1 900])
      end
    end

    it "fetches the next image on a timer wake", :smoke do
      _, expected = device_image(dev.mock, "two", device_number("2"))
      dev.mock.display = { image: "two", refresh_rate: 300 }
      dev.boot_asleep do |s|
        s.wait_for_deep_sleep
        req = dev.mock.next_request("/api/display", timeout: 90) { s.wake }
        expect(req).to have_header("Update-Source", "timer")
        s.wait(state: "deep_sleep", display_idle: true, timeout: 120)
        expect(s).to show_image(expected, tolerance: 64)
      end
    end

    it "wakes and refreshes on a button press", :smoke,
       needs: :button do
      dev.boot_asleep do |s|
        s.wait_for_deep_sleep
        req = dev.mock.next_request("/api/display", timeout: 90) { s.press(150) }
        expect(req).to have_header("Update-Source", device.button_source)
        s.wait_for_deep_sleep
      end
    end

    it "reports the battery voltage", :smoke do
      dev.boot_asleep do |s|
        s.set_battery(3700)
        s.wait_for_deep_sleep
        req = dev.mock.next_request("/api/display", timeout: 90) { s.wake }
        # boards that can't measure it report a fixed value (see Device#battery_tracks)
        expected = device.battery_tracks? ? 3.70 : device.battery_v
        expect(req).to have_header("Battery-Voltage", a_value_within(0.05).of(expected))
      end
    end

    it "reports the device identity", :smoke do
      dev.boot do |s|
        req = dev.mock.wait_for_request("/api/display", timeout: 90)
        %w[ID FW-Version Model RSSI Width Height].each { |h| expect(req).to have_header(h) }
        expect(req).to have_header("Model", device.model)
        expect(req.headers.values_at("Width", "Height").map(&:to_i)).to eq(device.size)
        expect(req).to have_header("RSSI", "-54")
        if device.panel_rev?
          # read from the UC8179 by bit-banging its REV command (the simulator's default)
          expect(req).to have_header("Panel-Rev", "0a0c1b2c")
        else
          expect(req).not_to have_header("Panel-Rev")
        end
        s.wait_for_deep_sleep
      end
    end

    it "reports the panel revision it reads", needs: :panel_rev do
      dev.boot(extra_args: ["--panel-rev", "0x00c0ffee"]) do |s|
        req = dev.mock.wait_for_request("/api/display", timeout: 90)
        expect(req).to have_header("Panel-Rev", "00c0ffee")
        s.wait_for_deep_sleep
      end
    end

    it "omits the panel revision when it reads zero", needs: :panel_rev do
      dev.boot(extra_args: ["--panel-rev", "0"]) do |s|
        req = dev.mock.wait_for_request("/api/display", timeout: 90)
        expect(req).not_to have_header("Panel-Rev")
        s.wait_for_deep_sleep
      end
    end

    it "keeps its credentials over a power cycle", :smoke do
      dev.boot_asleep do |s|
        s.wait_for_deep_sleep
        req = dev.mock.next_request("/api/display", timeout: 90) { s.power_cycle }
        expect(req).to have_header("Update-Source", "powercycle")
        expect(dev.mock.count("/api/setup")).to eq(0), "should not re-register"
      end
    end

    it "resets WiFi on a long press", needs: :button do
      dev.boot_asleep do |s|
        s.wait_for_deep_sleep
        s.press(6000)
        s.wait(portal: true, timeout: 120)
      end
    end

    it "forgets the device on the portal's soft reset",
       needs: :button do
      # A long press only forgets WiFi; the portal's Soft Reset also clears the API key, so the
      # next onboarding has to register with /api/setup again.
      dev.boot_asleep do |s|
        s.wait_for_deep_sleep
        s.press(6000)
        boots = s.wait(portal: true, timeout: 120)["status"]["boot_count"]
        begin
          s.portal_request("/soft-reset", retry_for: 0)
        rescue *TrmnlSim::Simulator::CONNECTION_ERRORS
          nil # the device may restart before it answers
        end
        s.wait(min_boots: boots + 1, portal: true, timeout: 120)
        s.portal_connect("TRMNL-Sim", "password", server: dev.mock.device_url)
        dev.mock.wait_for_request("/api/setup", timeout: 120)
        dev.mock.wait_for_request("/api/display", timeout: 120)
        s.wait_for_deep_sleep(timeout: 120)
      end
    end

    it "keeps running with WiFi out of range" do
      dev.boot_asleep do |s|
        s.wait_for_deep_sleep
        s.set_wifi(false)
        # The device must give up gracefully and sleep again without reaching the server.
        expect do
          s.wake
          s.wait_for_deep_sleep(timeout: 240)
        end.not_to(change { dev.mock.requests.size })
      end
    end
  end

  describe "FirmwareUpdate" do
    it "installs an OTA update and boots the other slot", :smoke do
      firmware = File.binread(File.join(build, "firmware.bin"))
      url = dev.mock.set_file("/firmware.bin", "application/octet-stream", firmware)
      dev.reset
      dev.mock.display_queue << { image: "default", update_firmware: true, firmware_url: url }
      dev.boot do |s|
        dev.mock.wait_for_request("/firmware.bin", timeout: 120)
        # The bootloader picks the freshly written slot after the restart.
        s.wait_for_console(/booting the app in the other slot, at #{format('%#x', Flash.ota_offset(build))}/,
                           timeout: 300)
        # the new app can't have reached the network yet
        req = dev.mock.next_request("/api/display", timeout: 120)
        expect(req).to have_header("FW-Version", "1.8.16")
        s.wait_for_deep_sleep(timeout: 120)
      end
    end
  end
end
