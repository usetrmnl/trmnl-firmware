# frozen_string_literal: true

# The TRMNL X touch bar in tap mode (the default): browsing cached images, holds, and the touch
# menu's Settings. Shared helpers: spec/support/touchbar.rb.

RSpec.describe "TRMNL X touch bar: tap mode", env: "TRMNL_X" do
  fixture(:shipped) { TrmnlX::ShippedX.new }
  fixture(:dev) { TrmnlX::ProvisionedX.new(shipped) }

  describe "TapMode" do
    include_context "provisioned X"

    it "right tap goes forward through cached images" do
      boot_two_images do |s, one, two|
        s.touch("left", ms: 150)
        s.wait(state: "deep_sleep", timeout: 15, settle_ms: 500)
        expect(s).to show_image(one, tolerance: 64, max_ratio: 0)
        touch_and_sleep(s, "right", 150, /Next button tapped/)
        expect(s).to show_image(two, tolerance: 64, max_ratio: 0)
      end
    end

    it "holding left or right browses too" do
      boot_two_images do |s, one, two|
        touch_and_sleep(s, "left", 2500, /Back button hold/)
        expect(s).to show_image(one, tolerance: 64, max_ratio: 0)
        touch_and_sleep(s, "right", 2500, /Next button hold/)
        expect(s).to show_image(two, tolerance: 64, max_ratio: 0)
      end
    end

    it "middle hold refreshes" do
      dev.boot_asleep do |s|
        s.wait(state: "deep_sleep", timeout: 15)
        c = s.status["console_total"]
        dev.mock.next_request("/api/display", timeout: 15) do
          s.touch("center", ms: 2500)
          s.wait(console: /Middle button hold/, since: c, timeout: 15)
        end
        s.wait(state: "deep_sleep", timeout: 15)
      end
    end
  end

  describe "SpecialMenu" do
    include_context "provisioned X"

    it "Send to Me redraws the image the menu covered" do
      boot_two_images do |s, _one, two|
        dev.mock.display = { action: "send_to_me", image_url: nil, filename: nil, refresh_rate: nil,
                             special_function: nil }
        c = s.status["console_total"]
        s.touch("center", ms: 150)
        s.wait(console: /menu_show_screen end/, since: c, timeout: 30)
        TrmnlX.menu_tap(s, "right", /Next Screen -> Previous Screen/)
        TrmnlX.menu_tap(s, "right", /Previous Screen -> Touchbar Tutorial/)
        TrmnlX.menu_tap(s, "right", /Touchbar Tutorial -> Special/)
        TrmnlX.menu_tap(s, "center", /chose Special/, screen: true)
        TrmnlX.menu_tap(s, "right", /Identify -> Restart Playlist/)
        TrmnlX.menu_tap(s, "right", /Restart Playlist -> Send to Me/)
        dev.mock.next_request("/api/display", timeout: 30) { s.touch("center", ms: 150) }
        s.wait(state: "deep_sleep", timeout: 60, settle_ms: 500)
        expect(s).to show_image(two, tolerance: 64, max_ratio: 0)
      end
    end
  end

  describe "WifiResetConfirmation" do
    include_context "provisioned X"

    # Open Reset Wi-Fi's confirmation on the X showing "2" (boot_two_images); yields the simulator,
    # the console index before and the expected screen of "2". A timeout redraws the image the
    # menu covered, then sleeps (show_last_image_and_sleep, src/cached_images.cpp).
    def ask
      boot_two_images do |s, _one, two|
        c = s.status["console_total"]
        TrmnlX.ask_to_reset_wifi(s)
        yield s, c, two
      end
    end

    it "choosing Confirm resets WiFi" do
      ask do |s|
        TrmnlX.menu_tap(s, "left", /Back -> Confirm Reset Wi-Fi/)
        s.touch("center", ms: 150)
        s.wait(portal: true, timeout: 90)
      end
    end

    it "Back goes back to Settings" do
      ask do |s, c|
        # the arrow starts on Back
        TrmnlX.menu_tap(s, "center", /chose Back/, screen: true)
        # Settings again, with the arrow on its first item
        TrmnlX.menu_tap(s, "right", /Diagnostics\.\.\. -> Reset Wi-Fi/)
        TrmnlX.menu_tap(s, "center", /chose Reset Wi-Fi/, screen: true)
        expect(s.console(c).join("\n")).not_to match(/Resetting WiFi/)
        expect(s.status["portal_url"]).to be_nil
      end
    end

    it "no answer cancels after 15 seconds" do
      ask do |s, c, two|
        s.wait(console: /Menu: timeout - closing/, since: c, timeout: 30)
        st = s.wait(state: "deep_sleep", timeout: 15, settle_ms: 500)["status"]
        expect(st["portal_url"]).to be_nil
        expect(s).to show_image(two, tolerance: 64, max_ratio: 0)
      end
    end
  end

  describe "ShutDown" do
    include_context "provisioned X"

    # Open Settings, then Shut Down..., leaving its confirmation on the screen
    def ask(s)
      TrmnlX.open_settings_menu(s)
      TrmnlX.menu_tap(s, "right", /Diagnostics\.\.\. -> Reset Wi-Fi/)
      TrmnlX.menu_tap(s, "right", /Reset Wi-Fi\.\.\. -> Shut Down/)
      TrmnlX.menu_tap(s, "center", /chose Shut Down/, screen: true)
    end

    it "confirming goes back to shipment mode" do
      boot_two_images do |s|
        ask(s)
        TrmnlX.menu_tap(s, "left", /Back -> Confirm Shut Down/)
        boots = s.status["boot_count"]
        c = s.status["console_total"]
        s.touch("center", ms: 150)
        s.wait(console: /Shutting down from the menu/, since: c, timeout: 15)
        deadline = Process.clock_gettime(Process::CLOCK_MONOTONIC) + 30
        while s.status["boot_count"] == boots
          expect(Process.clock_gettime(Process::CLOCK_MONOTONIC)).to be < deadline, "did not restart"
          sleep 0.2
        end
        # shipment status cleared: back to shipment mode until the charger is connected
        c = s.status["console_total"]
        s.wait(console: /Entering shipment mode light sleep loop/, since: c, timeout: 60)
      end
    end

    it "Back goes back to Settings" do
      boot_two_images do |s|
        c = s.status["console_total"]
        ask(s)
        # the arrow starts on Back
        TrmnlX.menu_tap(s, "center", /chose Back/, screen: true)
        TrmnlX.menu_tap(s, "right", /Diagnostics\.\.\. -> Reset Wi-Fi/)
        expect(s.console(c).join("\n")).not_to match(/Shutting down/)
      end
    end
  end

  describe "Diagnostics" do
    include_context "provisioned X"

    it "shows the device's values, and Back goes back to Settings" do
      boot_two_images do |s|
        TrmnlX.open_settings_menu(s)
        c = s.status["console_total"]
        TrmnlX.menu_tap(s, "center", /chose Diagnostics/, screen: true)
        log = s.console(c).join("\n")
        expect(log).to match(/Diagnostics: Firmware\t\d+\.\d+\.\d+/)
        expect(log).to match(/Diagnostics: Wi-Fi Network\tTRMNL-Sim/)
        expect(log).to match(/Diagnostics: Wi-Fi Signal\t-\d+ dBm/)
        TrmnlX.menu_tap(s, "center", /chose Back/, screen: true)
        TrmnlX.menu_tap(s, "right", /Diagnostics\.\.\. -> Reset Wi-Fi/)
      end
    end
  end
end
