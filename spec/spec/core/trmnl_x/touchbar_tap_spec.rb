# frozen_string_literal: true

# The TRMNL X touch bar in tap mode (the default): asking the server for the previous and next screens,
# staying awake for more taps, holds, and the touch menu. Shared helpers: spec/support/touchbar.rb.

RSpec.describe "TRMNL X touch bar: tap mode", env: "TRMNL_X" do
  fixture(:shipped) { TrmnlX::ShippedX.new }
  fixture(:dev) { TrmnlX::ProvisionedX.new(shipped) }

  describe "TapMode" do
    include_context "provisioned X"

    it "right tap asks the server for the next screen" do
      boot_two_images do |s|
        three = set_digits(dev.mock, "three", "3")
        dev.mock.display = { image: "three", refresh_rate: 300 }
        req = dev.mock.next_request("/api/display", timeout: 15) { s.touch("right", ms: 150) }
        expect(req).to have_header("Update-Source", "EXT0")
        expect(req).not_to have_header("special_function")
        dev.mock.wait_for_request("/images/three.png", timeout: 15)
        s.wait(state: "deep_sleep", timeout: 30, settle_ms: 500)
        expect(s).to show_image(three, tolerance: 64, max_ratio: 0)
      end
    end

    it "left tap asks the server for the previous screen, shown from the cache" do
      boot_two_images do |s, one|
        dev.mock.display_queue << previous_screen("one")
        n = dev.mock.cursor
        req = dev.mock.next_request("/api/display", timeout: 15) { s.touch("left", ms: 150) }
        expect(req).to have_header("special_function", "rewind")
        s.wait(state: "deep_sleep", timeout: 30, settle_ms: 500)
        expect(s).to show_image(one, tolerance: 64, max_ratio: 0)
        expect(dev.mock.paths.drop(n)).not_to include("/images/one.png")
      end
    end

    # The loading logo goes up while it downloads, with the arrow over it (showTouchbarLoading,
    # src/status_screens.cpp), as the logo's full refresh erases the one drawn on the touch.
    it "a previous screen that isn't cached is downloaded under the loading logo" do
      boot_two_images do |s|
        three = set_digits(dev.mock, "three", "3")
        dev.mock.display_queue << previous_screen("three")
        dev.mock.set_fault("/images/three.png", delay: 1, times: 1)
        n = dev.mock.cursor
        s.touch("left", ms: 150)
        dev.mock.wait_for_request("/images/three.png", after: n, timeout: 15)
        s.wait(display_idle: true, settle_ms: 100, timeout: 5)
        expect(s).to match_golden("touchbar_loading_left.png")
        s.wait(state: "deep_sleep", timeout: 30, settle_ms: 500)
        expect(s).to show_image(three, tolerance: 64, max_ratio: 0)
      end
    end

    it "shows the arrow until the screen is back, even if the screen didn't change" do
      boot_two_images do |s, _one, two|
        dev.mock.set_fault("/api/display", delay: 0.5, times: 1)
        c = s.status["console_total"]
        s.touch("right", ms: 150)
        s.wait(console: /Next button tapped/, since: c, timeout: 15)
        expect(arrow_pixels(s, two)).to be_positive
        s.wait(state: "deep_sleep", timeout: 30, settle_ms: 500)
        expect(s).to show_image(two, tolerance: 64, max_ratio: 0)
      end
    end

    it "holding left or right asks the server too" do
      boot_two_images do |s, one, two|
        dev.mock.display_queue << previous_screen("one")
        req = dev.mock.next_request("/api/display", timeout: 15) do
          touch_and_sleep(s, "left", 2500, /Back button hold/)
        end
        expect(req).to have_header("special_function", "rewind")
        expect(s).to show_image(one, tolerance: 64, max_ratio: 0)
        req = dev.mock.next_request("/api/display", timeout: 15) do
          touch_and_sleep(s, "right", 2500, /Next button hold/)
        end
        expect(req).not_to have_header("special_function")
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
        s.wait(state: "deep_sleep", timeout: 30)
      end
    end

    it "a server error on a tap shows the error" do
      boot_two_images do |s|
        dev.mock.set_fault("/api/display", close: true)
        s.touch("right", ms: 150)
        s.wait(state: "deep_sleep", timeout: 60, settle_ms: 500)
        expect(s).to match_golden("wifi_internal_error.png")
      end
    end
  end

  # After a touch, the X stays awake and online until 20 s pass without one (touchbar_stay_awake,
  # src/touchbar_actions.cpp).
  describe "StayAwake" do
    include_context "provisioned X"

    # A tap on the X showing "2", for the screen the server has now; yields once it is shown, with
    # the console index from before the tap.
    def tapped_awake
      boot_two_images do |s, one, two|
        c = s.status["console_total"]
        s.touch("right", ms: 150)
        s.wait(console: /Staying awake 20 s/, since: c, timeout: 30)
        yield s, c, one, two
      end
    end

    it "takes more taps on the same wake and connection, then sleeps after 20 s without a touch" do
      tapped_awake do |s, c, one, two|
        boots = s.status["boot_count"]
        wifi_starts = s.console(0).grep(/esp_wifi_start/).size
        dev.mock.display_queue << previous_screen("one")
        since = s.status["console_total"]
        req = dev.mock.next_request("/api/display", timeout: 15) { s.touch("left", ms: 150) }
        expect(req).to have_header("special_function", "rewind")
        s.wait(console: /Rewind: server sent/, since:, timeout: 15)
        s.wait(console: /display_show_image end/, since:, timeout: 15)
        s.wait(display_idle: true, settle_ms: 300, timeout: 15)
        expect(s).to show_image(one, tolerance: 64, max_ratio: 0)

        req = dev.mock.next_request("/api/display", timeout: 15) { s.touch("right", ms: 150) }
        expect(req).not_to have_header("special_function") # the rewind was for its own tap
        tapped_at = s.status["sim_time_s"]
        st = s.wait(state: "deep_sleep", timeout: 30, settle_ms: 500)["status"]
        expect(st["sim_time_s"] - tapped_at).to be >= 19
        expect(s).to show_image(two, tolerance: 64, max_ratio: 0)

        expect(st["boot_count"]).to eq(boots)
        expect(s.console(0).grep(/esp_wifi_start/).size).to eq(wifi_starts)
        expect(s.console(c).join("\n")).to match(/No touches for 20 s/)
      end
    end

    # Streaming publishes on every RDY: only touch changes get a line (iqs323_publish_snapshot).
    it "doesn't log every touch bar reading while it waits" do
      tapped_awake do |s, c|
        since = s.status["console_total"]
        s.wait(console: /No touches for 20 s/, since:, timeout: 30)
        log = s.console(c).join("\n")
        expect(s.console(since).grep(/publish snapshot/)).to be_empty
        expect(log).not_to match(/RDY time:|Queue value updates/)
      end
    end

    it "a finger held on the bar keeps it awake" do
      tapped_awake do |s|
        s.touch("right", ms: 25_000)
        released_at = s.status["sim_time_s"]
        expect(s.status["state"]).not_to eq("deep_sleep")
        st = s.wait(state: "deep_sleep", timeout: 30)["status"]
        expect(st["sim_time_s"] - released_at).to be_within(3).of(20)
      end
    end

    # A short refresh, e.g. the server's "Catching you up" screen, comes due while it's still awake:
    # it fetches the screen again, without an arrow as no tap asked.
    it "fetches a refresh that comes due while awake" do
      boot_two_images do |s, one|
        c = s.status["console_total"]
        dev.mock.display_queue << { image: "two", refresh_rate: 5 } << { image: "one", refresh_rate: 300 }
        s.touch("right", ms: 150)
        s.wait(console: /Refresh of 5 s due while awake/, since: c, timeout: 30)
        s.wait(state: "deep_sleep", timeout: 30, settle_ms: 500)
        expect(s).to show_image(one, tolerance: 64, max_ratio: 0)
        expect(dev.mock.count("/api/display")).to eq(2)
      end
    end

    # Every draw drives all of the indicator's pixels: only the first of a boot used to be sure to
    # (display_draw_touchbar_indicator, src/touchbar_indicator.cpp).
    it "draws the arrow on every tap of a wake" do
      tapped_awake do |s, c, _one, two|
        tapped = { "left" => /Back button tapped/, "right" => /Next button tapped/ }
        %w[right left right].each do |zone|
          dev.mock.display_queue << previous_screen("two") if zone == "left"
          dev.mock.set_fault("/api/display", delay: 0.5, times: 1)
          since = s.status["console_total"]
          s.touch(zone, ms: 150)
          s.wait(console: tapped[zone], since:, timeout: 15)
          expect(arrow_pixels(s, two)).to be_positive
          s.wait(console: /display_show_image end/, since:, timeout: 15)
          s.wait(display_idle: true, settle_ms: 300, timeout: 15)
          expect(s).to show_image(two, tolerance: 64, max_ratio: 0)
        end
        expect(s.console(c).join("\n")).not_to match(/Touchbar indicator update failed/)
      end
    end

    it "the menu closes back to the screen, and the wake takes another tap" do
      tapped_awake do |s, _c, _one, two|
        close_menu(s)
        n = dev.mock.count("/api/display")
        s.wait(display_idle: true, settle_ms: 300, timeout: 15)
        expect(s).to show_image(two, tolerance: 64, max_ratio: 0)
        expect(dev.mock.count("/api/display")).to eq(n)
        dev.mock.next_request("/api/display", timeout: 15) { s.touch("right", ms: 150) }
        s.wait(state: "deep_sleep", timeout: 30, settle_ms: 500)
      end
    end

    it "the menu's Next Screen fetches the screen" do
      tapped_awake do |s, c|
        three = set_digits(dev.mock, "three", "3")
        dev.mock.display = { image: "three", refresh_rate: 300 }
        since = s.status["console_total"]
        s.touch("center", ms: 150)
        s.wait(console: /menu_show_screen end/, since:, timeout: 30)
        dev.mock.next_request("/api/display", timeout: 15) { s.touch("center", ms: 150) }
        s.wait(state: "deep_sleep", timeout: 30, settle_ms: 500)
        expect(s).to show_image(three, tolerance: 64, max_ratio: 0)
        expect(s.console(c).join("\n")).to match(/Middle button tapped/)
      end
    end

    # A reset comes up in event mode: without going back to streaming the wake's taps would be lost,
    # and the reset's own read used to send the task round the reinit forever (iqs323_do_init).
    it "gets over a touch controller reset, once, and still takes taps" do
      tapped_awake do |s|
        since = s.status["console_total"]
        s.set_faults(touch_bar: "reset")
        s.wait(console: /IQS323 Task: Reinit successful/, since:, timeout: 15)
        s.set_faults(touch_bar: nil)
        dev.mock.next_request("/api/display", timeout: 15) { s.touch("right", ms: 150) }
        expect(s.console(since).grep(/IQS323 Task: Reinit requested/).size).to eq(1)
        s.wait(state: "deep_sleep", timeout: 30, settle_ms: 500)
      end
    end

    it "a timer wake doesn't stay awake" do
      boot_two_images do |s|
        c = s.status["console_total"]
        s.wake
        s.wait(state: "deep_sleep", timeout: 30)
        expect(s.console(c).join("\n")).not_to match(/Staying awake/)
      end
    end
  end

  describe "MainMenu" do
    include_context "provisioned X"

    it "Previous Screen asks the server for it" do
      boot_two_images do |s, one|
        dev.mock.display_queue << previous_screen("one")
        c = s.status["console_total"]
        s.touch("center", ms: 150)
        s.wait(console: /menu_show_screen end/, since: c, timeout: 30)
        TrmnlX.menu_tap(s, "right", /Next Screen -> Previous Screen/)
        req = dev.mock.next_request("/api/display", timeout: 15) { s.touch("center", ms: 150) }
        expect(req).to have_header("special_function", "rewind")
        s.wait(state: "deep_sleep", timeout: 30, settle_ms: 500)
        expect(s).to show_image(one, tolerance: 64, max_ratio: 0)
      end
    end

    # A wake that closes the menu never went online: it sleeps if no tap comes (process_iqs323_data).
    it "Close redraws the screen and sleeps after 20 s without asking the server" do
      boot_two_images do |s, _one, two|
        n = dev.mock.cursor
        close_menu(s)
        closed_at = s.status["sim_time_s"]
        st = s.wait(state: "deep_sleep", timeout: 30, settle_ms: 500)["status"]
        expect(st["sim_time_s"] - closed_at).to be >= 19
        expect(s).to show_image(two, tolerance: 64, max_ratio: 0)
        expect(dev.mock.paths.drop(n)).to eq([])
      end
    end

    # run_menu_screen drops taps made while its screen is drawn
    it "ignores a double tap's second tap, made while the menu is drawn" do
      boot_two_images do |s, _one, two|
        n = dev.mock.cursor
        c = s.status["console_total"]
        s.touch("center", ms: 150)
        s.wait(console: /menu_show_screen start/, since: c, timeout: 30)
        s.touch("center", ms: 150)
        s.wait(console: /Menu: timeout - closing/, since: c, timeout: 30)
        s.wait(state: "deep_sleep", timeout: 60, settle_ms: 500)
        expect(s.console(c).join("\n")).not_to match(/Menu: chose/)
        expect(s).to show_image(two, tolerance: 64, max_ratio: 0)
        expect(dev.mock.paths.drop(n)).to eq([])
      end
    end

    [1, 2].each do |profile|
      it "Close preserves saved temperature profile #{profile} without an API response" do
        boot_two_images do |s, _one, two|
          s.set_preference("temp_profile", "u32", profile)
          n = dev.mock.cursor
          c = s.status["console_total"]
          close_menu(s)
          s.wait(state: "deep_sleep", timeout: 30, settle_ms: 500)
          expect(s.console(c).join("\n")).to include("Saved temperature profile: #{profile}")
          expect(s.preferences.fetch("temp_profile")).to eq(profile.to_s)
          expect(s).to show_image(two, tolerance: 64, max_ratio: 0)
          expect(dev.mock.paths.drop(n)).to eq([])
        end
      end
    end

    it "a tap after Close goes on with the wake" do
      boot_two_images do |s|
        three = set_digits(dev.mock, "three", "3")
        dev.mock.display = { image: "three", refresh_rate: 300 }
        close_menu(s)
        s.wait(display_idle: true, settle_ms: 300, timeout: 15)
        req = dev.mock.next_request("/api/display", timeout: 30) { s.touch("right", ms: 150) }
        expect(req).not_to have_header("special_function")
        s.wait(state: "deep_sleep", timeout: 30, settle_ms: 500)
        expect(s).to show_image(three, tolerance: 64, max_ratio: 0)
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

    it "Toggle Hold Screen asks the server for it by name and shows its answer" do
      boot_two_images do |s|
        three = set_digits(dev.mock, "three", "3")
        dev.mock.display = { image: "three", action: "guest_mode", refresh_rate: 300, special_function: nil }
        c = s.status["console_total"]
        s.touch("center", ms: 150)
        s.wait(console: /menu_show_screen end/, since: c, timeout: 30)
        TrmnlX.menu_tap(s, "right", /Next Screen -> Previous Screen/)
        TrmnlX.menu_tap(s, "right", /Previous Screen -> Touchbar Tutorial/)
        TrmnlX.menu_tap(s, "right", /Touchbar Tutorial -> Special/)
        TrmnlX.menu_tap(s, "center", /chose Special/, screen: true)
        TrmnlX.menu_tap(s, "right", /Identify -> Restart Playlist/)
        TrmnlX.menu_tap(s, "right", /Restart Playlist -> Send to Me/)
        TrmnlX.menu_tap(s, "right", /Send to Me -> Guest Mode/)
        TrmnlX.menu_tap(s, "right", /Guest Mode -> Toggle Hold Screen/)
        req = dev.mock.next_request("/api/display", timeout: 30) { s.touch("center", ms: 150) }
        expect(req).to have_header("special_function", "toggle_hold")
        dev.mock.wait_for_request("/images/three.png", timeout: 30)
        s.wait(state: "deep_sleep", timeout: 60, settle_ms: 500)
        expect(s).to show_image(three, tolerance: 64, max_ratio: 0)
      end
    end
  end

  describe "WifiResetConfirmation" do
    include_context "provisioned X"

    # Open Reset Wi-Fi's confirmation on the X showing "2" (boot_two_images); yields the simulator,
    # the console index before and the expected screen of "2". A timeout redraws the image the
    # menu covered (show_last_image, src/cached_images.cpp), then sleeps once 20 s pass without a touch.
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
        st = s.wait(state: "deep_sleep", timeout: 30, settle_ms: 500)["status"]
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
