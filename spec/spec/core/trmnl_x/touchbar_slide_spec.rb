# frozen_string_literal: true

# The TRMNL X touch bar in slide mode: swipes (flicks aren't enabled), taps, the WiFi reset
# confirmation, and going back to tap mode. Shared helpers: spec/support/touchbar.rb.

RSpec.describe "TRMNL X touch bar: slide mode", env: "TRMNL_X" do
  fixture(:shipped) { TrmnlX::ShippedX.new }
  fixture(:dev) { TrmnlX::ProvisionedX.new(shipped) }

  describe "SlideMode" do
    include_context "provisioned X"

    it "swipes browse cached images" do
      boot_two_images("slide") do |s, one, two|
        c = s.status["console_total"]
        s.gesture("swipe_back")
        s.wait(console: /SLIDER: Swipe <-/, since: c, timeout: 15)
        s.wait(state: "deep_sleep", timeout: 15, settle_ms: 500)
        expect(s).to show_image(one, tolerance: 64, max_ratio: 0)
        c = s.status["console_total"]
        s.gesture("swipe_next")
        s.wait(console: /SLIDER: Swipe ->/, since: c, timeout: 15)
        s.wait(state: "deep_sleep", timeout: 15, settle_ms: 500)
        expect(s).to show_image(two, tolerance: 64, max_ratio: 0)
      end
    end

    it "flicks are not enabled" do
      # Slide mode's GESTURE_SELECT (0x0B) enables tap, swipe and hold but not flick, so
      # the controller never reports one (read_gesture_event's flick cases are unused).
      boot_two_images("slide") do |s, _one, two|
        boots = s.status["boot_count"]
        n = dev.mock.cursor
        %w[flick_back flick_next].each do |gesture|
          s.gesture(gesture)
          t0 = s.status["sim_time_s"]
          sleep 0.1 while s.status["sim_time_s"] < t0 + 2
        end
        st = s.status
        expect(st.values_at("state", "boot_count")).to eq(["deep_sleep", boots])
        expect(dev.mock.cursor).to eq(n)
        expect(s).to show_image(two, tolerance: 64, max_ratio: 0)
      end
    end

    # Slide mode wakes only on gesture events, and the IQS323 reports a tap on release, when
    # the slider coordinate is already 0xFFFF and no channel is touched: the wake can't tell
    # which zone was tapped (resolve_intent_slide_mode in lib/trmnl_x/src/touchbar_gesture.cpp
    # needs a touched channel), so a tap anywhere is an ordinary wake that refreshes.
    it "a tap anywhere refreshes" do
      boot_two_images("slide") do |s, _one, two|
        %w[left center right].each do |zone|
          c = s.status["console_total"]
          dev.mock.next_request("/api/display", timeout: 15) { s.touch(zone, ms: 150) }
          s.wait(console: /SLIDER: Tap/, since: c, timeout: 1)
          s.wait(state: "deep_sleep", timeout: 15, settle_ms: 300)
          expect(s.console(c).grep(/button (tapped|hold)/)).to be_empty
          expect(s).to show_image(two, tolerance: 64, max_ratio: 0)
        end
      end
    end

    it "holding both edges asks and a middle hold confirms" do
      boot_two_images("slide") do |s|
        TrmnlX.ask_to_reset_wifi(s, s.status["console_total"])
        s.touch("center", ms: 1500)
        s.wait(portal: true, timeout: 15)
      end
    end

    it "a tap cancels the confirmation" do
      boot_two_images("slide") do |s, _one, two|
        c = s.status["console_total"]
        TrmnlX.ask_to_reset_wifi(s, c)
        s.touch("left", ms: 150)
        s.wait(console: /WiFi reset cancelled by user - tap detected/, since: c, timeout: 15)
        st = s.wait(state: "deep_sleep", timeout: 15, settle_ms: 500)["status"]
        expect(st["portal_url"]).to be_nil
        expect(s).to show_image(two, tolerance: 64, max_ratio: 0)
      end
    end

    # The answer's touchbar_mode switches the mode and saves it (touchbar_mode, a bool: "1" is
    # tap) for the next boot to read (src/display_session.cpp:28-34, src/bl.cpp:261).
    # The mode only reaches the IQS323 on the way to sleep: goToSleep writes it
    # (touchbar_prepare_for_sleep, src/sleep_session.cpp:53) between preparing the controller
    # and stopping its task, and setGestureConfig(tap) drops swipes from GESTURE_SELECT (0x09,
    # lib/IQS323/IQS323.cpp:691). So once the server picks tap mode, the slide config the
    # controller slept with before is gone and a swipe no longer wakes the X.
    it "goes back to tap mode" do
      boot_two_images("slide") do |s, one|
        expect(s.preferences["touchbar_mode"]).to eq("0")
        dev.mock.display = { image: "two", refresh_rate: 300, touchbar_mode: "tap" }
        dev.mock.next_request("/api/display", timeout: 15) { s.wake }
        s.wait(state: "deep_sleep", timeout: 15, settle_ms: 500)
        expect(s.preferences["touchbar_mode"]).to eq("1")
        boots = s.status["boot_count"]
        n = dev.mock.cursor
        s.gesture("swipe_back")
        t0 = s.status["sim_time_s"]
        sleep 0.1 while s.status["sim_time_s"] < t0 + 2
        expect(s.status.values_at("state", "boot_count")).to eq(["deep_sleep", boots])
        expect(dev.mock.cursor).to eq(n)
        touch_and_sleep(s, "left", 150, /Back button tapped/)
        expect(s).to show_image(one, tolerance: 64, max_ratio: 0)
      end
    end
  end
end
