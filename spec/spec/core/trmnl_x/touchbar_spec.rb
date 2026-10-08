# frozen_string_literal: true

# The TRMNL X touch bar: browsing cached images, holds, the WiFi-reset and power-off
# confirmations, and slide mode (swipes; flicks aren't enabled).

RSpec.describe "TRMNL X touch bar", env: "TRMNL_X" do
  fixture(:shipped) { TrmnlX::ShippedX.new }
  fixture(:dev) { TrmnlX::ProvisionedX.new(shipped) }

  # Put fingers on several zones at the same instant (and leave them there).
  def hold_edges(s, *zones)
    s.pause { zones.each { s.touch_down(_1) } }
  end

  def lift(s, *zones) = zones.each { s.touch_up(_1) }

  # Touch the zone for `ms`, wait for the firmware's console `line` about it, and for the
  # device to be back asleep.
  def touch_and_sleep(s, zone, ms, line)
    c = s.status["console_total"]
    s.touch(zone, ms:)
    s.wait(console: line, since: c, timeout: 15)
    s.wait(state: "deep_sleep", timeout: 15, settle_ms: 500)
  end

  shared_context "provisioned X" do
    before { dev.reset }

    # Boot and show "1" then "2" (both cached); yields the simulator (asleep) and their
    # expected screens.
    def boot_two_images(mode = nil)
      m = dev.mock
      one = m.set_png("one", TrmnlX.digits("1"))
      two = m.set_png("two", TrmnlX.digits("2"))
      extra = mode ? { touchbar_mode: mode } : {}
      m.display_queue << { image: "one", refresh_rate: 300, **extra }
      m.display = { image: "two", refresh_rate: 300, **extra }
      dev.boot do |s|
        m.wait_for_request("/images/one.png", timeout: 15)
        s.wait(state: "deep_sleep", timeout: 15)
        s.wake
        m.wait_for_request("/images/two.png", timeout: 15)
        s.wait(state: "deep_sleep", timeout: 15, settle_ms: 500)
        yield s, one, two
      end
    end
  end

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

  describe "WifiResetConfirmation" do
    include_context "provisioned X"

    # Boot asleep and ask to reset WiFi; yields the simulator and the console index before.
    def ask
      dev.boot_asleep do |s|
        s.wait(state: "deep_sleep", timeout: 15)
        c = s.status["console_total"]
        TrmnlX.ask_to_reset_wifi(s, c)
        yield s, c
      end
    end

    it "holding both edges then the middle resets WiFi" do
      ask do |s|
        s.touch("center", ms: 1500)
        s.wait(portal: true, timeout: 15)
      end
    end

    it "tapping an edge cancels" do
      ask do |s, c|
        s.touch("left", ms: 150)
        s.wait(console: /Confirmation cancelled - outer button/, since: c, timeout: 15)
        st = s.wait(state: "deep_sleep", timeout: 15)["status"]
        expect(st["portal_url"]).to be_nil
      end
    end

    it "tapping the middle cancels" do
      ask do |s, c|
        s.touch("center", ms: 150)
        s.wait(console: /Confirmation cancelled - tap on middle/, since: c, timeout: 15)
        s.wait(state: "deep_sleep", timeout: 15)
      end
    end

    it "no answer cancels after 15 seconds" do
      ask do |s, c|
        s.wait(console: /Confirmation timeout - cancelling/, since: c, timeout: 15)
        s.wait(state: "deep_sleep", timeout: 15)
      end
    end
  end

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

    # The IQS323 reports a tap on release, when the slider coordinate is already 0xFFFF and no
    # channel is touched, so slide mode also wakes on the touch: the firmware notes the touched
    # channels, then waits for the gesture (slide_mode_await_gesture in src/bl.cpp).
    it "taps" do
      boot_two_images("slide") do |s|
        { "left" => "Back button pressed", "center" => "Middle button pressed",
          "right" => "Next button pressed" }.each do |zone, line|
          c = s.status["console_total"]
          s.touch(zone, ms: 150)
          s.wait(console: line, since: c, timeout: 15)
          s.wait(state: "deep_sleep", timeout: 15, settle_ms: 300)
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

    it "goes back to tap mode" do
      boot_two_images("slide") do |s, one|
        dev.mock.display = { image: "two", refresh_rate: 300, touchbar_mode: "tap" }
        dev.mock.next_request("/api/display", timeout: 15) { s.wake }
        s.wait(state: "deep_sleep", timeout: 15, settle_ms: 500)
        touch_and_sleep(s, "left", 150, /Back button tapped/)
        expect(s).to show_image(one, tolerance: 64, max_ratio: 0)
      end
    end
  end

  # In the setup portal, holding both edges asks whether to power off (back to shipment mode);
  # a middle hold confirms, a tap or 15 s without an answer cancels.
  describe "PowerOffConfirmation" do
    # Boot the shipped X into the portal and ask to power off; yields the simulator and the
    # console index before.
    def ask
      shipped.boot do |s|
        s.wait_for_console(/Entering shipment mode light sleep loop/, timeout: 30)
        s.dock(true)
        s.wait(portal: true, timeout: 30)
        s.dock(false)
        s.wait(display_idle: true, settle_ms: 200, timeout: 15)
        c = s.status["console_total"]
        hold_edges(s, "left", "right")
        s.wait(console: /Entering power-off confirmation mode/, since: c, timeout: 15)
        lift(s, "left", "right")
        s.wait(console: /display_show_msg end/, since: c, timeout: 15)
        s.wait(display_idle: true, settle_ms: 100, timeout: 15)
        yield s, c
      end
    end

    def expect_cancelled(s, c)
      s.wait(console: /Entering power-off confirmation mode/, since: c, timeout: 1)
      s.wait(display_idle: true, settle_ms: 200, timeout: 30)
      expect(s.status["portal_url"]).not_to be_nil # the portal carries on
    end

    it "middle hold powers off" do
      ask do |s, c|
        boots = s.status["boot_count"]
        s.touch("center", ms: 1500)
        s.wait(console: /Confirmed - holding middle button in tap mode/, since: c, timeout: 15)
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

    it "a tap cancels" do
      ask do |s, c|
        s.touch("left", ms: 150) # the portal runs the touch bar in tap mode
        s.wait(console: /Confirmation cancelled - outer button in tap mode/, since: c, timeout: 15)
        expect_cancelled(s, c)
      end
    end

    it "no answer cancels after 15 seconds" do
      ask do |s, c|
        s.wait(console: /Confirmation timeout - cancelling/, since: c, timeout: 30)
        expect_cancelled(s, c)
      end
    end
  end
end
