# frozen_string_literal: true

# The TRMNL X touch bar tutorial (src/menu.cpp, run_tutorial): shown on the first touch until it is
# finished, and from the main menu's Touchbar Tutorial. Shared helpers: spec/support/touchbar.rb.

RSpec.describe "TRMNL X touch bar: tutorial", env: "TRMNL_X" do
  fixture(:shipped) { TrmnlX::ShippedX.new }
  fixture(:dev) { TrmnlX::ProvisionedX.new(shipped) }

  describe "Tutorial" do
    include_context "provisioned X"

    # Tap the left, center and right of the touch bar in turn on the tutorial screen (from the
    # `from`th), then close it with a tap anywhere (the left, here).
    def practice_and_close(s, from: 0)
      practice = [["left", /Tutorial: tried Previous/], ["center", /Tutorial: tried Menu/],
                  ["right", /Tutorial: tried Next/]]
      practice.drop(from).each { |zone, log| TrmnlX.menu_tap(s, zone, log) }
      c = s.status["console_total"]
      s.touch("left", ms: 150)
      s.wait(console: /Tutorial: done/, since: c, timeout: 15)
      s.wait(state: "deep_sleep", timeout: 15, settle_ms: 500)
    end

    it "shows on the first touch, until it is finished" do
      boot_two_images do |s, _one, two|
        s.set_preference("touch_tutorial", "u8", 0) # as after an update from before it
        c = s.status["console_total"]
        s.touch("right", ms: 150)
        s.wait(console: /tutorial_show_screen end/, since: c, timeout: 30)
        expect(s.console(c).join("\n")).not_to match(/Next button tapped/)

        practice_and_close(s)
        expect(s).to show_image(two, tolerance: 64, max_ratio: 0)
        expect(s.preferences["touch_tutorial"]).to eq("1")

        touch_and_sleep(s, "left", 150, /Back button tapped/)
      end
    end

    it "takes the controls in order: left, center, right" do
      boot_two_images do |s|
        s.set_preference("touch_tutorial", "u8", 0)
        c = s.status["console_total"]
        s.touch("center", ms: 150)
        s.wait(console: /tutorial_show_screen end/, since: c, timeout: 30)
        TrmnlX.menu_tap(s, "center", /Tutorial: tapped Menu, waiting for Previous/)
        TrmnlX.menu_tap(s, "right", /Tutorial: tapped Next, waiting for Previous/)
        TrmnlX.menu_tap(s, "left", /Tutorial: tried Previous/)
        TrmnlX.menu_tap(s, "right", /Tutorial: tapped Next, waiting for Menu/)
        expect(s.console(c).join("\n")).not_to match(/Tutorial: done/)
        practice_and_close(s, from: 1)
      end
    end

    it "opens from the main menu" do
      boot_two_images do |s, _one, two|
        c = s.status["console_total"]
        s.touch("center", ms: 150)
        s.wait(console: /menu_show_screen end/, since: c, timeout: 30)
        TrmnlX.menu_tap(s, "right", /Next Screen -> Previous Screen/)
        TrmnlX.menu_tap(s, "right", /Previous Screen -> Touchbar Tutorial/)
        s.touch("center", ms: 150)
        s.wait(console: /tutorial_show_screen end/, since: c, timeout: 15)
        practice_and_close(s)
        expect(s).to show_image(two, tolerance: 64, max_ratio: 0)
      end
    end
  end
end
