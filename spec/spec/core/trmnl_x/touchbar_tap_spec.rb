# frozen_string_literal: true

# The TRMNL X touch bar in tap mode (the default): browsing cached images, holds, and the WiFi
# reset confirmation. Shared helpers: spec/support/touchbar.rb.

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
end
