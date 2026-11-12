# frozen_string_literal: true

# Live mode on the TRMNL X: while answers carry keep_wifi_on, the X stays awake and checks in every
# refresh_rate seconds, drawing each new frame, and sleeps once an answer drops it (touchbar_stay_awake).
# On 5 GHz the X fetches through its modem, on 2.4 GHz through ESP Wi-Fi with kept connections.

RSpec.describe "TRMNL X live mode", env: "TRMNL_X" do
  fixture(:shipped) { TrmnlX::ShippedX.new }

  { "5 GHz" => TrmnlX::SSID_5, "2.4 GHz" => TrmnlX::SSID_24 }.each do |band, ssid|
    describe "on #{band}" do
      fixture(:dev) { TrmnlX::ProvisionedX.new(shipped, ssid:) }

      include Touchbar

      before { dev.reset }

      def live(name) = { image: name, refresh_rate: 1, keep_wifi_on: true }

      it "checks in and draws each frame awake, then sleeps when the session ends" do
        m = dev.mock
        set_digits(m, "one", "1")
        set_digits(m, "two", "2")
        three = set_digits(m, "three", "3")
        m.display_queue = [live("one"), live("two"), live("three")]
        m.display = { image: "three", refresh_rate: 300 }
        dev.boot_asleep do |s|
          s.wait(state: "deep_sleep", timeout: 15)
          c = s.status["console_total"]
          n = m.cursor
          s.wake
          s.wait(state: "deep_sleep", timeout: 90, settle_ms: 500)
          lines = s.console(c)
          expect([m.paths.drop(n).count("/api/display"), lines.grep(/No touches for/).size]).to eq([4, 0])
          expect(s).to show_image(three, tolerance: 64, max_ratio: 0)
        end
      end
    end
  end
end
