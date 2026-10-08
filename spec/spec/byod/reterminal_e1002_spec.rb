# frozen_string_literal: true

require "tmpdir"

# Seeed reTerminal E1002 (`seeed_reTerminal_E1002`): an ESP32-S3 with a 7.3" Spectra 6 panel
# (black, white, yellow, red, blue, green), one button and a switched battery divider.

# Colors between the inks: a red-to-blue sweep over the top half, green-to-yellow below.
hues = lambda do |x, y|
  t = x * 255 / 799
  y < 240 ? [t, 128, 255 - t] : [t, 200, 40]
end

RSpec.describe "reTerminal E1002", env: "seeed_reTerminal_E1002" do
  fixture(:dev) { ProvisionedDevice.new(Builds.for_env("seeed_reTerminal_E1002")) }

  let(:images) { TrmnlSim::Images }
  let(:inks) { TrmnlSim::Images::SPECTRA6_RGB }

  # Wake the sleeping device and wait for it to show `path` and sleep again.
  def refresh(s, path)
    dev.mock.next_request(path, timeout: 30) { s.wake }
    s.wait(state: "deep_sleep", timeout: 60, settle_ms: 300)["status"]
  end

  # Serve `data` (a PNG) at /img/<name>.png as the current screen, the way plugins are served.
  def serve_png(name, data)
    dev.mock.set_file("/img/#{name}.png", "image/png", data)
    dev.mock.display = { image_url: "#{dev.mock.device_url}/img/#{name}.png", filename: "plugin-#{name}-1",
                         refresh_rate: 300 }
  end

  describe "Identity" do
    before { dev.reset }

    it "reports the e1002 model and battery" do
      dev.boot_asleep do |s|
        expect(s.status["board"]["name"]).to eq("reTerminal E1002")
        s.wait_for_deep_sleep(timeout: 30)
        s.wake
        req = dev.mock.wait_for_request("/api/display", timeout: 30)
        expect(req).to have_header("Model", "reterminal_e1002")
        expect(req).not_to have_header("Panel-Rev") # not read on Spectra panels
        expect(req.headers.values_at("Width", "Height")).to eq(%w[800 480])
        # 4.1 V through the divider, read while the firmware switches it on (GPIO21)
        expect(req).to have_header("Battery-Voltage", a_value_within(0.05).of(4.1))
        s.wait_for_deep_sleep(timeout: 60)
      end
    end
  end

  describe "Colors" do
    before { dev.reset }

    it "shows the six inks from a palette png" do
      expected = dev.mock.set_spectra6_png("bars", images.spectra_bars)
      dev.mock.display = { image: "bars", refresh_rate: 300 }
      dev.boot_asleep do |s|
        s.wait_for_deep_sleep(timeout: 30)
        refresh(s, "/images/bars.png")
        expect(s).to show_image(expected, tolerance: 16, max_ratio: 0)
      end
    end

    it "reduces a truecolor png to the inks" do
      # the inks, and in-between colors the firmware maps to the nearest
      color = ->(x, y) { y < 240 ? inks[:blue] : [x * 255 / 799, 128, 255 - (x * 255 / 799)] }
      serve_png("rgb", images.png_rgb(color))
      dev.boot_asleep do |s|
        s.wait_for_deep_sleep(timeout: 30)
        refresh(s, "/img/rgb.png")
        expect(s).to show_image(images.expected_spectra6(color), tolerance: 16, max_ratio: 0)
      end
    end

    it "takes the panel's long update to refresh" do
      dev.mock.set_spectra6_png("bars", images.spectra_bars)
      dev.mock.display = { image: "bars", refresh_rate: 300 }
      dev.boot_asleep do |s|
        s.wait_for_deep_sleep(timeout: 30)
        dev.mock.next_request("/images/bars.png", timeout: 30) { s.wake }
        t0 = s.status["sim_time_s"]
        st = s.wait_for_deep_sleep(timeout: 60)
        expect(st["sim_time_s"] - t0).to be > 18 # the Spectra 6 update alone is ~19 s
      end
    end
  end

  # Every pixel format png_draw_6clr decodes, reduced to the nearest ink.
  describe "PngFormats" do
    before { dev.reset }

    def show(name, data, expected)
      serve_png(name, data)
      dev.boot_asleep do |s|
        s.wait_for_deep_sleep(timeout: 30)
        refresh(s, "/img/#{name}.png")
        expect(s).to show_image(expected, tolerance: 16, max_ratio: 0)
      end
    end

    # The firmware widens gray samples its own way: 1-bit white is 0x80, 2-bit tops out at 0xc0.
    def gray(bits, expand)
      top = (1 << bits) - 1
      level = ->(x, y) { ((x * (top + 1) / 800) + (y / 120)) % (top + 1) }
      show("gray#{bits}", images.png_image(level, 800, 480, bits:),
           images.expected_spectra6(->(x, y) { [expand.(level.(x, y))] * 3 }))
    end

    define_method(:indexed) do |bits|
      # PLTE entries must be distinct for the index map
      palette = Array.new(1 << bits) { |x| hues.((x * 800) >> bits, (x & 1) * 240) }.uniq
      color = ->(x, y) { palette[((x * palette.size / 800) + (y / 120)) % palette.size] }
      show("pal#{bits}", images.png_palette(color, palette, bits:), images.expected_spectra6(color))
    end

    it("decodes 1-bit gray") { gray(1, ->(v) { v << 7 }) }
    it("decodes 2-bit gray") { gray(2, ->(v) { v << 6 }) }
    it("decodes 4-bit gray") { gray(4, ->(v) { v * 17 }) }
    it("decodes 8-bit gray") { gray(8, ->(v) { v }) }
    it("decodes a 1-bit palette") { indexed(1) }
    it("decodes a 2-bit palette") { indexed(2) }
    it("decodes an 8-bit palette") { indexed(8) }
    it("decodes truecolor with alpha") { show("rgba", images.png_rgba(hues), images.expected_spectra6(hues)) }
  end

  describe "Setup" do
    it "shows a setup screen matching the OG's" do
      # Same 800x480 layout, access point name and QR code as the OG (all but the version line).
      sim(Builds.for_env("seeed_reTerminal_E1002"), erase: true, extra_args: ["--offline"]) do |s|
        s.wait(portal: true, timeout: 30)
        s.wait(display_idle: true, min_refreshes: 3, settle_ms: 500, timeout: 30)
        expect(s).to match_screenshot(Golden.path("setup_screen_body.png"), region: [0, 56, 800, 424])
        expect(s).to match_screenshot(Golden.path("setup_screen_top_right.png"), region: [320, 0, 480, 56])
      end
    end
  end

  describe "SavePoint" do
    before { dev.reset }

    it "keeps the color image in a save point" do
      bars = dev.mock.set_spectra6_png("bars", images.spectra_bars)
      dev.mock.display = { image: "bars", refresh_rate: 300 }
      green = ->(_x, _y) { inks[:green] }
      Dir.mktmpdir do |tmp|
        path = File.join(tmp, "e1002.trmnlsave")
        dev.boot_asleep do |s|
          s.wait_for_deep_sleep(timeout: 30)
          refresh(s, "/images/bars.png")
          s.save_point(path)
        end
        dev.mock.set_spectra6_png("green", green)
        dev.mock.display = { image: "green", refresh_rate: 300 }
        dev.restore(path) do |s|
          s.wait_for_deep_sleep(timeout: 30)
          expect(s.status["board"]["name"]).to eq("reTerminal E1002")
          expect(s).to show_image(bars, tolerance: 0, max_ratio: 0)
          refresh(s, "/images/green.png")
          expect(s.screenshot(region: [0, 0, 1, 1]).getbyte(25)).to eq(2) # truecolor screenshot
          expect(s).to show_image(images.expected_spectra6(green), tolerance: 0, max_ratio: 0)
        end
      end
    end
  end

  describe "Button" do
    before { dev.reset }

    it "wakes and refreshes on a button press" do
      dev.boot_asleep do |s|
        s.wait_for_deep_sleep(timeout: 30)
        dev.mock.next_request("/api/display", timeout: 30) { s.press(100) }
        s.wait_for_deep_sleep(timeout: 60)
      end
    end
  end
end
