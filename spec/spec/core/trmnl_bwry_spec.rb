# frozen_string_literal: true

require "tmpdir"

# TRMNL BWRY (`trmnl_4clr`): the OG board with a 4-color black/white/yellow/red panel.

RSpec.describe "TRMNL BWRY", env: "trmnl_4clr" do
  fixture(:dev) { ProvisionedDevice.new(Builds.for_env("trmnl_4clr")) }

  let(:red) { ->(_x, _y) { TrmnlSim::Images::BWRY_RGB[:red] } }

  # Serve `color` as image `name` and make it the current screen; returns the expected screenshot.
  def serve(name, color)
    expected = dev.mock.set_color_png(name, color)
    dev.mock.display = { image: name, refresh_rate: 300 }
    expected
  end

  # PNG IHDR color type of a screenshot (2: truecolor).
  def color_type(s) = s.screenshot(region: [0, 0, 8, 8]).getbyte(25)

  describe "Bwry" do
    before { dev.reset }

    it "identifies as the 4-color model" do
      dev.boot do |s|
        expect(s.status["board"]["name"]).to eq("TRMNL BWRY")
        req = dev.mock.wait_for_request("/api/display", timeout: 120)
        expect(req).to have_header("Model", "og_4clr")
        # the REV read is only known to be safe on the black and white panel
        expect(req).not_to have_header("Panel-Rev")
        expect(req.headers.values_at("Width", "Height")).to eq(%w[800 480])
        s.wait_for_deep_sleep(timeout: 120)
      end
    end

    it "renders a color png in four colors" do
      expected = serve("bars", TrmnlSim::Images.color_bars)
      dev.boot do |s|
        dev.mock.wait_for_request("/images/bars.png", timeout: 120)
        s.wait_for_deep_sleep(timeout: 120)
        expect(s).to show_image(expected, tolerance: 16, max_ratio: 0)
      end
    end

    it "takes RGB screenshots" do
      serve("red", red)
      dev.boot do |s|
        dev.mock.wait_for_request("/images/red.png", timeout: 120)
        s.wait_for_deep_sleep(timeout: 120)
        expect(color_type(s)).to eq(2) # IHDR color type 2: truecolor
      end
    end

    it "takes the panel's long update to refresh" do
      serve("bars", TrmnlSim::Images.color_bars)
      dev.boot do |s|
        dev.mock.wait_for_request("/images/bars.png", timeout: 120)
        t0 = s.status["sim_time_s"]
        st = s.wait_for_deep_sleep(timeout: 120)
        expect(st["sim_time_s"] - t0).to be > 15 # the 4-color update alone is ~16 s
      end
    end

    it "keeps the color image in a save point" do
      bars = serve("bars", TrmnlSim::Images.color_bars)
      Dir.mktmpdir do |tmp|
        path = File.join(tmp, "bwry.trmnlsave")
        dev.boot do |s|
          dev.mock.wait_for_request("/images/bars.png", timeout: 120)
          s.wait_for_deep_sleep(timeout: 120)
          s.save_point(path)
        end
        dev.mock.requests.clear
        serve("red", red)
        dev.restore(path) do |s|
          s.wait_for_deep_sleep(timeout: 30)
          expect(s.status["board"]["name"]).to eq("TRMNL BWRY")
          expect(s).to show_image(bars, tolerance: 0, max_ratio: 0)
          dev.mock.next_request("/images/red.png", timeout: 120) { s.wake }
          s.wait_for_deep_sleep(timeout: 120)
          expect(color_type(s)).to eq(2)
          expect(dev.mock.paths).not_to include("/api/setup")
        end
      end
    end
  end

  describe "MemcheckBwry" do
    # display_show_image hands an uncompressed 1-bit BMP to the driver as the frame buffer; on
    # the 4-color panel writePlane reads it as 2 bits per pixel, 48 KB past the end of the 48 KB
    # buffer.
    it "does not send a 1bit bmp as a 2bit plane" do
      TrmnlSim::MockTrmnl.open do |mock|
        sim(Builds.for_env("trmnl_4clr"), erase: true, memcheck: "log", extra_args: ["--offline"]) do |s|
          mock.display = { image: "default", refresh_rate: 300 } # the OG's BMP
          s.wait(portal: true, timeout: 90)
          s.portal_connect("TRMNL-Sim", "password", server: mock.device_url)
          mock.wait_for_request("/api/display", timeout: 120)
          s.wait_for_deep_sleep(timeout: 180)
          s.assert_no_memory_errors
        end
      end
    end
  end
end
