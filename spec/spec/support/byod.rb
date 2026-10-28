# frozen_string_literal: true

# The BYOD boards (the firmware's `device_list[]` rows other than the TRMNL OG, BWRY and X): each
# spec group for one onboards it through the captive portal (cached, see SetupCache), then checks
# what the device reports and what it shows.
#
#   RSpec.describe "BYOD SSD16xx boards" do
#     describe "Waveshare397", env: "WAVESHARE_397" do        # left out unless ENVS lists it
#       byod_board name: "Waveshare ESP32-S3 3.97\"", model: "waveshare_397"
#       it_behaves_like "an SSD16xx board"
#
#       it "reports the battery from the pmic" do
#         dev.boot_asleep { |s| ... }
#       end
#     end
#   end
#
# `byod_board` gives the group `board` (a Byod::Board: what the board should report and show),
# a provisioned device `dev` (a group fixture), a mock reset before every example, the helpers
# of Byod::Helpers (refresh, serve_test_image, ...) and the examples every board gets ("a BYOD
# board").
module Byod
  I = TrmnlSim::Images

  # What a board should report and show.
  #
  #   env          PlatformIO environment (the build directory's name)
  #   name         the simulator's board name (status["board"]["name"])
  #   model        the firmware's DEVICE_MODEL (sent as the Model header)
  #   size         the panel's [width, height] as the device reports it
  #   battery_v    Battery-Voltage it reports at the default 4.1 V battery (nil: don't check)
  #   inks         "mono", "bwry", "spectra6" or "gray16" (a 16-gray parallel panel, served
  #                4-bit PNGs like the TRMNL X): what the panel shows
  #   png_default  serve a panel-sized PNG as the server's default screen (onboarding and
  #                between examples) instead of the 800x480 1-bit BMP, for panels whose firmware
  #                can't take that BMP
  # rubocop:disable Lint/StructNewOverride -- size as in Devices::Device
  Board = Struct.new(:env, :name, :model, :size, :battery_v, :inks, :png_default, keyword_init: true) do
    def width = size[0]
    def height = size[1]
  end
  # rubocop:enable Lint/StructNewOverride
  # A provisioned device whose server's default screen is a PNG the panel's size and kind
  # instead of the 800x480 BMP (see Board#png_default).
  class PanelSizedDefault < ProvisionedDevice
    def initialize(build, size, inks)
      @size = size
      @inks = inks
      super(build, panel_size: nil)
    end

    def cache_name = "provisioned-png-default"

    def new_mock
      mock = super
      w, h = @size
      case @inks
      when "spectra6" then mock.set_spectra6_png("default", I.spectra_bars, w, h)
      when "bwry" then mock.set_color_png("default", I.color_bars, w, h)
      else
        number = I.big_number("0", scale: [4, h / 30].max)
        mock.set_png("default", ->(x, y) { number.(x, y) ? 0 : 1 }, width: w, height: h)
      end
      mock
    end
  end

  # Example group macros (every group has them).
  module Macros
    # Make this group (which must have an `env:`) a BYOD board's: see the top of this file.
    def byod_board(name:, model:, size: [800, 480], battery_v: 4.1, inks: "mono", png_default: false)
      env = metadata[:env]
      raise ArgumentError, "#{metadata[:location]}: byod_board needs the group's env:" unless env.is_a?(String)

      spec = Board.new(env:, name:, model:, size:, battery_v:, inks:, png_default:)
      let(:board) { spec }
      fixture(:dev) do
        build = Builds.for_env(spec.env)
        if spec.png_default
          PanelSizedDefault.new(build, spec.size, spec.inks)
        else
          # Panels of other sizes get a default image of their size (see ProvisionedDevice).
          ProvisionedDevice.new(build, panel_size: spec.size == [800, 480] ? nil : spec.size)
        end
      end
      before { dev.reset }

      include Helpers

      include_examples "a BYOD board"
    end
  end

  # Helpers for a board's examples.
  module Helpers
    # Wait for the device to be in deep sleep with its display idle; returns its status.
    def wait_until_asleep(sim, timeout: 60) = sim.wait_for_deep_sleep(timeout:, display_idle: true)

    # Wake the sleeping device; the /api/display request it makes.
    def wake_request(sim) = dev.mock.next_request("/api/display", timeout: 60) { sim.wake }

    # Wake the sleeping device; wait for it to fetch `path`, finish the refresh and sleep.
    # Returns its status.
    def refresh(sim, path)
      dev.mock.next_request(path, timeout: 60) { sim.wake }
      wait_until_asleep(sim, timeout: 90)
    end

    # Serve a test image for this panel as the current screen; returns the screenshot it should
    # give.
    def serve_test_image(name = "test")
      w, h = board.size
      m = dev.mock
      expected =
        case board.inks
        when "spectra6" then m.set_spectra6_png(name, I.spectra_bars, w, h)
        when "bwry" then m.set_color_png(name, I.color_bars, w, h)
        else
          number = I.big_number("42", scale: [4, h / 30].max)
          checker = I.checkerboard([8, w / 20].max)
          bits = board.inks == "gray16" ? 4 : 1
          level = lambda { |x, y|
            if y < h / 2 ? number.(x, y) : checker.(x, y)
              0
            else
              (1 << bits) - 1
            end
          }
          m.set_png(name, level, width: w, height: h, bits:)
        end
      m.display = { image: name, refresh_rate: 300 }
      expected
    end
  end
end

RSpec.configure { |config| config.extend Byod::Macros }

# ---- the examples every board gets ---------------------------------------------------------------

RSpec.shared_examples "a BYOD board" do
  it "identifies itself" do
    dev.boot_asleep do |s|
      expect(s.status["board"]["name"]).to eq(board.name)
      wait_until_asleep(s)
      req = wake_request(s)
      expect(req).to have_header("Model", board.model)
      expect(req.headers.values_at("Width", "Height").map(&:to_i)).to eq(board.size)
      expect(req).to have_header("Battery-Voltage", a_value_within(0.06).of(board.battery_v)) if board.battery_v
      wait_until_asleep(s, timeout: 90)
    end
  end

  it "shows the served image" do
    expected = serve_test_image
    dev.boot_asleep do |s|
      wait_until_asleep(s)
      refresh(s, "/images/test.png")
      expect(s).to show_image(expected, tolerance: 16, max_ratio: 0.001)
    end
  end
end
