# frozen_string_literal: true

require "date"

# What the TRMNL X touch bar specs (spec/core/trmnl_x/touchbar_*_spec.rb) share: touch helpers
# and the provisioned X they run on, with images in its cache.
module Touchbar
  # Touch the zone for `ms`, wait for the firmware's console `line` about it, and for the
  # device to be back asleep (20 s of virtual time after the touch, see touchbar_stay_awake).
  def touch_and_sleep(s, zone, ms, line)
    c = s.status["console_total"]
    s.touch(zone, ms:)
    s.wait(console: line, since: c, timeout: 15)
    s.wait(state: "deep_sleep", timeout: 30, settle_ms: 500)
  end

  # How trmnl.app answers a rewind from firmware with guest mode (FIRMWARE_SPECIAL_FUNCTIONS in core's
  # Device::Functionable): the previous screen, as action guest_mode, with no function to save.
  def previous_screen(name) = { image: name, action: "guest_mode", refresh_rate: 300, special_function: nil }

  # The pixels by which the screen differs from `image`, that an arrow indicator (at most its
  # 172 x 76 with the border) can account for: 0 without one, and fails on any other difference.
  def arrow_pixels(s, image)
    pixels = s.compare_screen(image, tolerance: 64, max_ratio: 0)["diff_pixels"]
    expect(pixels).to be <= 172 * 76
    pixels
  end

  # Wake the X, or use it awake, into the touch menu, choose Close and wait for the redraw.
  def close_menu(s)
    c = s.status["console_total"]
    s.touch("center", ms: 150)
    s.wait(console: /menu_show_screen end/, since: c, timeout: 30)
    TrmnlX.menu_tap(s, "right", /Next Screen -> Previous Screen/)
    TrmnlX.menu_tap(s, "right", /Previous Screen -> Touchbar Tutorial/)
    TrmnlX.menu_tap(s, "right", /Touchbar Tutorial -> Special/)
    TrmnlX.menu_tap(s, "right", /Special\.\.\. -> Settings/)
    TrmnlX.menu_tap(s, "right", /Settings\.\.\. -> Close/)
    c = s.status["console_total"]
    s.touch("center", ms: 150)
    s.wait(console: /chose Close/, since: c, timeout: 15)
    s.wait(console: /display_show_image end/, since: c, timeout: 15) # the screen it covered is back
  end

  # Serve the X-sized digits `text` as images/<name>.png; returns the expected screen. Rendering
  # a 1872x1404 image takes Ruby about a second, so each is made once per process.
  def set_digits(mock, name, text)
    png, expected = Touchbar.digits(text)
    mock.images["#{name}.png"] = png
    mock.stamp(name)
    expected
  end

  @digits = {}

  # The PNG and expected screen of the digits `text`.
  def self.digits(text)
    level = TrmnlX.digits(text)
    @digits[text] ||= [TrmnlSim::Images.png_image(level, 1872, 1404, bits: 1),
                       TrmnlSim::Images.expected_gray(level, 1872, 1404, bits: 1)]
  end

  # Needs the `dev` fixture (a TrmnlX::ProvisionedX).
  RSpec.shared_context "provisioned X" do
    include Touchbar

    before { dev.reset }

    # The provisioned X asleep showing "2", with "1" before it in its image cache; yields the
    # simulator and their expected screens. Showing them (a boot and two downloads, decodes and
    # refreshes) is done once and kept as a save point in the setup cache, keyed by the day too:
    # the X purges cached images over 24 h old.
    def boot_two_images
      m = dev.mock
      inputs = { day: Date.today.iso8601, support: SetupCache.file_hash(__FILE__) }
      one = set_digits(m, "one", "1")
      two = set_digits(m, "two", "2")
      m.display_queue << { image: "one", refresh_rate: 300 } # when the state is made
      m.display = { image: "two", refresh_rate: 300 }
      state = dev.cached_state("two-images", inputs) do |s|
        m.wait_for_request("/images/one.png", timeout: 15)
        s.wait(state: "deep_sleep", timeout: 15)
        s.wake
        m.wait_for_request("/images/two.png", timeout: 15)
        s.wait(state: "deep_sleep", timeout: 15, settle_ms: 500)
      end
      dev.reset(display: m.display)
      dev.restore(state) do |s|
        s.wait(state: "deep_sleep", timeout: 15) # the save point is loaded
        # as the server named them when the state was made, not now: those are the files it has cached
        s.preferences.fetch("playlist_order").split("|").each do |file|
          name = %w[one two].find { file.start_with?("/#{m.filenames[_1][0, 13]}") }
          m.filenames[name] = file.delete_prefix("/")
        end
        yield s, one, two
      end
    end
  end
end
