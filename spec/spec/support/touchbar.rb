# frozen_string_literal: true

require "date"

# What the TRMNL X touch bar specs (spec/core/trmnl_x/touchbar_*_spec.rb) share: touch helpers
# and the provisioned X they browse cached images on.
module Touchbar
  # Touch the zone for `ms`, wait for the firmware's console `line` about it, and for the
  # device to be back asleep.
  def touch_and_sleep(s, zone, ms, line)
    c = s.status["console_total"]
    s.touch(zone, ms:)
    s.wait(console: line, since: c, timeout: 15)
    s.wait(state: "deep_sleep", timeout: 15, settle_ms: 500)
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
        yield s, one, two
      end
    end
  end
end
