# frozen_string_literal: true

# BYOD boards with other UltraChip UC81xx panels: the 7.5" black/white/red TRMNL DIY kit, the
# TRMNL Steam (5.83" 648x480) and the Xteink X3 (3.68" 792x528, BQ27220 fuel gauge).

images = TrmnlSim::Images

# Firmware bug: the trmnl_steam row of device_list[] (and EP583_648x480 in dpList) sits inside
# `#ifdef CMD_CS1_CS2`, which the bb_epaper this env pins (9181692) doesn't define.
# hw_config_init() doesn't find DEVICE_MODEL, logs the NULL name with %s (strlen(NULL): load
# access fault) and pDevice stays NULL; the device panics and reboots endlessly before bringing
# up the portal.

RSpec.describe "BYOD UC81xx boards" do
  # Does a factory-fresh device come up with its captive portal?
  def boots_to_the_portal(env, timeout: 30)
    sim(Builds.for_env(env), erase: true, extra_args: ["--offline"]) do |s|
      s.wait(portal: true, timeout:)
      true
    end
  rescue StandardError
    false
  end

  describe "DiyKitBwr", env: "TRMNL_7inch5_OG_DIY_Kit_3CLR" do
    # The firmware shows 1-bit images on this panel (black/white plane, red plane cleared).
    byod_board name: "TRMNL 7.5\" BWR DIY Kit", model: "xiao_epaper_3clr"

    it "refresh takes the panel's long color update" do
      serve_test_image
      dev.boot_asleep do |s|
        wait_until_asleep(s)
        expect(s.status["board"]["has_refresh_flashing"]).to be_truthy
        dev.mock.next_request("/images/test.png", timeout: 60) { s.wake }
        t0 = s.status["sim_time_s"]
        st = wait_until_asleep(s, timeout: 90)
        expect(st["sim_time_s"] - t0).to be > 15 # the OTP color update alone is ~16 s
      end
    end

    # Firmware bug: display.cpp has no black/white/red image path (GetBWRPixel is never called).
    # A 3-color palette PNG has 2 bits per pixel and more than two colors, so png_to_epd takes
    # the 4-gray path and writes the gray bit planes into the panel's black/white and red planes:
    # white comes out black, black and red come out red.
    it "shows red" do
      expected = dev.mock.set_bwr_png("bwr", images.bwr_bars)
      dev.mock.display = { image: "bwr", refresh_rate: 300 }
      dev.boot_asleep do |s|
        wait_until_asleep(s)
        refresh(s, "/images/bwr.png")
        expect(s).to show_image(expected, tolerance: 16, max_ratio: 0.001)
      end
    end

    # Firmware bug (as above): a 4-gray PNG goes down the 4-gray path, whose plane 0/1 split
    # lands in DTM1 (black/white) and DTM2 (red) of this panel: black and dark gray come out red,
    # light gray white and white black. The least it should do is threshold the grays to black
    # and white.
    it "4 gray png is shown in black and white" do
      level = ->(x, _y) { [3, x * 4 / 800].min }
      dev.mock.images["gray4.png"] = images.png_image(level, 800, 480, bits: 2)
      dev.mock.stamp("gray4")
      dev.mock.display = { image: "gray4", refresh_rate: 300 }
      dev.boot_asleep do |s|
        wait_until_asleep(s)
        refresh(s, "/images/gray4.png")
        expected = images.expected_gray(->(x, y) { level.(x, y) >> 1 }, 800, 480, bits: 1)
        expect(s).to show_image(expected, tolerance: 16, max_ratio: 0.001)
      end
    end
  end

  describe "TrmnlSteamBoots", env: "trmnl_steam" do
    it "comes up with the portal" do
      expect(boots_to_the_portal("trmnl_steam")).to be(true), "no portal: it boot-loops"
    end
  end

  describe "TrmnlSteam", env: "trmnl_steam" do
    byod_board name: "TRMNL Steam", model: "trmnl_steam", size: [648, 480]
  end

  describe "XteinkX3", env: "xteink_x3" do
    byod_board name: "Xteink X3", model: "xteink_x3", size: [792, 528]

    it "battery voltage comes from the fuel gauge" do
      dev.boot_asleep do |s|
        wait_until_asleep(s)
        s.set_battery(3720)
        expect(wake_request(s)).to have_header("Battery-Voltage", a_value_within(0.02).of(3.72))
        wait_until_asleep(s, timeout: 90)
      end
    end

    it "2bit png uses 4 gray levels" do
      w, h = board.size
      level = ->(x, y) { (200...260).cover?(y) ? 0 : [3, x * 4 / w].min }
      expected = dev.mock.set_png("gray4", level, width: w, height: h, bits: 2)
      dev.mock.display = { image: "gray4", refresh_rate: 300 }
      dev.boot_asleep do |s|
        wait_until_asleep(s)
        refresh(s, "/images/gray4.png")
        expect(s).to show_image(expected, tolerance: 48, max_ratio: 0.01)
      end
    end

    # Firmware bug: display_show_image flips an uncompressed BMP in place with the display's size
    # (flip_image(image_buffer+62, bbep.width(), bbep.height())) without checking the BMP's.
    # 792x528 needs 52272 bytes; an OG-sized 800x480 BMP has 48000, so the flip writes past the
    # download buffer, corrupting the heap: the device panics (tlsf_free) and reboots, and
    # repeats that on every wake.
    it "survives an 800x480 bmp" do
      dev.mock.set_image("og") { |x, y| ((x / 40) + (y / 40)).even? }
      dev.mock.display = { image: "og", refresh_rate: 300 }
      dev.boot_asleep do |s|
        wait_until_asleep(s)
        boots = s.status["boot_count"]
        refresh(s, "/images/og.bmp")
        expect(s.status["boot_count"]).to eq(boots)
      end
    end
  end
end
