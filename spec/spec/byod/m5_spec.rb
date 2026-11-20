# frozen_string_literal: true

# BYOD boards whose panel and wiring are built into bb_epaper (`bbep.begin()`), in the firmware's
# `#ifdef CMD_CS1_CS2` rows: the M5Paper Mono (3.97" 800x480 SSD1677, its supply and RST on an
# M5IOE1 I/O expander at I2C 0x4f), the M5Paper Color (4" 400x600 Spectra 6, powered through the
# board's PY32 at I2C 0x6e) and the Seeed reTerminal E1004 (13.3" 1200x1600 Spectra 6 driven by
# two controllers, one per half, on two chip selects).

rgb = TrmnlSim::Images::SPECTRA6_RGB
inks = rgb.keys # black, white, yellow, red, blue, green

RSpec.describe "BYOD bb_epaper boards" do
  describe "M5PaperMono", env: "m5_paper_mono" do
    byod_board name: "M5Paper Mono", model: "m5_paper_mono",
               battery_v: 4.2 # BATT_NONE

    # A 2-bit PNG: bb_epaper's 4-gray mode (EPD_M5_PAPER_MONO_4GRAY, a waveform in the LUT
    # register) gives black, two grays and white.
    it "four grays" do
      w, h = board.size
      # four vertical bands, and a checker of all four at the bottom
      level = ->(x, y) { y >= 400 ? ((x / 40) + (y / 40)) % 4 : x * 4 / w }
      expected = dev.mock.set_png("grays", level, width: w, height: h, bits: 2)
      dev.mock.display = { image: "grays", refresh_rate: 300 }
      dev.boot_asleep do |s|
        wait_until_asleep(s)
        refresh(s, "/images/grays.png")
        expect(s).to show_image(expected, tolerance: 16, max_ratio: 0.001)
      end
    end
  end

  describe "M5PaperColor", env: "m5_paper_color" do
    # USB CDC on boot and WAIT_FOR_SERIAL: the firmware waits up to 2 s for a host
    byod_board name: "M5Paper Color", model: "m5_paper_color", size: [400, 600], # portrait
               battery_v: 4.2, # BATT_NONE
               inks: "spectra6"

    # Six horizontal bands, one per ink, with a checker of all six at the bottom: rows and
    # columns of the portrait panel land where the image has them.
    it "every ink in portrait" do
      w, h = board.size
      color = ->(x, y) { y >= 540 ? rgb[inks[((x / 20) + (y / 20)) % 6]] : rgb[inks[y / 90]] }
      expected = dev.mock.set_spectra6_png("bands", color, w, h)
      dev.mock.display = { image: "bands", refresh_rate: 300 }
      dev.boot_asleep do |s|
        wait_until_asleep(s)
        refresh(s, "/images/bands.png")
        expect(s).to show_image(expected, tolerance: 16, max_ratio: 0)
      end
    end
  end

  describe "ReTerminalE1004", env: "seeed_reTerminal_E1004" do
    byod_board name: "reTerminal E1004", model: "reterminal_e1004", size: [1200, 1600], inks: "spectra6",
               png_default: true # the mock's usual 800x480 BMP crashes it ("takes an 800x480 bmp")

    # Firmware bug: display_show_image() un-flips an uncompressed BMP in place with
    # flip_image(image_buffer + 62, bbep.width(), bbep.height()), i.e. assuming the BMP is
    # panel-sized, and then sends it as the panel's plane (writePlane reads a 4-bit 1200x1600
    # plane, 960 KB, from it). An 800x480 1-bit BMP (48 KB) on this panel makes flip_image write
    # ~190 KB past the download buffer. Seen during onboarding with the mock's default BMP: the
    # heap's free-block headers get overwritten (with 0x11, white Spectra pixels) and the device
    # panics in HttpRetryRequest::releaseBody -> free(), then reboots, fetches the same image and
    # crashes again. Whether it crashes depends on the heap layout, so this example catches the
    # overflow with --memcheck. (The E1002 skips writePlane for BMPs; the X's flip_image overflows
    # the same way, see core/trmnl_x/memcheck_spec.rb.)
    it "takes an 800x480 bmp" do
      m = dev.mock
      m.set_image("bmp") { |x, y| ((x / 40) + (y / 40)).even? }
      m.display = { image: "bmp", refresh_rate: 300 }
      dev.boot_asleep(memcheck: "log") do |s|
        wait_until_asleep(s)
        refresh(s, "/images/bmp.bmp")
        expect(s.log).not_to include("Guru Meditation")
        expect(s.memcheck["violations"]).to eq([])
      end
    end

    # Different content left and right of the seam at x = 600, and bands that cross it: the left
    # controller (CS 10) shows the left half, the right one (CS 2) the right.
    it "each controller shows its half" do
      w, h = board.size
      color = lambda do |x, y|
        if y < 400 then rgb[inks[x * 6 / w]] # six bars across the whole width
        elsif y < 800 then rgb[x < w / 2 ? :red : :blue] # one ink per half
        elsif y < 1200 then rgb[inks[((x / 50) + (y / 50)) % 6]] # a checker straddling the seam
        else
          rgb[x < 590 || x >= 610 ? :green : :black] # a thin seam line
        end
      end
      expected = dev.mock.set_spectra6_png("halves", color, w, h)
      dev.mock.display = { image: "halves", refresh_rate: 300 }
      dev.boot_asleep do |s|
        refreshes = wait_until_asleep(s)["display_refreshes"]
        status = refresh(s, "/images/halves.png")
        expect(s).to show_image(expected, tolerance: 16, max_ratio: 0)
        # one refresh for the image: both controllers refresh together
        expect(status["display_refreshes"] - refreshes).to eq(1)
      end
    end
  end
end
