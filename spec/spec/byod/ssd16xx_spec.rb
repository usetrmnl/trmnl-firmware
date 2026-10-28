# frozen_string_literal: true

# BYOD boards with SSD16xx panels: the 4.26" 800x480 (Xteink X4, TRMNL 4.26" DIY kit), the 3.97"
# 800x480 (Waveshare ESP32-S3 3.97", Seeed Sticky) and the 4.2" 400x300 (CrowPanel).

images = TrmnlSim::Images

RSpec.describe "BYOD SSD16xx boards" do
  # What the SSD16xx boards have beyond the shared examples. `button_source`: Update-Source
  # after a button wake: ESP32-S3 boards wake by EXT0, C3 ones by GPIO.
  shared_examples "an SSD16xx board" do |button_source: "EXT0"|
    # The screenshot a `bits`-deep image of `level` should give on this board.
    define_method(:expected_screen) do |level, bits|
      w, h = board.size
      images.expected_gray(level, w, h, bits:)
    end

    # Four vertical bars, black to white, and a black/white strip along the bottom.
    def gray_level
      w, h = board.size
      lambda { |x, y|
        if y >= h - (h / 8)
          (x / 16).odd? ? 0 : 3
        else
          x * 4 / w
        end
      }
    end

    # A different picture per name, so every refresh changes pixels.
    def serve_test_image(name = "test")
      return super if name == "test"

      w, h = board.size
      seed = name.bytes.sum
      size = 8 + (seed % 24)
      level = ->(x, y) { (((x + seed) / size) + (y / size)).odd? ? 0 : 1 }
      dev.mock.images["#{name}.png"] = TrmnlSim::Images.png_image(level, w, h)
      dev.mock.stamp(name)
      dev.mock.display = { image: name, refresh_rate: 300 }
      expected_screen(level, 1)
    end

    it "shows a 4-gray image" do
      w, h = board.size
      dev.mock.images["gray.png"] = images.png_image(gray_level, w, h, bits: 2)
      dev.mock.stamp("gray")
      dev.mock.display = { image: "gray", refresh_rate: 300 }
      dev.boot_asleep do |s|
        wait_until_asleep(s)
        refresh(s, "/images/gray.png")
        expect(s).to show_image(expected_screen(gray_level, 2), tolerance: 16, max_ratio: 0.001)
      end
    end

    it "partial refresh after a full one" do
      # 1-bit images refresh partially (differential) once the panel holds one; each must end
      # up exactly on screen.
      dev.boot_asleep do |s|
        wait_until_asleep(s)
        %w[first second third].each do |name|
          expected = serve_test_image(name)
          refresh(s, "/images/#{name}.png")
          aggregate_failures(name) { expect(s).to show_image(expected, tolerance: 16, max_ratio: 0.001) }
        end
      end
    end

    it "button wakes it" do
      dev.boot_asleep do |s|
        wait_until_asleep(s)
        req = dev.mock.next_request("/api/display", timeout: 60) { s.press(150) }
        expect(req).to have_header("Update-Source", button_source)
        wait_until_asleep(s, timeout: 90)
      end
    end

    # Firmware bug: display_show_image() writes 1-bit BMP (and Group5) images to the SSD16xx's
    # new-image RAM only (writePlane() = PLANE_BOTH without a second plane) and asks for a
    # partial refresh. SSD16xx partial refreshes are differential: they drive only the pixels
    # where the new image differs from the "old" RAM (0x26), which the controller updates itself
    # only after a partial refresh. After a fast or full refresh of a PNG it holds the inverted
    # PNG (PLANE_FALSE_DIFF): every pixel that should change counts as unchanged and the old
    # picture stays up. (On the Sticky, whose panel supply is off during deep sleep, it holds
    # nothing: only the white pixels get drawn.) PNGs are fine: png_to_epd() writes both RAMs
    # every time.
    it "bmp after a fast refresh" do
      skip "the mock serves 800x480 BMPs" unless board.size == [800, 480]

      m = dev.mock
      dev.boot_asleep do |s|
        wait_until_asleep(s)
        serve_test_image("png")
        m.display[:refresh_rate] = 3600 # 30 min or more: fast instead of partial refreshes
        refresh(s, "/images/png.png")
        seven = images.big_number("7", scale: 20)
        m.set_image("bmp", seven)
        expected = expected_screen(->(x, y) { seven.(x, y) ? 0 : 1 }, 1)
        m.display = { image: "bmp", refresh_rate: 300 }
        refresh(s, "/images/bmp.bmp")
        expect(s).to show_image(expected, tolerance: 16, max_ratio: 0.001)
      end
    end
  end

  describe "XteinkX4", env: "xteink_x4" do
    byod_board name: "Xteink X4", model: "xteink_x4",
               battery_v: 0.0 # device_list[] has batt_pin 0xff
    it_behaves_like "an SSD16xx board", button_source: "button"

    # Firmware bug: the X4's battery divider is on GPIO0 (config.h: PIN_BATTERY 0 for
    # BOARD_XTEINK_X4), but its device_list[] row has batt_pin 0xff, so readVoltage() reads no
    # ADC pin and the X4 always reports 0 V.
    it "reports the battery voltage" do
      dev.boot_asleep do |s|
        s.set_battery(3800)
        wait_until_asleep(s)
        expect(wake_request(s)).to have_header("Battery-Voltage", a_value_within(0.06).of(3.8))
        wait_until_asleep(s, timeout: 90)
      end
    end
  end

  describe "DiyKit426", env: "TRMNL_4inch26_DIY_Kit" do
    byod_board name: "TRMNL 4.26\" DIY Kit", model: "xiao_epaper_mini"
    it_behaves_like "an SSD16xx board"
  end

  describe "Waveshare397", env: "WAVESHARE_397" do
    # Firmware (bb_epaper 2.1.9) bug: EP397_800x480's init sequences make the RAM Y address
    # count down from 479 (data entry mode 0x01, window 479..0) but start the counter at 0
    # (0x4F 0x00 0x00) instead of 479. The first row lands on RAM row 0, the next ones on 479,
    # 478, ...: the picture is one row too high, its top row at the bottom.
    byod_board name: "Waveshare ESP32-S3 3.97\"", model: "waveshare_397"
    it_behaves_like "an SSD16xx board"

    it "reports the battery from the pmic" do
      dev.boot_asleep do |s|
        s.set_battery(3650)
        wait_until_asleep(s)
        expect(wake_request(s)).to have_header("Battery-Voltage", a_value_within(0.02).of(3.65))
        wait_until_asleep(s, timeout: 90)
      end
    end
  end

  describe "SeeedSticky", env: "seeed_sticky" do
    byod_board name: "Seeed Sticky", model: "seeed_sticky"
    # Firmware (bb_epaper 2.1.11, the Sticky's pinned version) bug: EP397_800x480_4GRAY now
    # writes a custom 4-gray LUT (0x32) in its init sequence, but bbepRefresh() still starts
    # 4-gray refreshes with 0x22 0xD7, whose "load LUT" bit reloads the built-in LUT over it (the
    # 4.26" and 4.2" panels use 0xC7 / 0xCF, which don't). The built-in waveform shows the two
    # gray planes as black and white.
    it_behaves_like "an SSD16xx board"

    it "reports the battery from the gauge" do
      dev.boot_asleep do |s|
        s.set_battery(3650)
        wait_until_asleep(s)
        expect(wake_request(s)).to have_header("Battery-Voltage", a_value_within(0.02).of(3.65))
        wait_until_asleep(s, timeout: 90)
      end
    end
  end

  describe "CrowPanel42", env: "CrowPanel42" do
    # Firmware bug: the CrowPanel's device_list[] row has no pins, so display.cpp brings the
    # panel up with bbep.begin(<product>) (EPD_CROWPANEL42, which selects EP42B_400x300), but
    # png_to_epd() then calls bbep.setPanelType(dpList[...].OneBit) for 1-bit PNGs regardless,
    # passing that product number as a panel type: 8 = EP295_128x296_4GRAY. The image is decoded
    # 128 pixels wide into the 400x300 RAM and refreshed with that 2.9" panel's init sequence and
    # (SSD1680-format) LUT, so it never shows. 2-bit images take the begin() path and are fine.
    byod_board name: "CrowPanel 4.2\"", model: "crowpanel42", size: [400, 300],
               battery_v: 4.2 # BATT_NONE: a fixed 4.2 V
    it_behaves_like "an SSD16xx board"
  end
end
