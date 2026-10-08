# frozen_string_literal: true

# Image formats and bad images on the device under test (see devices.rb; the TRMNL X has its own,
# core/trmnl_x/images_spec): PNG in every depth and color type, JPEG, BMP variants, and images the device must
# refuse. Images are the panel's size; what should be on screen is the same picture, reduced to what
# the panel shows.

General.describe "Images" do
  fixture(:dev) { ProvisionedDevice.new(build) }

  before { dev.reset }

  # The device's panel: the general groups' images are its size.
  let(:w) { device.width }
  let(:h) { device.height }
  let(:seven) { panel_number("7") }

  def digit(level_black, level_white, text = "7")
    px = panel_number(text)
    ->(x, y) { px.(x, y) ? level_black : level_white }
  end

  # The screenshot a `bits`-deep gray PNG of `level` should give on the device's panel.
  # The color panels' PNG decoders (png_draw_4clr / png_draw_6clr in display.cpp) take a gray
  # sample's raw bits as its RGB value (2-bit 0x00/0x40/0x80/0xC0, 1-bit 0x00/0x80, 4-bit doubled)
  # and pick the nearest ink; the others show the grays.
  def expected(level, bits)
    if %w[bwry spectra6].include?(device.inks)
      widen = { 1 => ->(v) { v << 7 }, 2 => ->(v) { v << 6 }, 4 => ->(v) { v * 17 }, 8 => ->(v) { v } }.fetch(bits)
      color = ->(x, y) { [widen.(level.(x, y))] * 3 }
      quantized = device.inks == "bwry" ? :expected_bwry : :expected_spectra6
      return TrmnlSim::Images.public_send(quantized, color, w, h)
    end
    TrmnlSim::Images.expected_gray(level, w, h, bits:)
  end

  def png(level, bits = 1, width: w, height: h) = TrmnlSim::Images.png_image(level, width, height, bits:)

  # A 1-bit bottom-up BMP, blank (index 0) unless `black.(x, y)` says which pixels are index 0 (the
  # rest are index 1).
  def bmp(width = w, height = h, palette: [0, 0, 0, 0, 255, 255, 255, 0].pack("C*"), offset: nil, magic: "BM".b,
          black: nil)
    row = ((width + 31) / 32) * 4
    data = Array.new(row * height, 0)
    black && height.times do |y|
      base = (height - 1 - y) * row
      width.times { |x| data[base + (x / 8)] |= 0x80 >> (x % 8) unless black.(x, y) }
    end
    data = data.pack("C*")
    off = offset || (14 + 40 + palette.bytesize)
    header = magic + [14 + 40 + palette.bytesize + data.bytesize, 0, 0, off].pack("VvvV")
    info = [40, width, height, 1, 1, 0, data.bytesize, 2835, 2835, 2, 2].pack("Vl<l<vvVVl<l<VV")
    header + info + palette + data
  end

  def zeros(n) = "\0".b * n
  def data_file(name) = File.binread(File.join(Builds::HERE, "data", name))

  # Wait until the device sleeps or restarts (a crash); returns its status.
  def settle(s, timeout: 120)
    deadline = Process.clock_gettime(Process::CLOCK_MONOTONIC) + timeout
    loop do
      st = s.status
      return st if st["boot_count"] > 1 || st["state"] == "halted"
      if st["state"] == "deep_sleep"
        return s.wait(state: "deep_sleep", display_idle: true, timeout: 30, settle_ms: 200)["status"]
      end
      if Process.clock_gettime(Process::CLOCK_MONOTONIC) > deadline
        raise TrmnlSim::TimeoutError, "device neither slept nor restarted: #{st}"
      end

      sleep 0.2
    end
  end

  # Make the next /api/display answers point at `data`; returns its path.
  def serve(name, data, content_type, **fields)
    path = "/img/#{name}"
    url = dev.mock.set_file(path, content_type, data)
    uid = Digest::SHA1.hexdigest(name)[0, 6]
    dev.mock.display = { image_url: url, filename: "plugin-#{uid}-#{Time.now.to_i}", refresh_rate: 300, **fields }
    path
  end

  # Boot, wait until the device has downloaded `path` and gone to sleep without crashing, and yield
  # the simulator (asleep) and its status.
  def boot_and_fetch(path)
    dev.boot do |s|
      dev.mock.wait_for_request(path, timeout: 90)
      st = settle(s)
      expect([st["boot_count"], st["state"]]).to eq([1, "deep_sleep"]), "the device crashed"
      yield s, st if block_given?
    end
  end

  # Boot, get `data` shown, and yield the simulator (asleep afterwards).
  def show(name, data, content_type, **fields, &) = boot_and_fetch(serve(name, data, content_type, **fields), &)

  # Serve an image the device must refuse (or survive): it sleeps and doesn't halt.
  def survives(name, data, content_type)
    show(name, data, content_type) { |s| expect(s.status["state"]).not_to eq("halted") }
  end

  # Serve the next image (with the refresh's `fields`) and wake the sleeping device to show it.
  def show_next(s, name, data, **fields)
    path = serve(name, data, "image/png", **fields)
    dev.mock.next_request(path, timeout: 90) { s.wake }
    s.wait(state: "deep_sleep", display_idle: true, timeout: 90)
  end

  describe "Png" do
    it "1bit png" do
      level = digit(0, 1)
      show("one.png", png(level), "image/png") do |s|
        expect(s).to show_image(expected(level, 1), tolerance: 64, max_ratio: 0)
      end
    end

    it "2bit png with two colors is drawn as 1bit" do
      level = digit(0, 3)
      show("two-colors.png", png(level, 2), "image/png") do |s|
        expect(s).to show_image(expected(level, 2), tolerance: 64, max_ratio: 0)
      end
    end

    it "2bit png uses 4 gray levels", :smoke do
      level = ->(x, y) { seven.(x, y) ? 0 : [3, x * 4 / w].min }
      show("gray4.png", png(level, 2), "image/png") do |s|
        expect(s).to show_image(expected(level, 2), tolerance: 48, max_ratio: 0.01)
      end
    end

    it "8bit gray png is reduced" do
      level = digit(0, 255)
      show("gray8.png", png(level, 8), "image/png") do |s|
        expect(s).to show_image(expected(level, 8), tolerance: 64, max_ratio: 0)
      end
    end

    it "4bit gray png is reduced" do
      # a known failure on the black-and-white panels (reduce_bpp_4bit)
      level = digit(0, 15)
      show("gray16.png", png(level, 4), "image/png") do |s|
        expect(s).to show_image(expected(level, 4), tolerance: 64, max_ratio: 0)
      end
    end

    it "palette png is reduced" do
      colors = [[0, 0, 0], [255, 255, 255]]
      data = TrmnlSim::Images.png_palette(->(x, y) { seven.(x, y) ? colors[0] : colors[1] }, colors, w, h)
      show("palette.png", data, "image/png") do |s|
        expect(s).to show_image(expected(digit(0, 1), 1), tolerance: 64, max_ratio: 0)
      end
    end

    it "truecolor png is reduced" do
      data = TrmnlSim::Images.png_rgb(->(x, y) { seven.(x, y) ? [0, 0, 0] : [255, 255, 255] }, w, h)
      show("rgb.png", data, "image/png") do |s|
        expect(s).to show_image(expected(digit(0, 1), 1), tolerance: 64, max_ratio: 0)
      end
    end

    it "portrait png is rotated" do
      px = panel_number("7")
      level = ->(x, y) { px.(y, h - 1 - x) ? 0 : 1 } # the digit, turned
      show("portrait.png", png(level, width: h, height: w), "image/png") do |s|
        expect(s.screenshot).not_to be_empty
      end
    end

    it "larger png is cropped" do
      px = panel_number("7")
      level = ->(x, y) { x < w && y < h && px.(x, y) ? 0 : 1 }
      show("large.png", png(level, width: w + 200, height: h + 120), "image/png") do |s|
        expect(s).to show_image(expected(digit(0, 1), 1), tolerance: 64, max_ratio: 0)
      end
    end

    it "corrupt png is not drawn" do
      data = png(digit(0, 1))
      survives("corrupt.png", data.byteslice(0, 40) + zeros(data.bytesize - 40), "image/png")
    end

    it "same png again is not downloaded or redrawn" do
      level = digit(0, 1)
      show("again.png", png(level), "image/png") do |s|
        refreshes = s.status["display_refreshes"]
        n = dev.mock.cursor
        dev.mock.next_request("/api/display", timeout: 90) { s.wake }
        s.wait(state: "deep_sleep", display_idle: true, timeout: 90, settle_ms: 200)
        expect(dev.mock.paths.drop(n)).to eq(["/api/display"])
        expect(s.status["display_refreshes"]).to eq(refreshes)
      end
    end

    it "new version of a plugin image replaces the cached one" do
      m = dev.mock
      old = png(digit(0, 1, "1"))
      new = png(digit(0, 1, "2"))
      m.set_file("/img/v1.png", "image/png", old)
      m.set_file("/img/v2.png", "image/png", new)
      m.display_queue = [{ image_url: "#{m.device_url}/img/v1.png", filename: "plugin-abc123-1000", refresh_rate: 300 }]
      m.display = { image_url: "#{m.device_url}/img/v2.png", filename: "plugin-abc123-2000", refresh_rate: 300 }
      dev.boot do |s|
        m.wait_for_request("/img/v1.png", timeout: 15)
        s.wait(state: "deep_sleep", display_idle: true, timeout: 15)
        c = s.status["console_total"]
        s.wake
        s.wait(console: /Deleting older version of plugin image/, since: c, timeout: 15)
        s.wait(state: "deep_sleep", display_idle: true, timeout: 15)
        expect(s).to show_image(expected(digit(0, 1, "2"), 1), tolerance: 64, max_ratio: 0)
      end
    end

    it "long filenames are shortened" do
      level = digit(0, 1)
      path = serve("long.png", png(level), "image/png")
      dev.mock.display[:filename] = "mashup-066cc3-weather-and-calendar-1771674964"
      dev.boot do |s|
        dev.mock.wait_for_request(path, timeout: 15)
        s.wait(state: "deep_sleep", display_idle: true, timeout: 15)
        expect(s).to show_image(expected(level, 1), tolerance: 64, max_ratio: 0)
      end
    end

    it "temperature profile is saved" do
      level = digit(0, 1)
      show("temp.png", png(level), "image/png", temperature_profile: "a") do |s|
        expect(s).to show_image(expected(level, 1), tolerance: 64, max_ratio: 0)
        show_next(s, "temp2.png", png(digit(0, 1, "8")), temperature_profile: "b")
        expect(s).to show_image(expected(digit(0, 1, "8"), 1), tolerance: 64, max_ratio: 0)
      end
    end

    it "maximum compatibility forces full refreshes" do
      level = digit(0, 1)
      show("compat.png", png(level), "image/png", maximum_compatibility: true) do |s|
        expect(s).to show_image(expected(level, 1), tolerance: 64, max_ratio: 0)
      end
    end

    it "long refresh rates use fast instead of partial refreshes" do
      show("slow1.png", png(digit(0, 1)), "image/png", refresh_rate: 3600) do |s|
        eight = digit(0, 1, "8")
        show_next(s, "slow2.png", png(eight), refresh_rate: 3600)
        expect(s).to show_image(expected(eight, 1), tolerance: 64, max_ratio: 0)
      end
    end
  end

  describe "Jpeg" do
    it "jpeg is dithered to 1bit" do
      pending "bb_epaper leaves the UC81xx panel in partial mode after a JPEG (screen stays blank)"
      show("five.jpg", data_file("five_#{w}x#{h}.jpg"), "image/jpeg") do |s|
        # dithering edges: up to 2% of an 800x480 screen, as many pixels on other panels (a share
        # of a big panel would let a blank screen pass)
        expect(s).to show_image(expected(digit(0, 1, "5"), 1), tolerance: 64, max_ratio: [0.02, 7680.0 / (w * h)].min)
      end
    end

    it "jpeg of the wrong size is refused" do
      survives("small.jpg", data_file("five_640x480.jpg"), "image/jpeg") # no panel is 640x480
    end
  end

  describe "Bmp" do
    it "bmp with an inverted palette" do
      # parseBMPHeader recognizes the white/black palette ("Color scheme reversed") and sets
      # image_reverse, but nothing uses it (display_show_image's inversion is under #ifdef
      # FUTURE): the image is shown inverted.
      level = digit(0, 1)
      # index 0 = white, 1 = black
      data = bmp(palette: [255, 255, 255, 0, 0, 0, 0, 0].pack("C*"), black: ->(x, y) { !seven.(x, y) })
      show("inverted.bmp", data, "image/bmp") do |s|
        expect(s).to show_image(expected(level, 1), tolerance: 64, max_ratio: 0)
      end
    end

    it "not a bmp" do
      survives("garbage.bmp", "XX".b + zeros(bmp.bytesize - 2), "image/bmp")
    end

    it "bmp of the wrong size" do
      survives("small.bmp", bmp(w / 2, h / 2), "image/bmp")
    end

    it "bmp with a color palette" do
      survives("red.bmp", bmp(palette: [0, 0, 255, 0, 255, 255, 255, 0].pack("C*")), "image/bmp")
    end

    it "bmp with a bad data offset" do
      survives("offset.bmp", bmp(offset: 100), "image/bmp")
    end
  end

  describe "Refused" do
    it "image too large to download" do
      boot_and_fetch(serve("huge.png", zeros(device.max_image + 5000), "image/png"))
    end

    it "empty image" do
      boot_and_fetch(serve("empty.png", "".b, "image/png"))
      # Content-Length 0 takes the "no Content-Length" path (contentLength <= 0): if the server's
      # close has already arrived, writeToStream() reports the download as cut short; otherwise it
      # reads nothing and finishBody() says "No data received".
      log = dev.mock.wait_for_request("/api/log", timeout: 10)
      expect(log.body).to match(/No data received|connection closed mid-download/)
    end

    it "missing image" do
      dev.mock.display = { image_url: "#{dev.mock.device_url}/img/missing.png", filename: "plugin-000000-1",
                           refresh_rate: 300 }
      boot_and_fetch("/img/missing.png") do |_, st|
        expect(st["wake_at_s"] - st["sim_time_s"]).to be < 300
      end
    end
  end

  # The TRMNL BWRY's own examples: they run whenever it is listed, not only with :full.
  next unless device.env == "trmnl_4clr"

  describe "BwryPng", general: :own do
    # The TRMNL BWRY's 800x480 panel.
    let(:og_seven) { TrmnlSim::Images.big_number("7") }

    def og_digit(level_black, level_white, text = "7")
      px = TrmnlSim::Images.big_number(text)
      ->(x, y) { px.(x, y) ? level_black : level_white }
    end

    it "1bit png" do
      level = og_digit(0, 1)
      show("one.png", png(level, width: 800, height: 480), "image/png") do |s|
        expect(s).to show_image(TrmnlSim::Images.expected_gray(level, 800, 480), tolerance: 64, max_ratio: 0)
      end
    end

    it "8bit gray png" do
      level = og_digit(0, 255)
      show("gray8.png", png(level, 8, width: 800, height: 480), "image/png") do |s|
        expect(s).to show_image(TrmnlSim::Images.expected_gray(level, 800, 480, bits: 8), tolerance: 64, max_ratio: 0)
      end
    end

    it "truecolor png" do
      # trmnl_4clr doesn't set PNG_MAX_BUFFERED_PIXELS (trmnl does), so PNGdec's default row
      # buffer is too small for 800 px truecolor rows: the decode corrupts the heap (store fault
      # in tlsf_free), and the device crashes, restarts, downloads the same image and crashes
      # again, until the server sends something else.
      survives("rgb.png", TrmnlSim::Images.png_rgb(->(x, y) { og_seven.(x, y) ? [255, 0, 0] : [255, 255, 255] }),
               "image/png")
    end

    it "jpeg" do
      survives("five.jpg", data_file("five_800x480.jpg"), "image/jpeg")
    end
  end
end
