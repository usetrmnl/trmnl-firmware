# frozen_string_literal: true

# Image formats on the TRMNL X (1872x1404, 16 gray levels): PNG depths, portrait images,
# JPEG, BMP, and the image cache.

data_dir = File.join(Builds::HERE, "data")
w = 1872
h = 1404

RSpec.describe "TRMNL X images", env: "TRMNL_X" do
  fixture(:shipped) { TrmnlX::ShippedX.new }
  fixture(:dev) { TrmnlX::ProvisionedX.new(shipped) }

  before { dev.reset }

  # `text` in gray level `black` on `white`.
  def levels(black, white, text = "7")
    px = TrmnlX.digits(text)
    ->(x, y) { px.(x, y).zero? ? black : white }
  end

  # Serve `data` as the next screen's image_url, wake the device (asleep) to show it, and yield
  # the simulator once it is back asleep; returns the block's value.
  def show(name, data, content_type, timeout: 15)
    m = dev.mock
    url = m.set_file("/img/#{name}", content_type, data)
    m.display = { image_url: url, filename: format("plugin-%06d-1000", name.hash.abs % 999_999), refresh_rate: 300 }
    dev.boot_asleep do |s|
      s.wait(state: "deep_sleep", timeout: 15)
      s.wake
      m.wait_for_request("/img/#{name}", timeout: 15)
      st = s.wait(state: "deep_sleep", timeout:, settle_ms: 300)["status"]
      expect(st["state"]).not_to eq("halted")
      yield s if block_given?
    end
  end

  describe "XPng" do
    [[2, 3, "gray4.png"], [8, 255, "gray8.png"]].each do |bits, white, name|
      it "#{bits}bit png" do
        level = levels(0, white)
        show(name, TrmnlSim::Images.png_image(level, w, h, bits:), "image/png") do |s|
          expect(s).to show_image(TrmnlSim::Images.expected_gray(level, w, h, bits:), tolerance: 64, max_ratio: 0.001)
        end
      end
    end

    { 1 => 1, 4 => 15 }.each do |bits, white|
      it "portrait #{bits}bit png" do
        px = TrmnlX.digits("7")
        level = ->(x, y) { px.(y, w - 1 - x).zero? ? 0 : white }
        show("portrait#{bits}.png", TrmnlSim::Images.png_image(level, h, w, bits:), "image/png")
      end
    end
  end

  describe "XModemDownload" do
    it "long url slow download" do
      # An image URL too long for AT+HTTPCLIENT goes through AT+HTTPURLCFG; at 4 kB/s the
      # download takes a few seconds, so the modem driver reports its progress.
      m = dev.mock
      m.set_fault("/img/*", rate: 4000)
      name = "long-#{'x' * 240}.png"
      expected = m.set_png("tmp", TrmnlX.digits("4"))
      # (The driver's Serial lines aren't in a production build's log.)
      show(name, m.images.delete("tmp.png"), "image/png") do |s|
        expect(s).to show_image(expected, tolerance: 64, max_ratio: 0)
      end
    end
  end

  describe "XModemLimits" do
    it "image larger than the download buffer" do
      # MAX_IMAGE_SIZE is 750000 bytes on the X; the modem download stops past it.
      # (It is downloaded, and refused, 5 times: a too-big file is retried like any error.)
      # (The error log isn't submitted: on 5 GHz only the modem is connected, and stored
      # logs go out over the S3's own WiFi.)
      console = show("huge.png", "\0".b * 800_000, "image/png", timeout: 120) { |s| s.console(0).join("\n") }
      expect(console).not_to include("Guru Meditation")
      expect(console).to include("HTTPS_IMAGE_FILE_TOO_BIG - file size too big: more than 750000 bytes")
    end

    it "empty image" do
      console = show("empty.png", "".b, "image/png") { |s| s.console(0).join("\n") }
      # The modem driver treats a response without data as a failed request, so this never
      # gets to finishBody()'s "No data received" either.
      expect(console).to include("HTTPS_RESPONSE_CODE_INVALID - modem HTTP status -1, 0 bytes received")
    end
  end

  describe "XJpeg" do
    it "jpeg is dithered to 16 grays" do
      show("five.jpg", File.binread(File.join(data_dir, "x_five_1872x1404.jpg")), "image/jpeg") do |s|
        expect(s).to show_image(TrmnlSim::Images.expected_gray(levels(0, 1, "5"), w, h), tolerance: 96, max_ratio: 0.05)
      end
    end
  end

  describe "XCache" do
    it "same image again is not downloaded or redrawn" do
      m = dev.mock
      m.set_png("one", TrmnlX.digits("1"))
      m.display = { image: "one", refresh_rate: 300 }
      dev.boot_asleep do |s|
        s.wait(state: "deep_sleep", timeout: 15)
        s.wake
        m.wait_for_request("/images/one.png", timeout: 15)
        s.wait(state: "deep_sleep", timeout: 15, settle_ms: 300)
        refreshes = s.status["display_refreshes"]
        n = m.cursor
        m.next_request("/api/display", timeout: 15) { s.wake }
        s.wait(state: "deep_sleep", timeout: 15, settle_ms: 300)
        expect(m.paths.drop(n)).to eq(["/api/display"])
        expect(s.status["display_refreshes"]).to eq(refreshes)
      end
    end
  end
end
