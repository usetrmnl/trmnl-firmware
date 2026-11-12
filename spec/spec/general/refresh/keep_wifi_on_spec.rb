# frozen_string_literal: true

# An answer with keep_wifi_on keeps the device awake with Wi-Fi connected until its next check-in,
# refresh_rate seconds after the answer arrived, instead of deep sleeping (src/bl.cpp, bl_init).

General.describe "Keep Wi-Fi on", skip_if: :touchbar,
                                  why: "the X stays awake for short refreshes in its touch bar loop" do
  fixture(:dev) { ProvisionedDevice.new(build) }

  before { dev.reset }

  def wifi_joins(sim) = sim.console(0).count { _1.include?("[sim] wifi: connecting to") }

  # A PNG is cached under its filename (display_session.cpp), so a repeated filename skips the
  # download; a BMP is kept only as /current.bmp and downloaded again.
  def serve_png(name, number)
    w, h = device.size
    dev.mock.images["#{name}.png"] = TrmnlSim::Images.png_image(->(x, y) { number.(x, y) ? 0 : 1 }, w, h)
    dev.mock.stamp(name)
    ["/images/#{name}.png", TrmnlSim::Images.expected_gray(->(x, y) { number.(x, y) ? 0 : 1 }, w, h)]
  end

  it "checks in again without deep sleeping or joining Wi-Fi again", :smoke do
    dev.mock.display_queue = Array.new(3) { { image: "default", refresh_rate: 2, keep_wifi_on: true } }
    dev.mock.display = { image: "default", refresh_rate: 600 }
    dev.boot do |s|
      st = s.wait_for_deep_sleep(timeout: 180)
      expect(dev.mock.count("/api/display")).to eq(4)
      expect(wifi_joins(s)).to eq(1)
      check_ins = dev.mock.requests.select { _1.path == "/api/display" }
      expect(check_ins.drop(1)).to all(have_header("Update-Source", "timer"))
      expect(st["wake_at_s"] - st["sim_time_s"]).to be_within(15).of(600)
    end
  end

  it "draws a new image between check-ins without dropping Wi-Fi" do
    serve_png("one", device_number("1"))
    path, expected = serve_png("two", device_number("2"))
    dev.mock.display_queue = [{ image: "one", refresh_rate: 2, keep_wifi_on: true },
                              { image: "two", refresh_rate: 2, keep_wifi_on: true },
                              { image: "two", refresh_rate: 2, keep_wifi_on: true }]
    dev.mock.display = { image: "two", refresh_rate: 600 }
    dev.boot do |s|
      s.wait_for_deep_sleep(timeout: 180)
      expect(dev.mock.paths.count(path)).to eq(1)
      expect(wifi_joins(s)).to eq(1)
      expect(s).to show_image(expected, tolerance: 64, max_ratio: 0)
    end
  end

  it "stays awake past the 20 minute awake limit while check-ins keep working" do
    dev.mock.display = { image: "default", refresh_rate: 60, keep_wifi_on: true }
    dev.boot do |s|
      started = dev.mock.wait_for_request("/api/display", timeout: 90) && s.status["sim_time_s"]
      15.times { s.status["sim_time_s"] - started > 25 * 60 ? break : s.wait(settle_ms: 20_000, timeout: 60) }
      expect([s.status["sim_time_s"] - started, dev.mock.requests.select { _1.path == "/api/display" }])
        .to match([be > 25 * 60, all(have_header("Reset-Reason", "ESP_RST_POWERON"))])
    end
  end

  it "draws live frames after the first with the fast balanced waveform" do
    serve_png("one", device_number("1"))
    _, expected = serve_png("two", device_number("2"))
    dev.mock.display_queue = [{ image: "one", refresh_rate: 2, keep_wifi_on: true },
                              { image: "two", refresh_rate: 2, keep_wifi_on: true }]
    dev.mock.display = { image: "two", refresh_rate: 600 }
    dev.boot do |s|
      s.wait_for_deep_sleep(timeout: 180)
      drawn_live = s.console(0).grep(/Live frame drawn in/).size
      expect([drawn_live, s]).to match([1, show_image(expected, tolerance: 64, max_ratio: 0)])
    end
  end

  it "draws a live frame in full after an error screen covered the last one" do
    serve_png("one", device_number("1"))
    serve_png("two", device_number("2"))
    _, expected = serve_png("three", device_number("3"))
    dev.mock.set_fault("/images/two.png", status: 404)
    dev.mock.display_queue = %w[one two three].map { |name| { image: name, refresh_rate: 2, keep_wifi_on: true } }
    dev.mock.display = { image: "three", refresh_rate: 600 }
    dev.boot do |s|
      s.wait_for_deep_sleep(timeout: 180)
      expect(s).to show_image(expected, tolerance: 64, max_ratio: 0)
    end
  end

  it "does not write live frames to flash" do
    serve_png("one", device_number("1"))
    serve_png("two", device_number("2"))
    dev.mock.display_queue = [{ image: "one", refresh_rate: 2, keep_wifi_on: true },
                              { image: "two", refresh_rate: 2, keep_wifi_on: true }]
    dev.mock.display = { image: "two", refresh_rate: 600 }
    dev.boot do |s|
      s.wait_for_deep_sleep(timeout: 180)
      expect(s.console(0).grep(/Writing .* to SPIFFS/).size).to eq(0)

  it "draws an image sent inside the answer without downloading it" do
    path, expected = serve_png("one", device_number("1"))
    image_data = [dev.mock.images["one.png"]].pack("m0")
    dev.mock.display_queue = [{ image: "one", refresh_rate: 2, keep_wifi_on: true, image_data: }]
    dev.mock.display = { image: "one", refresh_rate: 600 }
    dev.boot do |s|
      s.wait_for_deep_sleep(timeout: 180)
      expect([dev.mock.paths.count(path), s]).to match([0, show_image(expected, tolerance: 64, max_ratio: 0)])
    end
  end

  it "downloads the image when the inline copy is not valid base64" do
    path, expected = serve_png("one", device_number("1"))
    dev.mock.display_queue = [{ image: "one", refresh_rate: 2, keep_wifi_on: true, image_data: "@@@" }]
    dev.mock.display = { image: "one", refresh_rate: 600 }
    dev.boot do |s|
      s.wait_for_deep_sleep(timeout: 180)
      expect([dev.mock.paths.count(path), s]).to match([1, show_image(expected, tolerance: 64, max_ratio: 0)])
    end
  end

  it "reports how long each step of the last image took" do
    serve_png("one", device_number("1"))
    dev.mock.display_queue = [{ image: "one", refresh_rate: 2, keep_wifi_on: true }]
    dev.mock.display = { image: "one", refresh_rate: 600 }
    dev.boot do |s|
      s.wait_for_deep_sleep(timeout: 180)
      first, second = dev.mock.requests.select { _1.path == "/api/display" }
      expect([first.headers["Timing"], second]).to match(
        [nil, have_header("Timing", /\Aapi=\d+;download=\d+;logs=\d+;draw=\d+;save=\d+;heap=\d+\z/)]
      )
    end
  end

  it "fully refreshes as often as the server asks" do
    frames = %w[1 2 3 4].map { |n| serve_png("frame#{n}", device_number(n)) && "frame#{n}" }
    dev.mock.display_queue = frames.map { |f| { image: f, refresh_rate: 2, keep_wifi_on: true, full_refresh_every: 1 } }
    dev.mock.display = { image: "frame4", refresh_rate: 600 }
    dev.boot do |s|
      s.wait_for_deep_sleep(timeout: 180)
      expect(s.console(0).grep(/Forcing full refresh/).size).to be >= 4
    end
  end

  # HttpRetryRequest tries a failing check-in 5 times; the failed answer has no keep_wifi_on.
  it "deep sleeps when a check-in fails" do
    dev.mock.display = { image: "default", refresh_rate: 2, keep_wifi_on: true }
    dev.boot do |s|
      dev.mock.wait_for_request("/api/display", timeout: 90)
      dev.mock.set_fault("/api/display", status: 500)
      s.wait_for_deep_sleep(timeout: 180)
      expect(dev.mock.count("/api/display")).to be <= 1 + 5 + 1
    end
  end

  it "waits at least a second between check-ins when refresh_rate is 0" do
    dev.mock.display = { image: "default", refresh_rate: 0, keep_wifi_on: true }
    dev.boot do |s|
      first = dev.mock.wait_for_request("/api/display", timeout: 90)
      started = s.status["sim_time_s"]
      s.wait(settle_ms: 10_000, timeout: 120)
      elapsed = s.status["sim_time_s"] - started
      expect(dev.mock.requests.count { _1.path == "/api/display" && _1 != first }).to be <= elapsed.ceil + 1
    end
  end

  it "checks in refresh_rate_ms after the answer when the server sends it" do
    dev.mock.display = { image: "default", refresh_rate: 1, refresh_rate_ms: 300, keep_wifi_on: true }
    dev.boot do |s|
      first = dev.mock.wait_for_request("/api/display", timeout: 90)
      started = s.status["sim_time_s"]
      s.wait(settle_ms: 10_000, timeout: 120)
      elapsed = s.status["sim_time_s"] - started
      expect(dev.mock.requests.count { _1.path == "/api/display" && _1 != first }).to be > elapsed * 1.5
    end
  end

  it "waits at least 250 ms between check-ins when refresh_rate_ms is lower" do
    dev.mock.display = { image: "default", refresh_rate: 1, refresh_rate_ms: 1, keep_wifi_on: true }
    dev.boot do |s|
      first = dev.mock.wait_for_request("/api/display", timeout: 90)
      started = s.status["sim_time_s"]
      s.wait(settle_ms: 10_000, timeout: 120)
      elapsed = s.status["sim_time_s"] - started
      expect(dev.mock.requests.count { _1.path == "/api/display" && _1 != first }).to be <= (elapsed / 0.25).ceil + 1
    end
  end

  it "keeps checking in when an image download fails" do
    serve_png("one", device_number("1"))
    dev.mock.display = { image: "one", refresh_rate: 2, keep_wifi_on: true }
    dev.mock.set_fault("/images/*", status: 404)
    dev.boot do |s|
      dev.mock.wait_for_request("/api/display", timeout: 90)
      cursor = dev.mock.requests.size
      dev.mock.wait_for_request("/api/display", after: cursor, timeout: 90)
      dev.mock.display = { image: "default", refresh_rate: 600 }
      s.wait_for_deep_sleep(timeout: 180)
    end
  end

  it "deep sleeps when Wi-Fi goes away between check-ins" do
    dev.mock.display = { image: "default", refresh_rate: 2, keep_wifi_on: true }
    dev.boot do |s|
      dev.mock.wait_for_request("/api/display", timeout: 90)
      s.set_wifi(false)
      s.wait_for_deep_sleep(timeout: 300)
    end
  end

  it "answers a button press between check-ins", needs: :button do
    dev.mock.display = { image: "default", refresh_rate: 5, keep_wifi_on: true }
    dev.boot do |s|
      dev.mock.wait_for_request("/api/display", timeout: 90)
      s.wait(display_idle: true, settle_ms: 500, timeout: 60)
      req = dev.mock.next_request("/api/display", timeout: 30) { s.press(150) }
      expect(req).to have_header("Update-Source", device.button_source)
      expect(dev.mock.next_request("/api/display", timeout: 30)).to have_header("Update-Source", "timer")
    end
  end

  # The press interrupt stays attached through the loop's check-ins, which take most of a 1 s round.
  it "answers a button press made during a check-in", needs: :button do
    dev.mock.display = { image: "default", refresh_rate: 2, keep_wifi_on: true }
    dev.mock.set_fault("/api/display", delay: 1)
    dev.boot do |s|
      dev.mock.wait_for_request("/api/display", timeout: 90)
      s.wait(display_idle: true, settle_ms: 500, timeout: 60)
      cursor = dev.mock.requests.size
      dev.mock.wait_for_request("/api/display", after: cursor, timeout: 30)
      s.press(150)
      s.wait(settle_ms: 4000, timeout: 60)
      sources = dev.mock.requests.drop(cursor).select { _1.path == "/api/display" }.map { _1.headers["Update-Source"] }
      expect(sources.first(2)).to eq(["timer", device.button_source])
    end
  end

  it "checks in refresh_rate seconds after the answer, not after the draw",
     slow: "wall-clock timing needs REALTIME=1, which paces the simulator" do
    serve_png("one", device_number("1"))
    serve_png("two", device_number("2"))
    dev.mock.display_queue = [{ image: "one", refresh_rate: 4, keep_wifi_on: true },
                              { image: "two", refresh_rate: 4, keep_wifi_on: true },
                              { image: "one", refresh_rate: 4, keep_wifi_on: true }]
    dev.mock.display = { image: "one", refresh_rate: 600 }
    dev.boot do |s|
      s.wait_for_deep_sleep(timeout: 180)
      at = dev.mock.requests.select { _1.path == "/api/display" }.map(&:at)
      expect(at.each_cons(2).map { |earlier, later| (later - earlier).round(1) }).to all(be_within(1).of(4))
    end
  end
end
