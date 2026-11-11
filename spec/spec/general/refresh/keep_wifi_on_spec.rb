# frozen_string_literal: true

# An answer with keep_wifi_on keeps the device awake with Wi-Fi connected until its next check-in,
# refresh_rate seconds after the answer arrived, instead of deep sleeping (src/bl.cpp, bl_init).

General.describe "Keep Wi-Fi on" do
  fixture(:dev) { ProvisionedDevice.new(build) }

  before { dev.reset }

  def wifi_joins(sim) = sim.console(0).count { _1.include?("[sim] wifi: connecting to") }
  def wifi_stops(sim) = sim.console(0).count { _1.include?("[sim] wifi: esp_wifi_stop()") }

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
      expect(wifi_stops(s)).to eq(1)
      expect(s).to show_image(expected, tolerance: 64, max_ratio: 0)
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
      req = dev.mock.next_request("/api/display", timeout: 30) { s.press(150) }
      expect(req).to have_header("Update-Source", device.button_source)
    end
  end

  # Measured on the paced clock (the simulator runs ahead of it otherwise).
  it "checks in refresh_rate seconds after the answer, not after the draw" do
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
