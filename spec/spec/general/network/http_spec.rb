# frozen_string_literal: true

# HTTP details on the device under test (see devices.rb): redirects, bodies without a
# Content-Length, and the error log (/api/log) when submitting fails. Images are served the way
# the TRMNL server serves the device (device_image).

General.describe "HTTP" do
  fixture(:dev) { ProvisionedDevice.new(build) }

  # The provisioned device, asleep after onboarding (showing the default image).
  def asleep
    dev.boot_asleep do |s|
      s.wait(state: "deep_sleep", display_idle: true, timeout: 15)
      yield s
    end
  end

  # Wake the (sleeping) device for a refresh; returns the new requests' paths.
  def refresh(s)
    n = dev.mock.cursor
    dev.mock.next_request("/api/display", timeout: 15) { s.wake }
    s.wait(state: "deep_sleep", display_idle: true, timeout: 15, settle_ms: 300)
    dev.mock.paths.drop(n)
  end

  describe "Redirects" do
    before { dev.reset }

    it "api display redirect to a relative location" do
      dev.mock.set_fault("/api/display", redirect: "/api/display?moved=1", times: 1)
      asleep { |s| expect(refresh(s).count("/api/display")).to eq(2) }
    end

    it "api display permanent redirect to an absolute url" do
      dev.mock.set_fault("/api/display", redirect: "#{dev.mock.device_url}/api/display?v=2", status: 308, times: 1)
      asleep { |s| expect(refresh(s).count("/api/display")).to eq(2) }
    end

    it "image redirect" do
      m = dev.mock
      path, seven = device_image(m, "seven", panel_number("7"))
      moved = path.gsub("seven", "moved")
      m.images[m.image_key(moved)] = m.images[m.image_key(path)]
      m.display = { image: "seven", refresh_rate: 300 }
      m.set_fault(path, redirect: moved, times: 1)
      asleep do |s|
        expect(refresh(s)).to include(moved)
        expect(s).to show_image(seven, tolerance: 64, max_ratio: 0)
      end
    end
  end

  describe "NoContentLength" do
    before { dev.reset }

    it "chunked image" do
      m = dev.mock
      path, eight = device_image(m, "eight", panel_number("8"))
      m.display = { image: "eight", refresh_rate: 300 }
      m.set_fault(path, chunked: true)
      asleep do |s|
        refresh(s)
        expect(s).to show_image(eight, tolerance: 64, max_ratio: 0)
      end
    end

    it "chunked image cut short is not shown" do
      m = dev.mock
      path, = device_image(m, "nine", panel_number("9"))
      m.display = { image: "nine", refresh_rate: 300 }
      m.set_fault(path, chunked: true, truncate: m.images[m.image_key(path)].bytesize / 2)
      asleep do |s|
        screen = s.screenshot
        refresh(s)
        expect(s).to show_image(screen, tolerance: 0, max_ratio: 0)
      end
    end

    it "chunked api display answer" do
      dev.mock.set_fault("/api/display", chunked: true)
      asleep { |s| expect(refresh(s)).to include(dev.mock.image_path("default")) }
    end
  end

  describe "ErrorLog" do
    before { dev.reset }

    it "log submission follows a redirect" do
      m = dev.mock
      m.set_fault("/images/*", status: 404)
      m.set_fault("/api/log", redirect: "/api/log?moved=1", times: 1)
      asleep { |s| expect(refresh(s).count("/api/log")).to eq(2) }
    end

    it "logs are kept while the log endpoint fails" do
      m = dev.mock
      m.set_fault("/images/*", status: 404)
      m.set_fault("/api/log", status: 500)
      asleep do |s|
        8.times { refresh(s) } # more errors than there are log slots
        m.clear_faults
        expect(refresh(s)).to include("/api/log")
        logs = m.requests.select { |r| r.path == "/api/log" && !r.body.to_s.empty? }
        expect(logs.last.json["logs"].size).to be > 1
      end
    end
  end

  describe "SetupRedirect" do
    # (/api/setup's logo is the mock's 800x480 BMP on every device)
    def onboard_with_logo_redirect(absolute:)
      device_mock do |mock|
        mock.images["moved"] = mock.images["default"]
        location = "#{mock.device_url if absolute}/images/moved.bmp"
        mock.set_fault("/images/default.bmp", redirect: location, times: 1)
        sim(erase: true, extra_args: ["--offline"]) do |s|
          s.wait(portal: true, timeout: 15)
          s.portal_connect("TRMNL-Sim", "password", server: mock.device_url)
          mock.wait_for_request("/images/moved.bmp", timeout: 15)
          mock.wait_for_request("/api/display", timeout: 15)
          s.wait(state: "deep_sleep", display_idle: true, timeout: 15)
        end
      end
    end

    it "setup logo redirect" do
      onboard_with_logo_redirect(absolute: true)
    end

    it "setup logo redirect to a relative location" do
      # DeviceSetup::downloadSetupImage follows a 307/308 with begin(getLocation()), without
      # resolving a relative Location against the image URL (HttpRetryRequest does, with
      # resolveRedirectLocation): the second GET fails and onboarding reports an error.
      onboard_with_logo_redirect(absolute: false)
    end
  end
end
