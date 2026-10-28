# frozen_string_literal: true

require "tmpdir"

# The gen-2 TRMNL OG on the ESP32-C5: `og_gen2` (env trmnl_gen2, the 7.5" black and white
# UC8179 panel) and `og_gen2_4clr` (trmnl_gen2_4clr, the 4-color BWRY panel). Both have a
# BQ27427 fuel gauge on I2C and a BQ25616 charger whose open-drain PG/STAT outputs are on
# GPIO 25/24; the simulator's dock switch plugs the USB cable in. The C5's own radio does
# 2.4 and 5 GHz. Groups whose build is missing skip.

RSpec.describe "OG gen 2" do
  # What both gen-2 boards get: the BYOD board tests (identity, the served image; see
  # spec/support/byod.rb) and more.
  shared_examples "a gen-2 board" do
    gen2 = Devices.fetch(metadata[:env])
    byod_board name: gen2.name, model: gen2.model, size: gen2.size, battery_v: gen2.battery_v, inks: gen2.inks
    let(:gen2_build) { Builds.for_env(gen2.env) }

    # Wake the sleeping device (timer, or the block); return its /api/display request once it
    # sleeps again.
    def wake_for_display(s, &wake)
      wait_until_asleep(s)
      req = dev.mock.next_request("/api/display", timeout: 60) { wake ? wake.() : s.wake }
      wait_until_asleep(s, timeout: 120)
      req
    end

    # ---- the gen-2 tests ---------------------------------------------------------------------------

    it "reports the battery from the fuel gauge" do
      dev.boot_asleep do |s|
        s.set_battery(3700)
        req = wake_for_display(s)
        expect(req).to have_header("Battery-Voltage", a_value_within(0.02).of(3.70))
      end
    end

    it "takes USB power and charging from the charger lines" do
      usb_and_charging = ->(req) { req.headers.values_at("USB-Connected", "Battery-Charging") }
      dev.boot_asleep do |s|
        expect(usb_and_charging.(wake_for_display(s))).to eq(%w[false 0])
        s.dock(true)
        expect(s.status["charging"]).to be(true)
        expect(usb_and_charging.(wake_for_display(s))).to eq(%w[true 1])
        # charged: USB still in, STAT high
        s.set_battery(4200)
        expect(usb_and_charging.(wake_for_display(s))).to eq(%w[true 0])
      end
    end

    it "wakes on its timer" do
      dev.boot_asleep do |s|
        st = s.wait_for_deep_sleep(display_idle: true, timeout: 60)
        expect(st["wake_at_s"] - st["sim_time_s"]).to be_within(15).of(300)
        req = wake_for_display(s)
        expect(req).to have_header("Update-Source", "timer")
        st = s.status
        expect(st["wake_at_s"] - st["sim_time_s"]).to be_within(15).of(300)
      end
    end

    it "wakes on a button press" do
      dev.boot_asleep do |s|
        req = wake_for_display(s) { s.press(150) }
        expect(req).to have_header("Update-Source", "button")
      end
    end

    it "round-trips a save point" do
      m = dev.mock
      serve_test_image("first")
      Dir.mktmpdir do |tmp|
        path = File.join(tmp, "gen2.trmnlsave")
        saved = screen = boots = nil
        dev.boot_asleep do |s|
          refresh(s, "/images/first.png")
          s.dock(true)
          saved = s.save_point(path)
          screen = s.screenshot
          boots = s.status["boot_count"]
        end
        expect(saved["deep_sleep"]).to be(true)
        second = if board.inks == "bwry"
                   m.set_color_png("second", ->(x, _y) { TrmnlSim::Images::BWRY_RGB[x < 400 ? :red : :yellow] })
                 else
                   m.set_image("second", TrmnlSim::Images.big_number("7"))
                 end
        m.display = { image: "second", refresh_rate: 300 }
        m.requests.clear
        dev.restore(path) do |s|
          st = s.wait_for_deep_sleep(timeout: 30)
          expect(st.values_at("boot_count", "docked", "charging")).to eq([boots, true, true])
          expect(s).to show_image(screen, tolerance: 0, max_ratio: 0)
          expect(s).not_to show_image(second, tolerance: 16, max_ratio: 0.001)
          req = wake_for_display(s)
          expect(req).to have_header("USB-Connected", "true")
          expect(m.paths).not_to include("/api/setup")
          expect(m.paths).to include(board.inks == "bwry" ? "/images/second.png" : "/images/second.bmp")
          expect(s).to show_image(second, tolerance: 16, max_ratio: 0.001)
        end
      end
    end

    it "boots again from a power-off save point" do
      Dir.mktmpdir do |tmp|
        path = File.join(tmp, "off.trmnlsave")
        saved = dev.boot_asleep do |s|
          s.wait_for_deep_sleep(display_idle: true, timeout: 60)
          s.wake
          s.wait(state: "running", timeout: 30)
          s.save_point(path)
        end
        expect(saved["deep_sleep"]).to be(false)
        after = dev.mock.cursor
        dev.restore(path) do |s|
          req = dev.mock.wait_for_request("/api/display", after:, timeout: 90)
          expect(req).to have_header("Model", board.model)
          s.wait_for_deep_sleep(display_idle: true, timeout: 120)
        end
      end
    end

    it "onboards on 5 GHz with its own radio" do
      TrmnlSim::MockTrmnl.open do |mock|
        sim(gen2_build, erase: true, extra_args: ["--offline"]) do |s|
          s.wait(portal: true, timeout: 90)
          scan = s.portal_scan["networks"].map { _1["name"] }
          expect(scan).to include("TRMNL-Sim-5G", "TRMNL-Sim")
          s.portal_connect("TRMNL-Sim-5G", "password", server: mock.device_url)
          req = mock.wait_for_request("/api/display", timeout: 120)
          expect(req).to have_header("RSSI", "-48")
          expect(req).to have_header("WiFi-Band", "5")
          s.wait_for_deep_sleep(display_idle: true, timeout: 120)
        end
      end
    end

    it "reports the band after onboarding on 2.4 GHz" do
      dev.boot_asleep do |s|
        req = wake_for_display(s)
        expect(req.headers.values_at("RSSI", "WiFi-Band")).to eq(%w[-54 2.4])
      end
    end

    it "uses the crypto accelerators for https" do
      # ECDHE-ECDSA with a P-384 certificate: the C5's ECC (point multiplication) and ECDSA
      # (signature verification) accelerators, SHA and AES-GCM over DMA
      TrmnlSim::MockTrmnl.open(tls: true) do |mock|
        sim(gen2_build, erase: true, extra_args: ["--offline"]) do |s|
          s.wait(portal: true, timeout: 90)
          s.portal_connect("TRMNL-Sim", "password", server: mock.device_url)
          req = mock.wait_for_request("/api/display", timeout: 120)
          expect(req).to have_header("Model", board.model)
          s.wait_for_deep_sleep(display_idle: true, timeout: 120)
          expect(mock.paths).to include("/api/setup")
        end
      end
    end
  end

  describe "OgGen2", env: "trmnl_gen2" do
    include_examples "a gen-2 board"

    it "memchecks onboarding and refresh cycles" do
      # The heap checker follows the C5's IDF 5.5 heap and its single-core FreeRTOS tasks.
      TrmnlSim::MockTrmnl.open do |mock|
        sim(gen2_build, erase: true, memcheck: "halt", extra_args: ["--offline"]) do |s|
          mock.set_image("one", TrmnlSim::Images.big_number("1"))
          mock.display = { image: "one", refresh_rate: 300 }
          s.wait(portal: true, timeout: 90)
          s.portal_connect("TRMNL-Sim", "password", server: mock.device_url)
          mock.wait_for_request("/api/display", timeout: 120)
          s.wait_for_deep_sleep(display_idle: true, timeout: 120)
          mock.next_request("/api/display", timeout: 90) { s.press(150) }
          s.wait_for_deep_sleep(display_idle: true, timeout: 90)
          report = s.memcheck
          expect(report["violations"]).to eq([])
          expect(report["heap"]["allocs"]).to be > 500
          stacks = report["stacks"].to_h { [_1["task"], _1] }
          expect(stacks["loopTask"]["size"]).to eq(8192)
          expect(stacks["loopTask"]["instances"]).to be >= 2 # one per boot
        end
      end
    end

    it "covers the setup boot" do
      Dir.mktmpdir do |tmp|
        sim(gen2_build, erase: true, coverage: File.join(tmp, "c5.info"), extra_args: ["--offline"]) do |s|
          s.wait(portal: true, timeout: 90)
          cov = s.write_coverage(File.join(tmp, "mid.info"))
          expect(cov["lines_hit"]).to be > 1000
          expect(cov["lines_hit"]).to be < cov["lines_found"]
          expect(File.read(File.join(tmp, "mid.info"))).to include("SF:src/bl.cpp")
        end
      end
    end
  end

  describe "OgGen2Bwry", env: "trmnl_gen2_4clr" do
    include_examples "a gen-2 board"

    it "renders a color png in four colors" do
      m = dev.mock
      expected = m.set_color_png("bars", TrmnlSim::Images.color_bars)
      m.display = { image: "bars", refresh_rate: 300 }
      dev.boot_asleep do |s|
        s.wait_for_deep_sleep(display_idle: true, timeout: 60)
        refresh(s, "/images/bars.png")
        expect(s).to show_image(expected, tolerance: 16, max_ratio: 0)
        expect(s.screenshot(region: [0, 0, 8, 8]).getbyte(25)).to eq(2) # truecolor PNG
      end
    end

    it "takes the panel's long update to refresh" do
      m = dev.mock
      m.set_color_png("red", ->(_x, _y) { TrmnlSim::Images::BWRY_RGB[:red] })
      m.display = { image: "red", refresh_rate: 300 }
      dev.boot_asleep do |s|
        s.wait_for_deep_sleep(display_idle: true, timeout: 60)
        m.next_request("/images/red.png", timeout: 60) { s.wake }
        t0 = s.status["sim_time_s"]
        st = s.wait_for_deep_sleep(display_idle: true, timeout: 120)
        expect(st["sim_time_s"] - t0).to be > 15 # the 4-color update alone is ~16 s
      end
    end
  end
end
