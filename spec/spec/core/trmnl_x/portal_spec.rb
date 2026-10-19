# frozen_string_literal: true

require "json"

# TRMNL X setup portal: timing out back to shipment mode, onboarding on either band, and failed
# joins. Kept apart from trmnl_x_spec.rb so that parallel_rspec, which hands out whole files, runs
# them on another worker.

RSpec.describe "TRMNL X", env: "TRMNL_X" do
  fixture(:shipped) { TrmnlX::ShippedX.new }

  def monotonic = Process.clock_gettime(Process::CLOCK_MONOTONIC)

  describe "PortalTimeout" do
    def time_out(s, docked: false)
      s.wait_for_console(/Entering shipment mode light sleep loop/, timeout: 30)
      s.dock(true)
      s.wait(portal: true, timeout: 90)
      s.dock(docked)
      s.set_portal_client(false) # so turbo can run to the timeout
      # Serial isn't running when enter_shipment_sleep() announces itself on a production
      # build, so go by the clock: the portal is gone once 15 minutes have passed.
      t0 = s.status["sim_time_s"]
      st = nil
      # about 40 s alone, over 150 s on a 4-vCPU CI runner next to other tests
      eventually("the portal did not time out", within: 300) { (st = s.status)["sim_time_s"] >= t0 + (15 * 60) + 5 }
      expect(st["portal_url"]).to be_nil
      st["console_total"]
    end

    it "unattended portal goes back to shipment mode" do
      shipped.boot do |s|
        # Still docked: "ready to ship" until it comes off USB power, then shipment mode.
        time_out(s, docked: true)
        g = s.status["display_generation"]
        s.dock(false) # checked every 2 s: then the shipping screen
        # (wall-clock limits allow for a machine busy with other simulators)
        eventually("the shipping screen was not drawn", within: 90, every: 0.2) do
          s.status["display_generation"] != g
        end
        s.wait(display_idle: true, timeout: 90)
        c = s.status["console_total"]
        s.dock(true) # the charger ends shipment mode
        s.wait(console: /CHARGER DETECTED - Exiting shipment mode/, since: c, timeout: 90)
      end
    end

    # The setup screen's display_sleep(1000) leaves a 1 s light-sleep timer wakeup enabled:
    # shipment mode must disable it, or the device wakes every 1.2 s in the box.
    it "shipment mode after the timeout stays asleep" do
      shipped.boot do |s|
        c = time_out(s)
        t0 = s.status["sim_time_s"]
        sleep 0.2 while s.status["sim_time_s"] < t0 + 10
        wakes = s.console(c).grep(/WAKEUP from light sleep/)
        expect(wakes).to eq([])
      end
    end
  end

  describe "Onboarding" do
    it "portal rescans both radios and joins the chosen band" do
      TrmnlSim::MockTrmnl.open do |mock|
        shipped.boot do |s|
          s.wait_for_console(/Entering shipment mode light sleep loop/, timeout: 30)
          s.dock(true)
          s.wait(portal: true, timeout: 90)
          s.dock(false)
          c = s.status["console_total"]
          s.portal_request("/scan?force=1")
          s.wait(console: /Modem re-scan found/, since: c, timeout: 30)
          deadline = monotonic + 30
          sleep 0.5 while s.portal_request("/scan")[0] != 200 && monotonic < deadline
          code, body = s.portal_request("/scan")
          expect(JSON.parse(body)).to include("mac_5ghz") if code == 200
          body = { ssid: TrmnlX::SSID_24, pswd: "password", server: mock.device_url, band: "2.4GHz" }
          expect(s.portal_request("/connect", body)[0]).to eq(200)
          req = mock.wait_for_request("/api/display", timeout: 30)
          expect(req).to have_header("RSSI", "-54") # the S3's own radio
        end
      end
    end

    it "onboarding on 2.4 GHz uses the S3 radio" do
      TrmnlSim::MockTrmnl.open do |mock|
        shipped.boot do |s|
          TrmnlX.onboard(s, mock, TrmnlX::SSID_24)
          req = mock.wait_for_request("/api/display")
          expect(req).to have_header("RSSI", "-54") # the 2.4 GHz AP, seen by the S3
        end
      end
    end

    # The S3 radio never starts on the modem path, and the server matches devices by this MAC.
    it "onboarding on 5 GHz sends the device's own MAC to /api/setup" do
      TrmnlSim::MockTrmnl.open do |mock|
        shipped.boot do |s|
          TrmnlX.onboard(s, mock, TrmnlX::SSID_5)
          expect(mock.wait_for_request("/api/setup")).to have_header("ID", TrmnlX::MAC)
        end
      end
    end
  end

  # A join that fails leaves the portal up, so the password can be corrected.
  describe "FailedJoin" do
    def portal(s)
      s.wait_for_console(/Entering shipment mode light sleep loop/, timeout: 180)
      s.dock(true)
      s.wait(portal: true, timeout: 180)
      s.dock(false)
      s.set_networks([{ ssid: TrmnlX::SSID_24, password: "password", rssi: -54, channel: 6 },
                      { ssid: TrmnlX::SSID_5, password: "password", rssi: -48, channel: 36 }])
    end

    def join(s, ssid, band)
      c = s.status["console_total"]
      code, data = s.portal_request("/connect", { ssid:, pswd: "wrong", server: "http://x", band: })
      expect(code).to eq(200), data
      c
    end

    # For a minute of simulated time the device stays in setup mode (whether its AP stayed up
    # or it brought the portal back), and the portal answers.
    def expect_portal_to_stay(s)
      t0 = s.status["sim_time_s"]
      s.set_portal_client(false) # nobody uses the portal while we watch, so turbo can run
      eventually("the simulation stalled", within: 120) do
        st = s.status
        expect(st["state"]).not_to eq("deep_sleep"), "went to sleep instead of keeping the portal"
        st["sim_time_s"] >= t0 + 60
      end
      s.set_portal_client(true)
      s.wait(portal: true, timeout: 90)
      expect(s.portal_request("/")[0]).to eq(200)
    end

    it "wrong password on 2.4 GHz keeps the portal" do
      shipped.boot do |s|
        portal(s)
        c = join(s, TrmnlX::SSID_24, "2.4GHz")
        s.wait(console: /connect attempt failed/, since: c, timeout: 60)
        expect_portal_to_stay(s)
      end
    end

    it "wrong password on 5 GHz keeps the portal" do
      shipped.boot do |s|
        portal(s)
        join(s, TrmnlX::SSID_5, "5GHz") # the modem's join fails silently on a production build
        expect_portal_to_stay(s)
      end
    end
  end
end
