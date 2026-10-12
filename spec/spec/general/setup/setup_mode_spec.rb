# frozen_string_literal: true

# A factory-fresh device: WiFi setup screen, captive portal, onboarding.

General.describe "Setup mode" do
  # Where the screens' texts are (display_show_msg in the firmware's display.cpp): the version
  # line at (40, 48) left of the top-right QR code, the portal's texts at fixed heights; centred.
  # On the OG's 800x480 panel: body [0, 56, 800, 424], top_right [320, 0, 480, 56], ssid_line
  # [140, 368, 520, 44], timed_out [260, 320, 280, 48]. Other panels compare with their own
  # goldens (golden/<env>/, see Golden.for_device).
  def region(name)
    w, h = device.size
    { body: [0, 56, w, h - 56],
      top_right: [320, 0, w - 320, 56],
      ssid_line: [(w - 520) / 2, 368, 520, 44],
      timed_out: [(w - 280) / 2, 320, 280, 48] }.fetch(name)
  end

  describe "FreshDevice" do
    fixture(:setup_sim) { fresh_device(settled: { display_idle: true, min_refreshes: 1, settle_ms: 500 }) }

    it "shows the setup screen" do
      # All but the "TRMNL firmware <version> (<git hash>)" line, which changes every commit.
      expect(setup_sim).to match_golden("setup_screen_body.png", region: region(:body))
      expect(setup_sim).to match_golden("setup_screen_top_right.png", region: region(:top_right))
    end

    it "setup screen names the access point", :smoke do
      # Just the "Connect ... to TRMNL-XXXXXX" line, so version bumps don't break it.
      expect(setup_sim).to match_golden("setup_ssid_line.png", region: region(:ssid_line))
    end

    it "portal lists simulated networks" do
      scan = setup_sim.portal_scan
      expect(scan["networks"].map { _1["name"] }).to include("TRMNL-Sim")
      expect(scan["mac"]).to eq(Builds::TEST_MAC)
    end

    it "status reports setup mode" do
      st = setup_sim.status
      expect(st["state"]).to eq("running")
      expect(st["wifi_connected"]).to be(false)
      expect(st["portal_url"]).not_to be_nil
    end
  end

  describe "PortalTimeout",
           skip_if: :shipment,
           why: "the portal times out back into shipment mode (core/trmnl_x/trmnl_x_spec PortalTimeout)" do
    it "unattended portal times out and sleeps" do
      fresh_device do |s|
        s.set_portal_client(false) # nobody joins, so turbo can run to the timeout
        # (a slow panel is still showing the message when the chip sleeps)
        st = s.wait_for_deep_sleep(display_idle: true, timeout: 120)
        expect(st["sim_time_s"]).to be >= 15 * 60
        # "Wifi Captive Portal timed out" / "Press button to try again"
        expect(s).to match_golden("portal_timed_out.png", region: region(:timed_out))
        if device.button?
          s.set_portal_client(true)
          s.press(200)
          s.wait(portal: true, timeout: 60)
        end
      end
    end
  end

  describe "Onboarding" do
    it "onboarding registers with server", :smoke do
      device_mock do |mock|
        fresh_device do |s|
          s.portal_connect("TRMNL-Sim", "any-password", server: mock.device_url)
          s.wait(wifi_connected: true, timeout: 60)
          setup = mock.wait_for_request("/api/setup", timeout: 60)
          expect(setup).to have_header("ID", Builds::TEST_MAC)
          expect(setup).to have_header("Model", device.model)
          if device.panel_rev?
            expect(setup).to have_header("Panel-Rev", "0a0c1b2c")
          else
            expect(setup).not_to have_header("Panel-Rev")
          end
          display = mock.wait_for_request("/api/display", timeout: 60)
          expect(display).to have_header("Access-Token", mock.api_key)
          expect(display).to have_header("Update-Source", "powercycle")
          s.wait_for_deep_sleep
        end
      end
    end
  end
end
