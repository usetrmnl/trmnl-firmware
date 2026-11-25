# frozen_string_literal: true

# The TRMNL X panel power check (src/panel_power_check.cpp): 30 panel power-ups, a FAIL when the
# TPS65185 takes over 1 s to report power good on any of them. It runs on the
# factory first boot, before shipment mode, and on a reset near the TRMNL_QA network.

RSpec.describe "TRMNL X panel power check", env: "TRMNL_X" do
  fixture(:shipped) { TrmnlX::ShippedX.new }
  fixture(:dev) { TrmnlX::ProvisionedX.new(shipped) }

  let(:networks) { [{ ssid: TrmnlX::SSID_24 }, { ssid: TrmnlX::SSID_5, channel: 36 }, { ssid: "TRMNL_QA" }] }

  it "passes on the factory first boot, then ships" do
    console = shipped.factory_console.join("\n")
    expect(console.index("[PANEL] Power check passed")).to be < console.index("Entering shipment mode")
  end

  it "keeps a unit whose panel never powers up out of shipment mode" do
    TrmnlX.sim(erase: true, faults: { panel_busy_stuck: true }) do |s|
      s.wait_for_console(/\[PANEL\] Power check FAILED/, timeout: 300)
      s.wait(state: "deep_sleep", timeout: 60)
      expect(s.console(0).join("\n")).not_to match(/shipment mode/)
    end
  end

  it "runs on a reset near the TRMNL_QA network, sleeps with the result on screen, then checks in on its timer" do
    dev.reset
    dev.boot(networks:) do |s|
      s.wait_for_console(/Panel power check PASS  slowest \d+ ms, 0 of 30 tries timed out/, timeout: 300)
      asleep = s.wait_for_deep_sleep(timeout: 60)
      expect(dev.mock.paths).not_to include("/api/display")
      expect(asleep["wake_at_s"]).to be_within(15).of(asleep["sim_time_s"] + 300), "no timer wake armed"
      request = dev.mock.next_request("/api/display", timeout: 60) { s.wake }
      expect(request).to have_header("Update-Source", "timer")
    end
  end

  it "does not run on the restart after an update near the TRMNL_QA network" do
    dev.reset
    firmware = dev.mock.set_file("/firmware.bin", "application/octet-stream",
                                 File.binread(File.join(TrmnlX.build, "firmware.bin")))
    dev.mock.display_queue = [{ image: "default", refresh_rate: 300, update_firmware: true, firmware_url: firmware }]
    dev.boot_asleep(networks:) do |s|
      s.wait_for_deep_sleep(timeout: 60)
      s.wake
      s.wait_for_console(/Modem OTA successful/, timeout: 60)
      dev.mock.next_request("/api/display", timeout: 60)
      expect(s.console(0).join("\n")).not_to match(/Scanning for network: TRMNL_QA/)
    end
  end
end
