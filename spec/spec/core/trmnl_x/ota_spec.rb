# frozen_string_literal: true

# TRMNL X firmware updates through the 5 GHz modem: a bad file, a missing one, a good one (see
# general/ota_spec.rb for the updates on the device under test).

RSpec.describe "TRMNL X OTA updates", env: "TRMNL_X" do
  fixture(:shipped) { TrmnlX::ShippedX.new }
  fixture(:dev) { TrmnlX::ProvisionedX.new(shipped) } # onboarded on 5 GHz: OTA goes through the modem
  include_context "OTA updates"

  describe "XModemUpdates" do
    it "rejects a modem update with a file that is not firmware" do
      expect_failed_update("/not-firmware.bin", not_firmware)
    end

    it "survives a modem update of a missing file" do
      expect_failed_update("/missing.bin")
    end

    it "reports a failed modem update through the modem" do
      # On 5 GHz the S3's own WiFi stays off, so stored logs can only reach /api/log through the modem.
      update_from("/missing.bin")
      dev.boot do |s|
        expect_to_keep_running_the_old_firmware(s, "/missing.bin")
        log = dev.mock.wait_for_request("/api/log", timeout: 60)
        expect(log.body).to include("Modem OTA download failed")
      end
    end

    it "reports the modem's WiFi link in logs sent through the modem" do
      update_from("/missing.bin")
      dev.boot do |s|
        expect_to_keep_running_the_old_firmware(s, "/missing.bin")
        entry = JSON.parse(dev.mock.wait_for_request("/api/log", timeout: 60).body)["logs"].last
        expect(entry).to include("wifi_status" => "connected", "wifi_signal" => be < 0)
      end
    end

    it "installs the new firmware with a modem update" do
      update_from("/firmware.bin", File.binread(File.join(TrmnlX.build, "firmware.bin")))
      dev.boot do |s|
        dev.mock.wait_for_request("/firmware.bin", timeout: 20)
        s.wait_for_console(/Modem OTA successful/, timeout: 20)
        dev.mock.next_request("/api/display", timeout: 20)
        s.wait_for_deep_sleep(timeout: 20)
      end
    end

    # 5 GHz networks often give the modem under 17 KB/s, so the X's 1.5 MB firmware can take
    # well over two minutes to arrive while it keeps arriving.
    it "installs the new firmware with a slow modem update" do
      update_from("/firmware.bin", File.binread(File.join(TrmnlX.build, "firmware.bin")))
      dev.mock.set_fault("/firmware.bin", rate: 10_000)
      dev.boot do |s|
        dev.mock.wait_for_request("/firmware.bin", timeout: 20)
        s.wait_for_console(/Modem OTA successful/, timeout: 240)
        dev.mock.next_request("/api/display", timeout: 20)
        s.wait_for_deep_sleep(timeout: 20)
      end
    end
  end
end
