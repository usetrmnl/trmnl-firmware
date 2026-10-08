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

    it "installs the new firmware with a modem update" do
      update_from("/firmware.bin", File.binread(File.join(TrmnlX.build, "firmware.bin")))
      dev.boot do |s|
        dev.mock.wait_for_request("/firmware.bin", timeout: 20)
        s.wait_for_console(/Modem OTA successful/, timeout: 20)
        dev.mock.next_request("/api/display", timeout: 20)
        s.wait_for_deep_sleep(timeout: 20)
      end
    end
  end
end
