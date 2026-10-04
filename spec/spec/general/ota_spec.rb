# frozen_string_literal: true

# Firmware updates that go wrong on the device under test: no URL, the download failing or cut
# short, a file that isn't firmware (the TRMNL X's updates through its modem:
# core/trmnl_x/ota_spec.rb).

General.describe "OTA updates" do
  fixture(:dev) { ProvisionedDevice.new(build) }
  include_context "OTA updates"

  describe "OgUpdates" do
    it "ignores an update without a url" do
      expect_failed_update(nil)
    end

    it "survives a missing firmware file" do
      expect_failed_update("/missing.bin")
    end

    it "does not retry a failed update within a day" do
      # The failure's time is stored so a bad update can't boot-loop the device.
      m = dev.mock
      update_from("/missing.bin")
      dev.boot do |s|
        expect_to_keep_running_the_old_firmware(s, "/missing.bin")
        if m.display_queue.empty?
          m.display_queue = [{ image: "default", refresh_rate: 300, update_firmware: true,
                               firmware_url: "#{m.device_url}/missing.bin" }]
        end
        n = m.cursor
        c = s.status["console_total"]
        s.wake
        m.wait_for_request("/api/display", after: n, timeout: 20)
        s.wait(console: /Last OTA attempt was < 24h ago, skipping/, since: c, timeout: 20)
        s.wait_for_deep_sleep(timeout: 20)
        expect(m.paths.drop(n)).not_to include("/missing.bin")
      end
    end

    it "survives a download cut short",
       skip: "slow: after the connection drops, Update.writeStream waits well over 20 s for the rest of the firmware" do
      update_from("/firmware.bin", File.binread(File.join(dev.build, "firmware.bin")))
      dev.mock.set_fault("/firmware.bin", truncate: 20_000)
      dev.boot { |s| expect_to_keep_running_the_old_firmware(s, "/firmware.bin") }
    end

    it "rejects a file that is not firmware" do
      expect_failed_update("/not-firmware.bin", not_firmware)
    end

    it "rejects firmware too large for the slot" do
      flash = File.binread(File.join(dev.cache, "flash.bin"))
      slot = Flash.partition_table(flash).find { |p| p.type.zero? && p.subtype == 0x11 }
      expect_failed_update("/huge.bin", "\xE9".b + ("\0".b * slot.size)) # a byte too many
    end

    it "survives the firmware host being unreachable" do
      update_from("/firmware.bin", "x")
      dev.mock.set_fault("/firmware.bin", close: true)
      dev.boot { |s| expect_to_keep_running_the_old_firmware(s, "/firmware.bin") }
    end
  end
end
