# frozen_string_literal: true

# A TRMNL X the way a customer gets it: shipped in shipment mode on the release before stable
# (Builds.factory_release, built by `rake "firmware:build_release[<tag>]"`), then brought up to
# this firmware by an update or a USB flash.

RSpec.describe "TRMNL X shipped on the factory release",
               env: "TRMNL_X", skip: Builds.release_missing(Builds.factory_release) do
  include_context "X updates to this firmware"

  fixture(:shipped) { TrmnlX::ShippedX.new(Builds.release(Builds.factory_release)) }
  # onboarded on 5 GHz: the update goes through the modem
  fixture(:dev) { TrmnlX::ProvisionedX.new(shipped, image: "white") }
  fixture(:dev_2ghz) { TrmnlX::ProvisionedX.new(shipped, ssid: TrmnlX::SSID_24, image: "white") }

  let(:factory_version) { Builds.factory_release.delete_prefix("v") }

  describe "UpdateAfterUnboxing" do
    before do
      dev.reset
      dev_2ghz.reset
    end

    it "updates to this firmware through the modem on 5 GHz", :smoke do
      expect_update(dev, from: factory_version)
    end

    it "updates to this firmware on 2.4 GHz" do
      expect_update(dev_2ghz, from: factory_version)
    end

    # A failed update is stored so it can't boot-loop the X (services/firmware_update.cpp,
    # performUpdate): it keeps checking in on the factory release and tries again a day later.
    it "updates a day after a failed update" do
      m = dev.mock
      offer_update(dev, "/missing.bin")
      dev.boot(extra_args: elf_dir) do |s|
        m.wait_for_request("/missing.bin", timeout: 60)
        s.wait_for_deep_sleep(timeout: 60)
        offer_update(dev, "/missing.bin")
        n = m.cursor
        s.wake
        expect(m.wait_for_request("/api/display", after: n, timeout: 60)).to have_header("FW-Version", factory_version)
        s.wait_for_deep_sleep(timeout: 60)
        expect(m.paths.drop(n)).not_to include("/missing.bin")

        s.set_preference("last_ota", "u32", s.preferences["last_ota"].to_i - 90_000) # a day and an hour ago
        m.set_file("/firmware.bin", "application/octet-stream", firmware)
        offer_update(dev, "/firmware.bin")
        n = m.cursor
        s.wake
        m.wait_for_request("/firmware.bin", after: n, timeout: 60)
        expect(m.wait_for_request("/api/display", after: m.cursor, timeout: 120))
          .to have_header("FW-Version", Builds.fw_version)
      end
    end
  end

  describe "ShipmentMode" do
    # The factory release reads the charger only when /INT fires (#695 fixes it): with USB already
    # in, it sleeps again, and only unplugging and plugging in again wakes it.
    it "reaches the portal when the charger is unplugged and plugged in again" do
      shipped.boot do |s|
        s.wait_for_console(/Entering shipment mode light sleep loop/, timeout: 180)
        s.pause do
          s.dock(true)
          s.power_cycle # the battery died in shipment mode; USB powers it back on
        end
        eventually("it did not go back to sleep on USB", within: 90) do
          st = s.status
          st["boot_count"] == 2 && st["state"] == "light_sleep"
        end
        expect(s.status["portal_url"]).to be_nil
        s.dock(false)
        s.dock(true)
        s.wait(portal: true, timeout: 90)
      end
    end

    # What `pio run -t upload` writes: the bootloader, the partition table, an empty otadata
    # (boot_app0.bin, so ota_0 boots) and the app in ota_0. NVS keeps the shipment state.
    def flash_over_usb(flash)
      image = File.binread(flash)
      table = Flash.partition_table(image)
      otadata = table.find { |p| p.type == 1 && p.subtype.zero? }
      ota0 = table.find { |p| p.ota_slot&.zero? }
      writes = { 0 => "bootloader.bin", 0x8000 => "partitions.bin", ota0.offset => "firmware.bin" }
      image[otadata.offset, otadata.size] = "\xFF".b * otadata.size
      writes.each do |offset, file|
        data = File.binread(File.join(TrmnlX.build, file))
        image[offset, data.bytesize] = data
      end
      File.binwrite(flash, image)
    end

    it "stays in shipment mode when this firmware is flashed over USB, and onboards on it" do
      flash = shipped.copy("usb-flashed")
      flash_over_usb(flash)
      TrmnlSim::MockTrmnl.open do |m|
        m.display = { image: "default", refresh_rate: 300 }
        TrmnlX.sim(flash:) do |s|
          TrmnlX.onboard(s, m, TrmnlX::SSID_5) # from shipment mode, docked into the portal
          expect(m.wait_for_request("/api/display")).to have_header("FW-Version", Builds.fw_version)
          expect(s.console(0).join("\n")).not_to include("Starting modem flash") # no second factory flow
        end
      end
    end
  end
end
