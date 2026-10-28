# frozen_string_literal: true

# Firmware updates that must fail harmlessly, for groups with a provisioned `dev` (a
# ProvisionedDevice or TrmnlX::ProvisionedX):
#
#   include_context "OTA updates"
#   it("...") { expect_failed_update("/missing.bin") }
RSpec.shared_context "OTA updates" do
  before { dev.reset }

  # Bytes that aren't a firmware image.
  let(:not_firmware) { (0..255).to_a.pack("C*") * 400 }

  # The first /api/display answer asks for an update from `path` on the mock (serving `data`
  # there if given; no path: an empty URL).
  def update_from(path, data = nil, content_type = "application/octet-stream")
    m = dev.mock
    url = if data.nil?
            path ? m.device_url + path : ""
          else
            m.set_file(path, content_type, data)
          end
    m.display_queue = [{ image: "default", refresh_rate: 300, update_firmware: true, firmware_url: url }]
  end

  def expect_to_keep_running_the_old_firmware(s, path)
    dev.mock.wait_for_request(path, timeout: 20) if path
    st = s.wait_for_deep_sleep(timeout: 20, settle_ms: 500)
    expect(st["boot_count"]).to eq(1), "it restarted"
    # (bootloaders that log would say so if they booted the other slot)
    other = Flash.partition_table(File.binread(s.flash)).find { |p| p.type.zero? && p.subtype == 0x11 }
    expect(s.console(0).join("\n")).not_to include(format("Loaded app from partition at offset %#x", other.offset))
    st
  end

  # Boot with an update from `path` pending, which must fail.
  def expect_failed_update(path, data = nil)
    update_from(path, data)
    dev.boot { |s| expect_to_keep_running_the_old_firmware(s, path) }
  end
end
