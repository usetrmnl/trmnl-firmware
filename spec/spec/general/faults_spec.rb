# frozen_string_literal: true

# Fault injection on the device under test: server errors, broken downloads, bad networks
# (DNS failure, an access point without internet, slow and lossy links), power loss in the
# middle of flash writes, and a stuck panel. The device must cope: sleep, retry later, and keep
# booting. The image it is served is the one its server would send (see Integration#device_image).

module FaultsSpec
  # Like ProvisionedDevice, but onboarded with the server's host name instead of its IP, so
  # every request needs a DNS lookup (answered by the simulator via --dns).
  class NamedServerDevice < ProvisionedDevice
    HOST = "trmnl-mock.test"

    def cache_name = "named-server"
    def device_host = HOST
    def sim_args = ["--offline", "--dns", "#{HOST}=10.0.2.2"]
  end
end

General.describe "Faults" do
  fixture(:dev) { ProvisionedDevice.new(build) }
  fixture(:named) { FaultsSpec::NamedServerDevice.new(build) }

  # The test image: a big "1" over a strip of noise along the bottom, so that even as a PNG
  # (which compresses the rest to nothing) the file is a few KB: download faults cut it part
  # of the way through, after the /api/display response.
  def one_with_noise
    number = panel_number("1")
    strip = device.height - (device.height / 6)
    lambda do |x, y|
      next number.(x, y) if y < strip

      n = ((x * 374_761_393) + (y * 668_265_263)) & 0xFFFFFFFF
      n = ((n ^ (n >> 13)) * 1_274_126_177) & 0xFFFFFFFF
      (n ^ (n >> 16)).odd?
    end
  end

  # Serve the test image ("1") as the current screen; returns its path and the screenshot it
  # should give.
  def serve_one(mock)
    served = device_image(mock, "one", one_with_noise)
    mock.display = { image: "one", refresh_rate: 300 }
    served
  end

  # After the fault is gone, the next wake fetches and shows the image.
  def expect_image_next_time(s, mock = dev.mock)
    mock.clear_faults
    s.set_faults(net: nil)
    mock.next_request(path, timeout: 90) { s.wake }
    s.wait(state: "deep_sleep", timeout: 90, settle_ms: 300)
    expect(s).to show_image(expected, tolerance: 64)
  end

  before { dev.reset }

  let!(:served) { serve_one(dev.mock) }
  let(:path) { served.first }
  let(:expected) { served.last }
  # the image file's size: download faults cut it part of the way through (the OG's 48 KB BMP
  # at 10 and 20 KB)
  let(:size) { dev.mock.image_size(path) }

  describe "ServerErrors" do
    it "http 500 from api display is retried then sleeps", :smoke do
      dev.mock.set_fault("/api/display", status: 500)
      dev.boot do |s|
        st = s.wait_for_deep_sleep(timeout: 120)
        expect(dev.mock.count("/api/display")).to eq(5), "the firmware retries 5 times"
        expect(dev.mock.count("/api/log")).to eq(1), "and reports the failure"
        expect(st["wake_at_s"] - st["sim_time_s"]).to be_within(15).of(300)
        expect(s).not_to show_image(expected, tolerance: 64)
        expect_image_next_time(s)
      end
    end

    it "malformed json is rejected" do
      dev.mock.set_fault("/api/display", body: '{"status": 0, "image_url": ')
      dev.boot do |s|
        s.wait(console: /JSON deserialization error/, timeout: 120)
        s.wait_for_deep_sleep(timeout: 120)
        expect(dev.mock.count(path)).to eq(0)
        expect_image_next_time(s)
      end
    end
  end

  describe "BrokenDownloads" do
    it "truncated image is not shown" do
      dev.mock.set_fault("/images/*", truncate: [10_000, size / 4].min)
      dev.boot do |s|
        s.wait(console: /incomplete download/, timeout: 120)
        s.wait_for_deep_sleep(timeout: 120)
        expect(s).not_to show_image(expected, tolerance: 64)
        expect_image_next_time(s)
      end
    end

    it "connection reset mid download" do
      dev.boot(faults: { net: { tcp_cut: { after_bytes: [20_000, size / 2].min } } }) do |s|
        dev.mock.wait_for_request(path, timeout: 120)
        s.wait_for_deep_sleep(timeout: 120)
        expect(s).not_to show_image(expected, tolerance: 64)
        expect_image_next_time(s)
      end
    end

    it "stalled download times out" do
      dev.boot(faults: { net: { tcp_cut: { after_bytes: [20_000, size / 2].min, stall: true } } }) do |s|
        dev.mock.wait_for_request(path, timeout: 120)
        s.wait_for_deep_sleep(timeout: 180)
        expect(s).not_to show_image(expected, tolerance: 64)
      end
    end

    { "slow high latency link" => { latency_ms: 250, bandwidth_bps: 16_000 },
      "lossy link" => { loss: 0.05 } }.each do |link, net|
      it "#{link} still works" do
        dev.boot(faults: { net: }) do |s|
          s.wait(state: "deep_sleep", timeout: 180, settle_ms: 300)
          expect(s).to show_image(expected, tolerance: 64)
        end
      end
    end
  end

  describe "BadNetworks" do
    it "access point without internet" do
      dev.boot(faults: { net: { no_internet: true } }) do |s|
        st = s.wait(wifi_connected: true, timeout: 60)["status"]
        expect(st["ip"]).to eq("10.0.2.15"), "DHCP still works"
        s.wait_for_deep_sleep
        expect(dev.mock.requests).to be_empty
        expect_image_next_time(s)
      end
    end

    %w[servfail timeout].each do |fault|
      it "dns failure (#{fault})" do
        named.reset
        serve_one(named.mock)
        named.boot(faults: { net: { dns: fault } }) do |s|
          s.wait_for_deep_sleep
          expect(named.mock.requests).to be_empty
          expect_image_next_time(s, named.mock)
        end
      end
    end
  end

  describe "PowerLoss" do
    # A failing /api/display makes the firmware write NVS (its retry counter and the log it
    # stores for /api/log). Cut the power in the middle of those writes, leaving a torn page,
    # at a few different points.
    [1, 4, 9].each do |nth|
      it "power loss during nvs writes then boots (write #{nth})" do
        dev.mock.set_fault("/api/display", status: 500)
        dev.boot(faults: { power_loss: { partition: "nvs", op: "program", nth:, cut: "torn" } }) do |s|
          s.wait(console: /\[sim\] power lost: program/, timeout: 120)
          dev.mock.clear_faults
          dev.mock.next_request(path, timeout: 120)
          st = s.wait(state: "deep_sleep", timeout: 120, settle_ms: 300)["status"]
          expect(st["power_losses"]).to eq(1)
          expect(s).to show_image(expected, tolerance: 64)
          expect(dev.mock.count("/api/setup")).to eq(0), "must not lose its registration"
        end
      end
    end

    it "power loss while the bootloader writes otadata" do
      # The fresh flash has blank otadata, which the bootloader initialises on first boot.
      dev.boot(faults: { power_loss: { partition: "otadata", cut: "torn" } }) do |s|
        s.wait(console: /\[sim\] power lost: erase/, timeout: 60)
        s.wait(state: "deep_sleep", timeout: 120, settle_ms: 300)
        expect(s).to show_image(expected, tolerance: 64)
      end
    end

    it "power loss during ota keeps the old firmware" do
      firmware = File.binread(File.join(build, "firmware.bin"))
      url = dev.mock.set_file("/firmware.bin", "application/octet-stream", firmware)
      dev.mock.display_queue = [{ image: "one", update_firmware: true, firmware_url: url }]
      flash = nil
      s = dev.boot(faults: { power_loss: { partition: "ota_1", op: "program", nth: 200, cut: "torn" } }) do |s|
        dev.mock.wait_for_request("/firmware.bin", timeout: 120)
        c = s.wait_for_console(/\[sim\] power lost: program #200/, timeout: 120)
        flash = File.binread(s.flash)
        expect(c).to include("partition #{Flash.ota_slot_label(flash, 1)}")
        if s.wait(console: /2nd stage bootloader|BL init success/, timeout: 60)["line"]["text"].include?("bootloader")
          # the bootloader logs (not in the ESP32-S3's Arduino 2 builds): from the old slot
          old = Flash.partition_table(flash).find { |p| p.label == Flash.ota_slot_label(flash, 0) }
          s.wait(console: /Loaded app from partition at offset #{format('%#x', old.offset)}\b/, timeout: 60)
        end
        s.wait_for_deep_sleep(timeout: 120)
        s
      end
      expect(Flash.boot_slot(File.binread(s.flash))).to eq(Flash.ota_slot_label(flash, 0)),
                                                        "otadata still boots the old slot"
    end
  end

  describe "Peripherals" do
    it "panel busy stuck does not hang the device" do
      dev.boot(faults: { panel_busy_stuck: true }) do |s|
        dev.mock.wait_for_request(path, timeout: 120)
        s.wait_for_deep_sleep(timeout: 180)
      end
    end
  end
end
