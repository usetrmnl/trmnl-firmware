# frozen_string_literal: true

require "fileutils"
require "tmpdir"

# --memcheck on the TRMNL X: its factory flow, onboarding and refreshes under the memory checker
# (see general/tooling/memcheck_spec.rb for the device under test's).

module TrmnlXMemcheckSpec
  # A factory-fresh X (the QA flow and modem flashing run under memcheck too, once per
  # firmware/simulator: it comes from the setup cache and is only cached if it was clean).
  class ShippedMemcheckX
    def initialize
      inputs = { build: SetupCache.build_id(TrmnlX.build), turbo: Builds::TURBO }
      @cache, = SetupCache.entry("x-shipped-memcheck", inputs) do |dir|
        TrmnlX.sim(flash: File.join(dir, "flash.bin"), erase: true, memcheck: "halt", name: "x-memcheck-factory") do |s|
          s.wait_for_console(/Entering shipment mode light sleep loop/, timeout: 180)
        end
        {}
      end
      @dir = Dir.mktmpdir("trmnl-x-memcheck-")
    end

    # Boot a copy of it.
    def boot(**, &)
      flash = File.join(@dir, "flash-#{Dir.children(@dir).size}.bin")
      FileUtils.cp(File.join(@cache, "flash.bin"), flash)
      TrmnlX.sim(flash:, **, &)
    end

    def close = FileUtils.rm_rf(@dir)
  end
end

RSpec.describe "TRMNL X memcheck", env: "TRMNL_X" do
  fixture(:shipped_x) { TrmnlXMemcheckSpec::ShippedMemcheckX.new }

  # A factory-fresh X (see TrmnlXMemcheckSpec::ShippedMemcheckX), then onboarding on 2.4 GHz: WiFi
  # stop/start around the portal is where Arduino once freed a netif the event task still used.
  describe "MemcheckX" do
    it "sees clean onboarding and refresh" do
      TrmnlSim::MockTrmnl.open do |mock|
        shipped_x.boot(memcheck: "halt") do |s|
          mock.set_png("dots", ->(x, y) { ((x / 8) + (y / 8)) & 1 })
          mock.display = { image: "dots", refresh_rate: 300 }
          TrmnlX.onboard(s, mock, TrmnlX::SSID_24)
          mock.next_request("/api/display", timeout: 120) { s.touch("center", ms: 150) }
          s.wait_for_deep_sleep(timeout: 120)

          report = s.memcheck
          expect(report["violations"]).to eq([])
          heap = report["heap"]
          expect(heap["allocs"]).to be > 500
          expect(heap["psram"]["peak_bytes"]).to be > 1_000_000 # frame buffers
          stacks = report["stacks"].to_h { |t| [t["task"], t] }
          expect(stacks.keys).to include("loopTask", "sys_evt", "tiT", "IDLE0", "IDLE1")
          expect_no_low_stacks(report["stacks"])
        end
      end
    end
  end

  describe "MemcheckXBmp" do
    # display_show_image flips an uncompressed BMP with the panel's dimensions: an 800x480 BMP
    # (48 KB) is flipped as 1872x1404, far past the end of its buffer.
    it "flips a bmp image within its buffer" do
      TrmnlSim::MockTrmnl.open do |mock|
        shipped_x.boot(memcheck: "log") do |s|
          mock.display = { image: "default", refresh_rate: 300 } # the OG's BMP
          TrmnlX.onboard(s, mock, TrmnlX::SSID_24)
          s.assert_no_memory_errors
        end
      end
    end
  end
end
