# frozen_string_literal: true

require "fileutils"
require "tmpdir"

# --memcheck: whole onboarding and refresh cycles of the device under test under the
# memory checker (heap use-after-free, overflows, bad frees; stack high-water marks).
#
# The TRMNL X's and BWRY's own memcheck specs are with their devices' (core/trmnl_x/memcheck_spec.rb,
# core/trmnl_bwry_spec.rb).

# The IDF tasks the firmware doesn't size (see "leaves the idf task stacks room").
idf_tasks = %w[IDLE ipc]

General.describe "Memcheck" do
  describe "MemcheckOG" do
    def onboard(s, mock)
      s.wait(portal: true, timeout: 90)
      s.portal_connect("TRMNL-Sim", "password", server: mock.device_url)
      mock.wait_for_request("/api/display", timeout: 120)
      s.wait_for_deep_sleep(timeout: 120)
    end

    # A fresh device under memcheck onboarded against a mock showing "one"; yields both.
    def onboarded(**sim_kw)
      TrmnlSim::MockTrmnl.open do |mock|
        sim(erase: true, extra_args: ["--offline"], **sim_kw) do |s|
          device_image(mock, "one", panel_number("1"))
          mock.display = { image: "one", refresh_rate: 300 }
          onboard(s, mock)
          yield s, mock
        end
      end
    end

    it "sees clean onboarding and refresh cycles" do
      onboarded(memcheck: "halt") do |s, mock|
        mock.next_request("/api/display", timeout: 90) { s.wake }
        s.wait_for_deep_sleep
        mock.next_request("/api/display", timeout: 90) { device.button? ? s.press(150) : s.wake }
        s.wait_for_deep_sleep

        report = s.memcheck
        expect(report["enabled"]).to be(true)
        expect(report["violations"]).to eq([])
        heap = report["heap"]
        expect(heap["allocs"]).to be > 500
        expect(heap["internal"]["peak_bytes"]).to be > 50_000
        if device.psram_frame_buffers?
          expect(heap["psram"]["peak_bytes"]).to be > 1_000_000
        elsif !device.psram?
          expect(heap["psram"]["peak_bytes"]).to eq(0)
        end
        stacks = report["stacks"].to_h { |t| [t["task"], t] }
        expect(stacks["loopTask"]["size"]).to eq(8192)
        expect(stacks["loopTask"]["max_used"]).to be > 1000
        expect(stacks["loopTask"]["instances"]).to be >= 3 # one per boot
        expect_no_low_stacks(report["stacks"], but: idf_tasks) # (see "leaves the idf task stacks room")
      end
    end

    it "leaves the idf task stacks room" do
      # The IDF's own small tasks (IDLE, ipc), sized by the framework's sdkconfig, over an
      # onboarding.
      onboarded(memcheck: "log") do |s|
        report = s.memcheck
        expect(report["stacks"].select { |t| t["task"].start_with?("IDLE") }).not_to be_empty, "no IDLE task seen"
        expect_no_low_stacks(report["stacks"].select { |t| t["task"].start_with?(*idf_tasks) })
      end
    end

    it "does not use the sntp server name after free" do
      # Clock::sync hands configTime() the c_str() of a String that setTimeFromNTP frees on
      # return; SNTP keeps the pointer and resolves it again on every retry.
      # Without DNS (the server is reached by its IP) the NTP servers' names don't resolve,
      # so SNTP keeps retrying after setTimeFromNTP returned.
      onboarded(memcheck: "log", faults: { net: { dns: "servfail" } }) do |s, _mock|
        s.assert_no_memory_errors
      end
    end

    it "does not read the api display body past its end" do
      # bodyAsString() calls String::concat(body, size), which copies size + 1 bytes of a
      # body that isn't NUL-terminated. MockTrmnl sends a Content-Length, which takes the path
      # with a malloc'd body.
      onboarded(memcheck: "log") do |s, _mock|
        s.assert_no_memory_errors
      end
    end
  end
end
