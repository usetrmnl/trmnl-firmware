# frozen_string_literal: true

# Fault injection on the TRMNL X: a missing fuel gauge or one that loses its configuration, a
# panel whose PMIC never powers up, a modem that stops answering, downloads cut on the 5 GHz
# (modem) path, and power loss while the firmware writes NVS.

fuel_gauge = 0x55 # BQ27427
touch_bar = 0x44 # IQS323

RSpec.describe "Faults on the TRMNL X", env: "TRMNL_X" do
  fixture(:shipped) { TrmnlX::ShippedX.new }
  fixture(:dev) { TrmnlX::ProvisionedX.new(shipped) } # on the 5 GHz network: all HTTP goes through the modem

  describe "FaultsX" do
    before do
      dev.reset(display: { image: "five", refresh_rate: 300 })
      @five = dev.mock.set_png("five", TrmnlX.digits("5"))
    end

    it "carries on with the fuel gauge absent" do
      dev.boot(faults: { i2c_absent: [fuel_gauge] }) do |s|
        req = dev.mock.wait_for_request("/api/display", timeout: 120)
        # The firmware reports -1 when it can't read the gauge, and carries on.
        expect(req).to have_header("Battery-Voltage", -1.0)
        s.wait(state: "deep_sleep", timeout: 120, settle_ms: 300)
        expect(s).to show_image(@five, tolerance: 64, max_ratio: 0)
      end
    end

    # bl_init() restarts the device when the IQS323 task fails to initialize, on every
    # boot: a device whose touch controller died boot-loops (never reaching the server,
    # draining the battery) instead of carrying on without its touch bar.
    it "keeps running with the touch controller absent" do
      dev.boot(faults: { i2c_absent: [touch_bar] }) do |s|
        dev.mock.wait_for_request("/api/display", timeout: 15)
        expect(s.status["boot_count"]).to be < 3
      end
    end

    it "writes the golden file again when the fuel gauge loses its configuration" do
      # A power-on reset (battery disconnected) puts the gauge back on factory data memory
      # with ITPOR set; the next wake has to write the golden file again.
      h = dev.boot_asleep do |s|
        s.wait(state: "deep_sleep", timeout: 15)
        s.set_faults(gauge_reset: true)
        s.set_faults(gauge_reset: nil)
        headers = dev.mock.next_request("/api/display", timeout: 30) { s.wake }.headers
        s.wait(state: "deep_sleep", timeout: 30)
        headers
      end
      # Readable again (an unconfigured gauge is reported as -1), with the golden file's
      # 6000 mAh design capacity rather than the factory 1340 mAh.
      expect(h.values_at("Gauge-SOC", "Gauge-Capacity")).to eq(%w[83 4980/6000])
      # The factory calibration inverts the current; the firmware flips CC Gain's sign back.
      expect(h["Battery-Current"]).to eq("-50")
    end

    it "reinitializes a touch controller reset while asleep" do
      # The wake stub reads SHOW_RESET; the IQS323 task reinitializes it (its "IQS323 Task:"
      # lines go to Serial, which production X builds don't mirror to the log).
      expect(touch_bar_fault("reset")).to include("wakeup_stub_iqs_status.status: 0x80")
    end

    it "survives a touch controller i2c lockup" do
      touch_bar_fault("lockup")
    end

    it "survives a touch controller ati error" do
      touch_bar_fault("ati_error")
    end

    {
      "station mode fails" => %w[AT+CWMODE],
      "auto connect setting fails" => %w[AT+CWAUTOCONN],
      "baud rate change fails" => %w[AT+UART_CUR],
      "cannot join" => %w[AT+CWJAP=],
      "http headers rejected" => %w[AT+HTTPCHEAD],
      "time sync fails" => %w[AT+CIPSNTPCFG],
      "time query fails" => %w[AT+CIPSNTPTIME],
      "mac and signal queries fail" => %w[AT+CIPSTAMAC AT+CWJAP?]
    }.each do |what, prefixes|
      it "sleeps again when the modem #{what}" do
        modem_errors(*prefixes)
      end
    end

    it "sleeps without a picture on a panel power failure" do
      # The PMIC's power good never comes: the panel gets no drive voltages.
      dev.boot(faults: { panel_busy_stuck: true }) do |s|
        dev.mock.wait_for_request("/images/five.png", timeout: 120)
        s.wait(state: "deep_sleep", timeout: 180)
        expect(s).not_to show_image(@five, tolerance: 64, max_ratio: 0)
      end
    end

    it "recovers from an unresponsive modem" do
      dev.boot(faults: { modem_unresponsive: true }) do |s|
        s.wait(console: /Connection failed/, timeout: 180)
        s.wait(state: "deep_sleep", timeout: 180)
        expect(dev.mock.requests).to eq([])
        s.clear_faults
        s.wake
        dev.mock.wait_for_request("/images/five.png", timeout: 120)
        s.wait(state: "deep_sleep", timeout: 120, settle_ms: 300)
        expect(s).to show_image(@five, tolerance: 64, max_ratio: 0)
      end
    end

    it "retries a download cut on the modem path" do
      # /api/display (about 230 bytes) gets through; the PNG (about 1.8 kB) does not.
      dev.boot(faults: { net: { tcp_cut: { after_bytes: 1_000 } } }) do |s|
        s.wait(console: /Image download failed/, timeout: 180)
        s.wait(state: "deep_sleep", timeout: 120)
        expect(dev.mock.count("/images/five.png")).to eq(5), "the firmware retries 5 times"
        expect(s).not_to show_image(@five, tolerance: 64, max_ratio: 0)
        s.clear_faults
        dev.mock.next_request("/images/five.png", timeout: 120) { s.wake }
        s.wait(state: "deep_sleep", timeout: 120, settle_ms: 300)
        expect(s).to show_image(@five, tolerance: 64, max_ratio: 0)
      end
    end

    it "boots after a power loss during nvs write" do
      # The X writes NVS early on every boot; tear the first page program.
      dev.boot(faults: { power_loss: { partition: "nvs", op: "program", cut: "torn" } }) do |s|
        s.wait(console: /\[sim\] power lost: program #1 .* in partition nvs/, timeout: 120)
        dev.mock.wait_for_request("/images/five.png", timeout: 120)
        s.wait(state: "deep_sleep", timeout: 120, settle_ms: 300)
        expect(s).to show_image(@five, tolerance: 64, max_ratio: 0)
        expect(dev.mock.count("/api/setup")).to eq(0), "must not lose its registration"
      end
    end

    private

    # Asleep, the touch controller starts misbehaving; wake by a tap, then by the timer (with
    # the fault cleared). Returns the console.
    def touch_bar_fault(kind)
      dev.boot_asleep do |s|
        s.wait(state: "deep_sleep", timeout: 15)
        s.set_faults(touch_bar: kind)
        dev.mock.next_request("/api/display", timeout: 15) { s.touch("center", ms: 150) }
        s.wait(state: "deep_sleep", timeout: 15, settle_ms: 300)
        s.set_faults(touch_bar: nil)
        dev.mock.next_request("/api/display", timeout: 15) { s.wake }
        s.wait(state: "deep_sleep", timeout: 15)
        s.console(0).join("\n")
      end
    end

    # Asleep on 5 GHz, the modem starts answering ERROR to these commands; the device must
    # still get back to sleep after a timer wake. Returns the status.
    def modem_errors(*prefixes)
      dev.boot_asleep do |s|
        s.wait(state: "deep_sleep", timeout: 15)
        s.set_faults(modem_at_errors: prefixes)
        s.wake
        st = s.wait(state: "deep_sleep", timeout: 30, settle_ms: 300)["status"]
        expect(st["state"]).not_to eq("halted")
        st
      end
    end
  end
end
