# frozen_string_literal: true

RSpec.describe "OG Bluetooth fault recovery", env: "trmnl" do
  def advertised(s)
    eventually("firmware did not advertise", within: 30, every: 0.05) do
      s.status.dig("bluetooth", "advertising")
    end
  end

  def connect(s)
    central = nil
    # Status snapshots can still show the previous advertisement while the guest
    # processes disconnect. A phone also retries until advertising resumes.
    eventually("BLE did not become connectable", within: 5, every: 0.05) do
      central = TrmnlSim::Bluetooth.new(s)
    rescue TrmnlSim::Error => e
      raise unless e.message.include?("device is not connectable")

      false
    end
    central
  end

  def authenticate(s)
    advertised(s)
    identity = JSON.parse(s.preferences.fetch("ble_identity"))
    central = connect(s)
    [central, TrmnlSim::Security1.new(central, session: identity.fetch("sid"), proof: identity.fetch("pop")), identity]
  end

  def pairing_server(**options)
    device_mock(tls: true) do |mock|
      host = "trmnl.eliz.live"
      sim(erase: true, extra_args: ["--offline", "--dns", "#{host}=10.0.2.2"],
          host_ports: { 443 => mock.port }, **options) { |s| yield s, mock }
    end
  end

  def ready_code(session)
    reply = nil
    eventually("setup did not produce a code", within: 45, every: 0.1) do
      reply = session.command("status", attempt: 1)
      reply["state"] == "code_ready"
    end
    reply
  end

  describe "MalformedCommands" do
    it "rejects hostile encrypted commands without changing state or losing the cipher stream" do
      fresh_device do |s|
        _, session, identity = authenticate(s)
        expect(identity).not_to have_key("model")
        expect(session.command("capabilities")).not_to have_key("model")
        base = { v: 1, sid: identity.fetch("sid"), op: "status", attempt: 0 }
        commands = [
          ["{", "invalid_request"],
          [JSON.generate(base.merge(v: 2)), "unsupported_version"],
          [JSON.generate(base.merge(sid: "another-device")), "wrong_session"],
          [JSON.generate(base.merge(sid: "#{identity.fetch('sid')}\0")), "wrong_session"],
          [JSON.generate(base.merge(attempt: -1)), "invalid_request"],
          [JSON.generate(base.merge(attempt: true)), "invalid_request"],
          [JSON.generate(base.merge(op: "configure\0", attempt: 1)), "invalid_request"],
          [JSON.generate(base.merge(op: "networks", offset: 0, limit: 0)), "invalid_request"],
          [JSON.generate(base.merge(op: "networks", offset: 0, limit: 9)), "invalid_request"],
          [JSON.generate(base.merge(op: "configure", attempt: 1, ssid: "TRMNL-Sim\0evil",
                                    password: "password")), "invalid_credentials"]
        ]
        commands.each do |plain, error|
          expect(session.request(plain)).to include("error" => error, "attempt" => 0)
          expect(session.command("status")).to include("state" => "waiting", "attempt" => 0)
        end
        expect(s.status["halted_reason"]).to be_nil
      end
    end
  end

  describe "TransportBoundaries" do
    it "handles fragmented writes and survives a rejected oversized write" do
      fresh_device do |s|
        central, session, = authenticate(s)
        expect(session.command("status", padding: 300)).not_to have_key("error")
        expect { central.write(TrmnlSim::Bluetooth::CONTROL, "z" * 513) }
          .to raise_error(TrmnlSim::Bluetooth::AttError)
        expect(session.command("status")).to include("state" => "waiting")
        central.disconnect
        _, reconnected, = authenticate(s)
        expect(reconnected.command("status")).to include("state" => "waiting")
      end
    end
  end

  describe "SetupIsolation" do
    it "rejects management commands during provisioning without modifying saved networks" do
      fresh_device do |s|
        _, session, = authenticate(s)
        before = s.preferences("wificaptive")
        %w[wake_hold wake_release wifi_list].each do |op|
          expect(session.command(op)).to include("error" => "unsupported_operation")
        end
        expect(session.command("wifi_add", ssid: "evil", password: "password"))
          .to include("error" => "unsupported_operation")
        expect(s.preferences("wificaptive")).to eq(before)
        expect(session.command("status")).to include("state" => "waiting")
      end
    end
  end

  describe "NetworkPagination" do
    it "returns long scan results without truncation while the portal also requests rescans" do
      networks = Array.new(12) do |i|
        { ssid: format("network-%<index>02d-%<suffix>s", index: i, suffix: "x" * 21), rssi: -30 - i }
      end
      fresh_device(networks:) do |s|
        _, session, = authenticate(s)
        first = nil
        eventually("scan snapshot did not populate", within: 20, every: 0.1) do
          first = session.command("networks", offset: 0, limit: 8)
          first.fetch("networks").size == 8
        end
        expect(first).not_to have_key("error")
        expect(first.fetch("next_offset")).to be > 0
        s.portal_request("/scan?force=1")
        second = session.command("networks", offset: first.fetch("next_offset"), limit: 8)
        expect(second).not_to have_key("error")
        expect(second.fetch("next_offset")).to be_nil
        names = [*first.fetch("networks"), *second.fetch("networks")].map { _1.fetch("ssid") }
        expect(names).to match_array(networks.map { _1.fetch(:ssid) })
        expect(session.command("status")).to include("state" => "waiting")
      end
    end
  end

  describe "AuthenticationIsolation" do
    it "rejects unauthenticated control writes and a wrong proof, then accepts a fresh valid session" do
      fresh_device do |s|
        advertised(s)
        central = connect(s)
        expect { central.endpoint(TrmnlSim::Bluetooth::CONTROL, '{"op":"status"}') }
          .to raise_error(TrmnlSim::Bluetooth::AttError)
        central.disconnect
        central, session, identity = authenticate(s)
        expect(session.command("status")).not_to have_key("error")
        central.disconnect
        advertised(s)
        wrong = connect(s)
        expect do
          TrmnlSim::Security1.new(wrong, session: identity.fetch("sid"), proof: "wrong-proof")
        end.to raise_error(TrmnlSim::Bluetooth::AttError)
        wrong.disconnect
        # Leave a margin beyond the two-second cooldown: status is a sampled
        # snapshot, not the clock read inside the security callback.
        retry_at = s.status.fetch("sim_time_s") + 2.25
        eventually("authentication cooldown did not elapse", within: 10, every: 0.1) do
          s.status.fetch("sim_time_s") >= retry_at
        end
        _, valid, = authenticate(s)
        expect(valid.command("status")).not_to have_key("error")
      end
    end
  end

  describe "WrongWifiPassword" do
    it "keeps BLE reachable after a failed join and accepts a corrected attempt" do
      pairing_server(networks: [{ ssid: "TRMNL-Sim", password: "correct-password" }]) do |s, mock|
        _, session, = authenticate(s)
        boots = s.status.fetch("boot_count")
        session.command("configure", attempt: 1, ssid: "TRMNL-Sim", password: "incorrect-password")
        s.set_portal_client(false)
        eventually("failed join did not return to BLE", within: 30, every: 0.1) do
          session.command("status", attempt: 1)["state"] == "wifi_failed"
        end
        expect(s.status.fetch("boot_count")).to eq(boots)
        expect(s.preferences("wificaptive").values).not_to include("incorrect-password")
        expect(session.command("configure", attempt: 2, ssid: "TRMNL-Sim", password: "correct-password"))
          .not_to have_key("error")
        reply = nil
        eventually("corrected join did not complete setup", within: 45, every: 0.1) do
          reply = session.command("status", attempt: 2)
          reply["state"] == "code_ready"
        end
        expect(reply).to include("friendly_id" => mock.friendly_id, "attempt" => 2)
        expect(session.command("ack_code", attempt: 2)).to include("state" => "handed_off")
      end
    end
  end

  describe "SetupRetry" do
    it "retries a failed API setup without accepting a conflicting duplicate Wi-Fi submission" do
      pairing_server do |s, mock|
        mock.setup = nil
        _, session, = authenticate(s)
        expect(session.command("configure", attempt: 1, ssid: "TRMNL-Sim", password: "password"))
          .not_to have_key("error")
        s.set_portal_client(false)
        eventually("setup did not fail", within: 45, every: 0.1) do
          session.command("status", attempt: 1)["state"] == "setup_failed"
        end
        expect(session.command("configure", attempt: 1, ssid: "different-network", password: "password"))
          .to include("attempt" => 1, "state" => "setup_failed")
        expect(s.preferences("wificaptive").values).not_to include("different-network")
        mock.setup = {}
        expect(session.command("retry_setup", attempt: 2)).to include("error" => "stale_attempt")
        expect(session.command("retry_setup", attempt: 1)).not_to have_key("error")
        expect(ready_code(session)["friendly_id"]).to eq(mock.friendly_id)
        expect(session.command("ack_code", attempt: 1)).to include("state" => "handed_off")
      end
    end
  end

  describe "InterruptedHandoff" do
    it "recovers the code after a reset and clears its pending attempt only after acknowledgement" do
      pairing_server do |s, mock|
        central, session, identity = authenticate(s)
        session.command("configure", attempt: 1, ssid: "TRMNL-Sim", password: "password")
        s.set_portal_client(false)
        expect(ready_code(session)["friendly_id"]).to eq(mock.friendly_id)
        expect(s.preferences).to include("ble_pending" => "1")
        s.reset
        expect { central.endpoint(TrmnlSim::Bluetooth::CONTROL, "stale") }.to raise_error(TrmnlSim::Error)
        _, restored, restored_identity = authenticate(s)
        expect(restored_identity).to eq(identity)
        expect(ready_code(restored)).to include("attempt" => 1, "friendly_id" => mock.friendly_id)
        expect(restored.command("ack_code", attempt: 2)).to include("error" => "stale_attempt")
        expect(s.preferences).to include("ble_pending" => "1")
        expect(restored.command("ack_code", attempt: 1)).to include("state" => "handed_off")
        expect(s.preferences).not_to have_key("ble_pending")
        s.wait(state: "deep_sleep", timeout: 30)
        expect(s.status.dig("bluetooth", "initialized")).to be(false)
      end
    end
  end

  describe "IdentityRecovery" do
    fixture(:dev) { ProvisionedDevice.new(build) }
    before { dev.reset }

    it "generates a missing identity on timer wake and keeps it stable across later refreshes" do
      dev.boot_asleep do |s|
        s.wait_for_deep_sleep
        s.delete_preference("ble_identity")
        request = dev.mock.next_request("/api/display") { s.wake }
        s.wait(state: "deep_sleep", timeout: 30)
        identity = JSON.parse(s.preferences.fetch("ble_identity"))
        expect(request).to have_header("BLE-Key", identity.fetch("pop"))
        expect(request).to have_header("BLE-Session-ID", identity.fetch("sid"))
        expect(request).to have_header("Update-Source", "timer")
        expect(s.status.dig("bluetooth", "initialized")).to be(false)
        second = dev.mock.next_request("/api/display") { s.wake }
        s.wait(state: "deep_sleep", timeout: 30)
        expect(second).to have_header("BLE-Key", identity.fetch("pop"))
        expect(s.preferences.fetch("ble_identity")).to eq(JSON.generate(identity))
      end
    end

    it "uses a remaining Wi-Fi network when the first saved slot was removed" do
      dev.boot_asleep do |s|
        s.wait_for_deep_sleep
        s.set_preference("wifi_2_ssid", "string", "TRMNL-Sim", namespace: "wificaptive")
        s.set_preference("wifi_2_pswd", "string", "password", namespace: "wificaptive")
        s.delete_preference("wifi_0_ssid", namespace: "wificaptive")
        request = dev.mock.next_request("/api/display") { s.wake }
        expect(request).to have_header("Update-Source", "timer")
        s.wait(state: "deep_sleep", timeout: 30)
        expect(s.status.fetch("portal_url")).to be_nil
        expect(s.preferences("wificaptive")).to include("wifi_2_ssid" => "TRMNL-Sim")
        expect(s.preferences("wificaptive")).not_to have_key("wifi_0_ssid")
      end
    end

    it "replaces the identity after a server-requested device wipe" do
      dev.mock.display = { image: "default", status: 500, refresh_rate: 300 }
      dev.boot_asleep do |s|
        old_identity = JSON.parse(s.preferences.fetch("ble_identity"))
        s.wake
        s.wait(portal: true, min_boots: 2, timeout: 30)
        advertised(s)
        identity = JSON.parse(s.preferences.fetch("ble_identity"))
        expect(identity.fetch("sid")).not_to eq(old_identity.fetch("sid"))
        expect(identity.fetch("pop")).not_to eq(old_identity.fetch("pop"))
        expect(s.preferences).not_to include("api_key", "friendly_id", "ble_pending")
        _, session, = authenticate(s)
        expect(session.command("status")).to include("state" => "waiting", "attempt" => 0)
      end
    end
  end
end
