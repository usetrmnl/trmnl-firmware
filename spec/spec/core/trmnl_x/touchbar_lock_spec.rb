# frozen_string_literal: true

# The TRMNL X touch bar lock (touchbar_locked_wake in src/touchbar_actions.cpp, run_hold_prompt in
# src/menu.cpp). Shared helpers: spec/support/touchbar.rb.

RSpec.describe "TRMNL X touch bar: lock", env: "TRMNL_X" do
  fixture(:shipped) { TrmnlX::ShippedX.new }
  fixture(:dev) { TrmnlX::ProvisionedX.new(shipped) }

  include_context "provisioned X"

  # Open Settings, then Lock Touchbar..., leaving its hold prompt on the screen.
  def ask_to_lock(s)
    TrmnlX.open_settings_menu(s)
    TrmnlX.menu_tap(s, "right", /Diagnostics\.\.\. -> Reset Wi-Fi/)
    TrmnlX.menu_tap(s, "right", /Reset Wi-Fi\.\.\. -> Shut Down/)
    TrmnlX.menu_tap(s, "right", /Shut Down\.\.\. -> Lock Touchbar/)
    c = s.status["console_total"]
    s.touch("center", ms: 150)
    s.wait(console: /hold_prompt_show_screen end/, since: c, timeout: 15)
  end

  # Hold the middle of a sleeping locked X for 5 s, leaving the unlock prompt on the screen.
  def ask_to_unlock(s)
    c = s.status["console_total"]
    s.touch("center", ms: 5500)
    s.wait(console: /hold_prompt_show_screen end/, since: c, timeout: 30)
    expect(s.console(c).join("\n")).to match(/asking to unlock/)
  end

  def lock(s) = s.set_preference("touch_locked", "u8", 1)

  describe "locking" do
    it "locks on a 3 s middle hold, and goes back to the image" do
      boot_two_images do |s, _one, two|
        ask_to_lock(s)
        c = s.status["console_total"]
        s.touch("center", ms: 3500)
        s.wait(console: /Hold prompt: held/, since: c, timeout: 15)
        s.wait(state: "deep_sleep", timeout: 30, settle_ms: 500)
        expect(s.console(c).join("\n")).not_to match(/staying awake/i)
        expect(s.preferences["touch_locked"]).to eq("1")
        expect(s).to show_image(two, tolerance: 64, max_ratio: 0)
      end
    end

    it "goes back to Settings on another tap" do
      boot_two_images do |s|
        ask_to_lock(s)
        TrmnlX.menu_tap(s, "left", /Hold prompt: another input - cancelled/, screen: true)
        TrmnlX.menu_tap(s, "right", /Diagnostics\.\.\. -> Reset Wi-Fi/)
        s.wait(state: "deep_sleep", timeout: 60, settle_ms: 500)
        expect(s.preferences["touch_locked"]).to be_nil
      end
    end

    it "goes back to Settings on letting go early" do
      boot_two_images do |s|
        ask_to_lock(s)
        c = s.status["console_total"]
        s.touch("center", ms: 1000)
        s.wait(console: /Hold prompt: let go early - cancelled/, since: c, timeout: 15)
        s.wait(console: /menu_show_screen end/, since: c, timeout: 15)
        s.wait(state: "deep_sleep", timeout: 60, settle_ms: 500)
        expect(s.preferences["touch_locked"]).to be_nil
      end
    end

    it "closes the menu on a timeout" do
      boot_two_images do |s, _one, two|
        ask_to_lock(s)
        s.wait(state: "deep_sleep", timeout: 60, settle_ms: 500)
        expect(s.preferences["touch_locked"]).to be_nil
        expect(s).to show_image(two, tolerance: 64, max_ratio: 0)
      end
    end
  end

  describe "while locked" do
    it "ignores taps and holds: no fetch, the same screen, the same refresh" do
      boot_two_images do |s, _one, two|
        lock(s)
        dev.mock.requests.clear
        wake_at = s.status["wake_at_s"]

        [["left", 150], ["center", 150], ["right", 150], ["center", 3000]].each do |zone, ms|
          touch_and_sleep(s, zone, ms, /Touch bar locked - ignoring the touch/)
          expect(s).to show_image(two, tolerance: 64, max_ratio: 0)
          expect(s.status["wake_at_s"]).to be_within(1).of(wake_at)
        end
        expect(dev.mock.requests).to be_empty
        expect(s.preferences["touch_locked"]).to eq("1")
      end
    end

    it "ignores a 5 s hold on the side" do
      boot_two_images do |s, _one, two|
        lock(s)
        dev.mock.requests.clear
        c = s.status["console_total"]
        t0 = s.status["sim_time_s"]
        s.touch("right", ms: 6000)
        sleep 0.2 while s.status["sim_time_s"] < t0 + 7
        s.wait(state: "deep_sleep", timeout: 15, settle_ms: 500)
        expect(s.console(c).join("\n")).not_to match(/asking to unlock/)
        expect(dev.mock.requests).to be_empty
        expect(s).to show_image(two, tolerance: 64, max_ratio: 0)
      end
    end

    it "still refreshes on the timer" do
      boot_two_images do |s|
        lock(s)
        dev.mock.next_request("/api/display", timeout: 120) { s.wake }
        s.wait(state: "deep_sleep", timeout: 60, settle_ms: 500)
        expect(s.preferences["touch_locked"]).to eq("1")
      end
    end
  end

  describe "unlocking" do
    it "unlocks on a 5 s middle hold, then another 3 s hold" do
      boot_two_images do |s, _one, two|
        lock(s)
        ask_to_unlock(s)
        c = s.status["console_total"]
        s.touch("center", ms: 3500)
        s.wait(console: /Touch bar unlocked/, since: c, timeout: 15)
        s.wait(state: "deep_sleep", timeout: 30, settle_ms: 500)
        expect(s.preferences["touch_locked"]).to be_nil
        expect(s).to show_image(two, tolerance: 64, max_ratio: 0)

        touch_and_sleep(s, "left", 150, /Back button tapped/)
      end
    end

    it "takes a new hold: the 5 s one going on does not count" do
      boot_two_images do |s|
        lock(s)
        c = s.status["console_total"]
        s.touch("center", ms: 12_000)
        s.wait(console: /hold_prompt_show_screen end/, since: c, timeout: 30)
        s.wait(console: /Hold prompt: timeout/, since: c, timeout: 60)
        expect(s.console(c).join("\n")).not_to match(/Hold prompt: hold started/)
        s.wait(state: "deep_sleep", timeout: 30, settle_ms: 500)
        expect(s.preferences["touch_locked"]).to eq("1")
      end
    end

    { "another tap" => ["right", 150], "letting go early" => ["center", 1000] }.each do |name, (zone, ms)|
      it "stays locked and goes back to the image on #{name}" do
        boot_two_images do |s, _one, two|
          lock(s)
          ask_to_unlock(s)
          c = s.status["console_total"]
          s.touch(zone, ms:)
          s.wait(console: /Hold prompt: .*cancelled/, since: c, timeout: 15)
          s.wait(state: "deep_sleep", timeout: 30, settle_ms: 500)
          expect(s.preferences["touch_locked"]).to eq("1")
          expect(s).to show_image(two, tolerance: 64, max_ratio: 0)
        end
      end
    end

    { "a reset" => :reset, "a power cycle" => :power_cycle }.each do |name, action|
      it "unlocks on #{name}" do
        boot_two_images do |s|
          lock(s)
          c = s.status["console_total"]
          s.public_send(action)
          s.wait(console: /Reset \(reason \d+\) - unlocking the touch bar/, since: c, timeout: 30)
          s.wait(state: "deep_sleep", timeout: 60, settle_ms: 500)
          expect(s.preferences["touch_locked"]).to be_nil
        end
      end
    end

    it "unlocks when the server resets the device" do
      boot_two_images do |s|
        lock(s)
        dev.mock.display = { image: "two", reset_firmware: true, refresh_rate: 300 }
        s.wake
        s.wait(portal: true, timeout: 60)
        expect(s.preferences["touch_locked"]).to be_nil
      end
    end

    it "unlocks on the portal's soft reset" do
      boot_two_images do |s|
        lock(s)
        s.set_preference("wifi_0_ssid", "string", "", namespace: "wificaptive") # so the wake opens the portal
        s.wake
        boots = s.wait(portal: true, timeout: 60)["status"]["boot_count"]
        expect(s.preferences["touch_locked"]).to eq("1")
        begin
          s.portal_request("/soft-reset", retry_for: 0)
        rescue *TrmnlSim::Simulator::CONNECTION_ERRORS
          nil # the device may restart before it answers
        end
        s.wait(min_boots: boots + 1, portal: true, timeout: 60)
        expect(s.preferences["touch_locked"]).to be_nil
      end
    end

    it "stays locked and goes back to the image on a timeout" do
      boot_two_images do |s, _one, two|
        lock(s)
        ask_to_unlock(s)
        s.wait(state: "deep_sleep", timeout: 60, settle_ms: 500)
        expect(s.preferences["touch_locked"]).to eq("1")
        expect(s).to show_image(two, tolerance: 64, max_ratio: 0)
      end
    end
  end
end
