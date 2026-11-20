# frozen_string_literal: true

require "fileutils"
require "tmpdir"

# Save points (on the device under test): save a device in deep sleep, restore it in a fresh
# simulator, and carry on where it left off: same screen, no re-onboarding, both wake sources.

module SavepointsSpec
  # The provisioned device's save point in deep sleep, showing `seven` (in `dir`, a temporary
  # directory the other examples' files go to too): `screen` is the screen when it was taken (a
  # gray panel's differs from `seven` a little), `status` the device's status.
  SavePoint = Struct.new(:dir, :path, :screen, :status) do
    def close = FileUtils.rm_rf(dir)
  end
end

General.describe "Save points" do
  # Another device's firmware, which must refuse this device's save points.
  other_env = device.env == "trmnl" ? "trmnl_4clr" : "trmnl"

  fixture(:dev) { ProvisionedDevice.new(build) }

  fixture(:saved) do
    dir = Dir.mktmpdir("trmnl-savepoints-")
    path = File.join(dir, "asleep.trmnlsave")
    image, = device_image(dev.mock, "seven", device_number("7"))
    dev.mock.display = { image: "seven", refresh_rate: 300 }
    dev.boot do |s|
      dev.mock.wait_for_request(image, timeout: 120)
      status = s.wait_for_deep_sleep(timeout: 120, display_idle: true)
      # (whether it shows `seven` right is general/refresh/refresh_cycle_spec's business; a save point must
      # bring back whatever is on screen)
      screen = s.screenshot
      info = s.save_point(path, label: "asleep showing 7")
      raise "not a deep-sleep save point: #{info}" unless info["deep_sleep"] && info["label"] == "asleep showing 7"

      SavepointsSpec::SavePoint.new(dir, path, screen, status)
    end
  end

  describe "SavePoints" do
    let(:eight) { device_image(dev.mock, "eight", device_number("8")) }

    before do
      saved # taken first, with the mock serving `seven`
      dev.reset
      eight
      dev.mock.display = { image: "eight", refresh_rate: 300 }
    end

    # A simulator resumed from `saved`, once it is in deep sleep (and its status then).
    def restored
      dev.restore(saved.path) { |s| yield s, s.wait_for_deep_sleep(timeout: 30) }
    end

    it "restores into the same deep sleep" do
      restored do |s, st|
        expect(st["boot_count"]).to eq(saved.status["boot_count"])
        expect(st["display_refreshes"]).to eq(saved.status["display_refreshes"])
        expect(st["wake_at_s"]).to be_within(0.0005).of(saved.status["wake_at_s"])
        expect(st["sim_time_s"]).to be >= saved.status["sim_time_s"]
        expect(s).to show_image(saved.screen, tolerance: 0, max_ratio: 0)
        s.wait_for_console(/restored save point "asleep showing 7": deep sleep, wakes in/)
        expect(dev.mock.requests).to be_empty
      end
    end

    it "timer wake refreshes without onboarding", :smoke do
      restored do |s|
        req = dev.mock.next_request("/api/display", timeout: 120) { s.wake }
        expect(req).to have_header("Access-Token", dev.mock.api_key)
        expect(req).to have_header("Update-Source", "timer")
        s.wait(console: /rst:0x5 \(DSLEEP\)/, min_boots: saved.status["boot_count"] + 1)
        s.wait_for_deep_sleep(timeout: 120, display_idle: true)
        expect(s).to show_image(eight[1])
        expect(dev.mock.paths).not_to include("/api/setup")
      end
    end

    it "button wakes a restored device", needs: :button do
      restored do |s|
        req = dev.mock.next_request("/api/display", timeout: 120) { s.press(150) }
        expect(req).to have_header("Update-Source", device.button_source)
        s.wait_for_deep_sleep(timeout: 120)
      end
    end

    it "in memory slot goes back in time" do
      restored do |s|
        slot = s.save_point
        expect(s.save_points.map { _1["id"] }).to eq([slot["id"]])
        s.wake
        dev.mock.wait_for_request(eight[0], timeout: 120)
        st = s.wait_for_deep_sleep(timeout: 120, display_idle: true)
        expect(s).to show_image(eight[1])
        info = s.restore(id: slot["id"])
        expect(info["label"]).to eq(slot["label"])
        back = s.wait_for_deep_sleep(timeout: 30)
        expect(s).to show_image(saved.screen, tolerance: 0, max_ratio: 0)
        expect(back["boot_count"]).to eq(st["boot_count"] - 1)
        expect(back["sim_time_s"]).to be < st["sim_time_s"]
      end
    end

    it "power off save point boots the saved flash" do
      path = File.join(saved.dir, "off.trmnlsave")
      restored do |s|
        s.wake
        s.wait(console: /rst:0x5 \(DSLEEP\)/)
        s.pause
        begin
          info = s.save_point(path)
        rescue TrmnlSim::Error => e # caught mid-refresh: try again once it is idle
          expect(e.message).to include("refreshing")
          s.pause(false)
          s.wait(display_idle: true, state: "deep_sleep", timeout: 120)
          skip "the device refreshed before it could be paused"
        end
        expect(info["deep_sleep"]).to be(false)
      end
      after = dev.mock.cursor
      dev.restore(path) do |s|
        s.wait_for_console(/powering on/)
        s.wait(console: /rst:0x1 \(POWERON\)/)
        req = dev.mock.wait_for_request("/api/display", after:, timeout: 120)
        expect(req).to have_header("Access-Token", dev.mock.api_key)
        expect(dev.mock.paths.drop(after)).not_to include("/api/setup")
      end
    end

    it "bad files are refused" do
      junk = File.join(saved.dir, "junk.trmnlsave")
      File.binwrite(junk, "not a save point")
      restored do |s|
        expect { s.restore(junk) }.to raise_error(TrmnlSim::Error, /not a trmnl-sim save point/)
        expect { s.restore(id: 99) }.to raise_error(TrmnlSim::Error, /no save point #99/)
        expect(s.status["state"]).to eq("deep_sleep")
      end
      expect { sim(restore: junk) { nil } }.to raise_error(TrmnlSim::Error, /not a trmnl-sim save point/)
    end

    it "other firmware is refused", needs_build: other_env do
      other = Builds.for_env(other_env)
      expect { sim(other, restore: saved.path) { nil } }
        .to raise_error(TrmnlSim::Error, /restore it with the build it was saved from/)
      sim(other, erase: true) do |s|
        expect { s.restore(saved.path) }.to raise_error(TrmnlSim::Error, /save point was taken with firmware/)
      end
    end
  end
end
