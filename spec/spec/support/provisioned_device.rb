# frozen_string_literal: true

require "fileutils"
require "tmpdir"

# A mock API server plus a device that already completed onboarding against a server like it:
# `boot` powers on a copy of its flash, `boot_asleep` resumes it in the deep sleep that followed
# onboarding (a save point), skipping the first refresh cycle. Both take a block (the simulator
# is closed afterwards) or return the simulator.
#
# The onboarded device comes from the setup cache (see SetupCache). It knows the server by the
# port of the mock it was onboarded with, so every simulator started here gets `--host-port` to
# send that port to this fixture's mock.
#
# Subclasses change what the server serves by overriding `new_mock` (and `cache_name`, so their
# onboarded devices are cached apart).
class ProvisionedDevice
  SIM_ARGS = ["--offline"].freeze
  DEVICE_HOST = "10.0.2.2"

  attr_reader :build, :panel_size, :mock, :cache, :host_ports

  # `build`: the firmware (a group's `build`). `panel_size`: serve the default image as a 1-bit
  # PNG of that size (as the TRMNL server does for other panels) instead of the OG's 800x480
  # BMP; default, what the build's device takes (see Device#default_bmp?).
  def initialize(build, panel_size: :auto)
    @build = build
    if panel_size == :auto
      device = Builds.device_of(build)
      panel_size = device.nil? || device.default_bmp? ? nil : device.size
    end
    @panel_size = panel_size
    inputs = { build: SetupCache.build_id(build), memcheck: Builds::MEMCHECK, turbo: Builds::TURBO,
               args: sim_args, host: device_host }
    inputs[:panel_size] = panel_size if panel_size
    @cache, meta = SetupCache.entry("#{cache_name}-#{File.basename(build)}", inputs) { |dir| onboard(dir) }
    @mock = new_mock
    @host_ports = { meta.fetch("port") => mock.port }
    @dir = Dir.mktmpdir("trmnl-provisioned-")
  end

  def cache_name = "provisioned"
  def sim_args = SIM_ARGS
  def device_host = DEVICE_HOST

  # A mock server for this device (onboarding and tests): serving images/default (panel-sized
  # with `panel_size`) and a 300 s refresh rate. Subclasses change what it serves.
  def new_mock
    mock = TrmnlSim::MockTrmnl.new
    if panel_size
      w, h = panel_size
      number = TrmnlSim::Images.big_number("0")
      mock.images["default.png"] = TrmnlSim::Images.png_image(->(x, y) { number.(x, y) ? 0 : 1 }, w, h)
    end
    mock.device_host = device_host
    mock.display = { image: "default", refresh_rate: 300 }
    mock
  end

  # Forget what the mock saw and serve the default image again (between examples).
  def reset(display: { image: "default", refresh_rate: 300 }) = mock.reset(display:)

  # Power on a copy of the provisioned device.
  def boot(**kw, &)
    flash = File.join(@dir, "flash-#{Dir.children(@dir).size}.bin")
    FileUtils.cp(File.join(cache, "flash.bin"), flash)
    start(flash:, **kw, &)
  end

  # The provisioned device in deep sleep right after onboarding (showing the mock's default
  # image); wake it, press, touch... to carry on.
  def boot_asleep(**kw, &) = restore(File.join(cache, "asleep.trmnlsave"), **kw, &)

  # A simulator resumed from a save point of this device (reaching this mock).
  def restore(path, **kw, &) = start(restore: path, **kw, &)

  def close
    mock.close
    FileUtils.rm_rf(@dir)
  end

  private

  def onboard(dir)
    m = new_mock
    Sims.start(build, flash: File.join(dir, "flash.bin"), erase: true, extra_args: sim_args,
                      name: "#{cache_name}-setup") do |s|
      s.wait(portal: true, timeout: 90)
      s.portal_connect("TRMNL-Sim", "password", server: m.device_url)
      m.wait_for_request("/api/display", timeout: 120)
      # a slow panel (Spectra 6) can still be refreshing when the chip sleeps
      s.wait(state: "deep_sleep", display_idle: true, timeout: 120)
      s.save_point(File.join(dir, "asleep.trmnlsave"), label: "onboarded, asleep")
    end
    { port: m.port }
  ensure
    m&.close
  end

  def start(**kw, &)
    extra = [*kw.delete(:extra_args), *sim_args]
    Sims.start(build, extra_args: extra, host_ports:, **kw, &)
  end
end
