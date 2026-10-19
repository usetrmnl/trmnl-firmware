# frozen_string_literal: true

require "fileutils"
require "tempfile"
require "tmpdir"

# Shared fixtures for the TRMNL X tests (ESP32-S3, 1872x1404 panel, C5 modem), and what makes
# the general tests (General.describe) run on it (see XSim and TrmnlX.unboxed).
module TrmnlX
  MAC = "D8:3B:DA:00:00:01"
  SSID_24 = "TRMNL-Sim"
  SSID_5 = "TRMNL-Sim-5G"

  module_function

  def build = Builds.for_env("TRMNL_X")

  # big_number(text) scaled up to the X's 1872x1404 panel, as a 1-bit level (0 = black).
  def digits(text)
    pixel = TrmnlSim::Images.big_number(text)
    ->(x, y) { pixel.(x * 800 / 1872, y * 480 / 1404) ? 0 : 1 }
  end

  # A simulated X (offline, with the X's MAC). With a block: yields it and closes it.
  def sim(**kw, &block)
    kw = { name: Sims.current_name, mac: MAC, turbo: Builds::TURBO, memcheck: Builds::MEMCHECK }.merge(kw)
    extra = kw.delete(:extra_args).to_a
    extra += ["--offline"] unless extra.include?("--offline")
    s = TrmnlSim::Simulator.new(Builds.env_of(build), build, extra_args: extra, **kw)
    block ? s.session(&block) : s
  end

  # Take a shipped device off shipment mode and through the setup portal.
  def onboard(s, mock, ssid)
    s.wait_for_console(/Entering shipment mode light sleep loop/, timeout: 180)
    s.dock(true)
    s.wait(portal: true, timeout: 180)
    s.dock(false)
    s.portal_connect(ssid, "password", server: mock.device_url)
    mock.wait_for_request("/api/display", timeout: 240)
    s.wait(state: "deep_sleep", timeout: 240)
    skip_touchbar_tutorial(s)
  end

  # Mark the touch bar tutorial (src/menu.cpp, run_tutorial) done on an X in deep sleep: until it is,
  # any touch shows it instead of doing what it does. touchbar_tutorial_spec.rb is about it.
  def skip_touchbar_tutorial(s) = s.set_preference("touch_tutorial", "u8", 1)

  # Wake the X and refresh, as a middle tap did before the touch menu: a middle tap opens the
  # menu (src/menu.cpp, menu_open) and a second one chooses its first item,
  # Next Screen, after which the wake goes on to fetch the next screen.
  def next_screen(s)
    c = s.status["console_total"]
    s.touch("center", ms: 150)
    s.wait(console: /menu_show_screen end/, since: c, timeout: 30)
    s.touch("center", ms: 150)
  end

  # Tap the touch bar in the touch menu, wait for the firmware to log `log`, then for the screen to
  # be redrawn: a new menu screen (`screen: true`) or the arrow's move. The firmware draws before it
  # reads the touch bar again, so a tap during a refresh is missed.
  def menu_tap(s, zone, log, screen: false)
    c = s.status["console_total"]
    s.touch(zone, ms: 150)
    s.wait(console: log, since: c, timeout: 15)
    if screen
      s.wait(console: /menu_show_screen end/, since: c, timeout: 15)
    else
      s.wait(display_idle: true, settle_ms: 100, timeout: 15)
    end
  end

  # Wake a sleeping X into the touch menu and go to Settings..., leaving it on the screen with the
  # arrow on its first item, Diagnostics...
  def open_settings_menu(s)
    c = s.status["console_total"]
    s.touch("center", ms: 150)
    s.wait(console: /menu_show_screen end/, since: c, timeout: 30)
    menu_tap(s, "right", /Next Screen -> Previous Screen/)
    menu_tap(s, "right", /Previous Screen -> Touchbar Tutorial/)
    menu_tap(s, "right", /Touchbar Tutorial -> Special/)
    menu_tap(s, "right", /Special\.\.\. -> Settings/)
    menu_tap(s, "center", /chose Settings/, screen: true)
  end

  # Open Settings..., then Reset Wi-Fi..., leaving its confirmation (Confirm Reset Wi-Fi, Back)
  # on the screen with the arrow on Back.
  def ask_to_reset_wifi(s)
    open_settings_menu(s)
    menu_tap(s, "right", /Diagnostics\.\.\. -> Reset Wi-Fi/)
    menu_tap(s, "center", /chose Reset Wi-Fi/, screen: true)
  end

  # The flash of an X a customer just unboxed: a shipped X (see ShippedX) taken out of shipment
  # mode by its dock; it restarted into the setup portal and remembers being shipped, so booting
  # this flash goes straight to the portal. The general tests' "fresh device" (erase: true) boots
  # a copy of it, as the X never gets to the portal from an erased flash without the factory flow.
  def unboxed
    shipped = ShippedX.new
    inputs = { shipped: File.basename(shipped.cache), memcheck: Builds::MEMCHECK, turbo: Builds::TURBO }
    cache, = SetupCache.entry("x-unboxed", inputs) do |dir|
      FileUtils.cp(shipped.template, File.join(dir, "flash.bin"))
      sim(flash: File.join(dir, "flash.bin"), name: "x-unbox") do |s|
        s.wait_for_console(/Entering shipment mode light sleep loop/, timeout: 180)
        s.dock(true)
        s.wait(portal: true, timeout: 180)
      end
      {}
    end
    File.join(cache, "flash.bin")
  ensure
    shipped&.close
  end

  # Sims.start for the X: an XSim; a factory-fresh device (`erase: true`, or no flash) boots a
  # copy of the unboxed X instead of an erased flash (TrmnlX.sim(erase: true) runs the factory
  # flow).
  def general_sim(build, erase: false, **kw)
    own = nil
    if erase || (kw[:flash].nil? && kw[:restore].nil?)
      unless kw[:flash]
        own = Tempfile.create(["trmnl-x-unboxed-", ".bin"]).tap(&:close).path
        kw[:flash] = own
      end
      FileUtils.cp(unboxed, kw[:flash])
    end
    XSim.new(Builds.env_of(build), build, own_flash: own, **kw)
  end

  # A simulated TRMNL X for the general tests: the button actions they use are done with the
  # touch bar, the X's equivalent in the firmware (bl.cpp, process_iqs323_data):
  #
  # - a short press (up to 1 s, which wakes the OG and refreshes) is a tap in the middle, which
  #   wakes the X (Update-Source: EXT0) into the touch menu, then a second one choosing Next
  #   Screen, which refreshes (TrmnlX.next_screen);
  # - a long press (5-15 s: forget WiFi, open the setup portal) is the X's WiFi reset: Settings,
  #   Reset Wi-Fi... and its confirmation in the touch menu (TrmnlX.ask_to_reset_wifi);
  # - the OG's double click or 1-5 s press (run the special function) and 15 s press (soft reset:
  #   forget the device too) aren't mapped: the X runs the saved special function from its touch
  #   menu, but nothing drives that here yet, and it has no soft reset. An example using them is
  #   skipped (give it `needs: :double_click` / `needs: :soft_reset_press`).
  class XSim < TrmnlSim::Simulator
    def initialize(*, own_flash: nil, **)
      super(*, **)
      @own_flash = own_flash
    end

    def press(ms = 100)
      skip_tutorial
      if ms >= 15_000
        skip!("the TRMNL X has no soft-reset gesture (the OG's 15 s press)")
      elsif ms > 5000
        reset_wifi_gesture
      elsif ms > 1000
        skip!("the TRMNL X has no gesture that runs the special function (the OG's 1-5 s press)")
      else
        TrmnlX.next_screen(self)
      end
    end

    def double_click(**)
      skip!("the TRMNL X has no double click (the OG's special function gesture)")
    end

    def button(down)
      down ? touch_down("center") : touch_up("center")
    end

    # Forget WiFi and open the setup portal from the touch menu. The X reads the touch bar when
    # it wakes, so it waits for the device to be asleep first.
    def reset_wifi_gesture
      wait(state: "deep_sleep", timeout: 120, settle_ms: 200) unless status["state"] == "deep_sleep"
      skip_tutorial
      TrmnlX.ask_to_reset_wifi(self)
      TrmnlX.menu_tap(self, "left", /Back -> Confirm Reset Wi-Fi/)
      touch("center", ms: 150)
    end

    def close
      super
      FileUtils.rm_f(@own_flash) if @own_flash
    end

    private

    # The general tests' touches are the OG's button presses, not a first touch of the touch bar:
    # mark its tutorial done (only while asleep, when the X reads its touches).
    def skip_tutorial
      return if status["state"] != "deep_sleep" || preferences["touch_tutorial"] == "1"

      TrmnlX.skip_touchbar_tutorial(self)
    end

    def skip!(why) = raise(RSpec::Core::Pending::SkipDeclaredInExample, why)
  end

  # A TRMNL X straight out of the factory: QA passed, modem flashed, waiting in shipment mode
  # (light sleep until it is docked). From the setup cache (see SetupCache).
  class ShippedX
    attr_reader :cache, :template, :factory_console

    def initialize
      inputs = { build: SetupCache.build_id(TrmnlX.build), memcheck: Builds::MEMCHECK, turbo: Builds::TURBO }
      @cache, meta = SetupCache.entry("x-shipped", inputs) do |dir|
        TrmnlX.sim(flash: File.join(dir, "flash.bin"), erase: true, name: "x-factory") do |s|
          s.wait_for_console(/Entering shipment mode light sleep loop/, timeout: 180)
          { console: s.console(0) }
        end
      end
      @template = File.join(cache, "flash.bin")
      @factory_console = meta.fetch("console")
      @dir = Dir.mktmpdir("trmnl-x-")
    end

    # A copy of the shipped flash to boot (or to change).
    def copy(name)
      flash = File.join(@dir, "#{name}-#{Dir.children(@dir).size}.bin")
      FileUtils.cp(template, flash)
      flash
    end

    def boot(**, &) = TrmnlX.sim(flash: copy("shipped"), **, &)

    def close = FileUtils.rm_rf(@dir)
  end

  # A mock server plus an X that completed onboarding (on `ssid`) against a server like it; see
  # ProvisionedDevice, which this mirrors (`boot`, `boot_asleep`, `restore`, `reset`).
  class ProvisionedX
    attr_reader :shipped, :ssid, :cache, :mock, :host_ports

    def initialize(shipped, ssid: SSID_5)
      @shipped = shipped
      @ssid = ssid
      inputs = { shipped: File.basename(shipped.cache), ssid:, memcheck: Builds::MEMCHECK, turbo: Builds::TURBO }
      @cache, meta = SetupCache.entry("x-provisioned-#{ssid}", inputs) { |dir| onboard(dir) }
      @mock = TrmnlSim::MockTrmnl.new
      mock.display = { image: "default", refresh_rate: 300 }
      @host_ports = { meta.fetch("port") => mock.port }
    end

    def reset(display: { image: "default", refresh_rate: 300 }) = mock.reset(display:)

    def boot(**, &)
      flash = shipped.copy("provisioned")
      FileUtils.cp(File.join(cache, "flash.bin"), flash)
      TrmnlX.sim(flash:, host_ports:, **, &)
    end

    def boot_asleep(**, &) = restore(File.join(cache, "asleep.trmnlsave"), **, &)

    def restore(path, **, &) = TrmnlX.sim(restore: path, host_ports:, **, &)

    # A save point of this X asleep after the block ran on it (booted from the onboarded flash,
    # served by `mock`), for `restore`. Cached like the onboarding: `inputs` names everything
    # the block depends on (e.g. the hash of the file it is in).
    def cached_state(name, inputs = {})
      dir, = SetupCache.entry("x-state-#{name}", { provisioned: File.basename(cache), **inputs }) do |d|
        boot do |s|
          yield s
          s.save_point(File.join(d, "state.trmnlsave"), label: name)
        end
        {}
      end
      File.join(dir, "state.trmnlsave")
    end

    def close = mock.close

    private

    def onboard(dir)
      FileUtils.cp(shipped.template, File.join(dir, "flash.bin"))
      TrmnlSim::MockTrmnl.open do |m|
        m.display = { image: "default", refresh_rate: 300 }
        TrmnlX.sim(flash: File.join(dir, "flash.bin"), name: "x-provision") do |s|
          TrmnlX.onboard(s, m, ssid)
          s.save_point(File.join(dir, "asleep.trmnlsave"), label: "onboarded, asleep")
        end
        { port: m.port }
      end
    end
  end
end
