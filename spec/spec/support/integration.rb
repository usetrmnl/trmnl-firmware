# frozen_string_literal: true

# What example groups know about their device, and helpers every example group gets (see
# spec_helper.rb).
module Integration
  # A group's device: the one its `env:` (or its parent's) names. `device` and `build` in group
  # bodies (every group is extended with this).
  module DeviceContext
    def device = Devices.fetch(metadata.fetch(:env) { raise ArgumentError, "#{metadata[:location]}: no env:" })
    def build = Builds.for_env(device.env)
  end

  # Helpers for examples (and `before(:context)` hooks). `device` and `build` are the group's.
  module Helpers
    I = TrmnlSim::Images

    def device = self.class.device
    def build = self.class.build

    # A simulator (see Sims.start); with a block, closed after it.
    def sim(build = self.build, **, &) = Sims.start(build, **, &)

    # A factory-fresh device, offline, with its setup portal up (and then `settled`, e.g. the
    # screen drawn: display_idle:, min_refreshes:). `sim_kw` go to `sim` (networks:...). With a
    # block: closed after it.
    def fresh_device(settled: {}, **sim_kw, &block)
      s = sim(erase: true, extra_args: ["--offline"], **sim_kw)
      begin
        s.wait(portal: true, timeout: 90)
        s.wait(**settled, timeout: 60) if settled.any?
      rescue Exception # rubocop:disable Lint/RescueException -- don't leave the simulator behind
        s.close
        raise
      end
      block ? s.session(&block) : s
    end

    # No task in a memcheck report's `stacks` came close to overflowing its stack (but those whose
    # names start with one of `but`).
    def expect_no_low_stacks(stacks, but: [])
      low = stacks.select { |t| t["low"] && !t["task"].start_with?(*but) }.map { |t| [t["task"], t["min_free"]] }
      expect(low).to eq([]), "tasks close to overflowing their stacks: #{low}"
    end

    # Poll (every `every` s of wall-clock time) until the block is true, failing with `message`
    # if that takes more than `within` s.
    def eventually(message, within:, every: 0.5)
      deadline = Process.clock_gettime(Process::CLOCK_MONOTONIC) + within
      until yield
        expect(Process.clock_gettime(Process::CLOCK_MONOTONIC)).to be < deadline, message
        sleep every
      end
    end

    # ---- images for the group's device ---------------------------------------------------------------

    # Serve a black-and-white image the way the TRMNL server would for `device`: an 800x480
    # 1-bit BMP for the OG-size panels, else a PNG of the panel's size (a palette PNG for color
    # panels). `black.(x, y)` says which pixels are ink. Returns the path the device downloads
    # and the screenshot it should give.
    def device_image(mock, name, black, device: self.device)
      w, h = device.size
      return ["/images/#{name}.bmp", mock.set_image(name, black)] if device.default_bmp? && device.inks == "mono"

      color = ->(x, y) { black.(x, y) ? [0, 0, 0] : [255, 255, 255] }
      case device.inks
      when "bwry" then ["/images/#{name}.png", mock.set_color_png(name, color, w, h)]
      when "spectra6" then ["/images/#{name}.png", mock.set_spectra6_png(name, color, w, h)]
      else
        level = ->(x, y) { black.(x, y) ? 0 : 1 }
        mock.images["#{name}.png"] = I.png_image(level, w, h)
        mock.stamp(name)
        ["/images/#{name}.png", I.expected_gray(level, w, h)]
      end
    end

    # `big_number(text)` centred on the panel of `device` instead of on 800x480: pixel.(x, y)
    # is true for ink.
    def panel_number(text, scale: 24, device: self.device)
      w, h = device.size
      shift(I.big_number(text, scale:), w, h)
    end

    # `big_number(text)` drawn to fit and centred on the panel of `device`, for `device_image`.
    # On the OG's 800x480 it is exactly big_number(text).
    def device_number(text, device: self.device)
      w, h = device.size
      return I.big_number(text) if [w, h] == [800, 480]

      shift(I.big_number(text, scale: [4, [w, h].min / 20].max), w, h)
    end

    # A MockTrmnl whose default screen is what the TRMNL server would send `device`: the OG's
    # 800x480 BMP, else a 1-bit PNG of the panel's size (as ProvisionedDevice serves).
    # With a block: yields it and closes it afterwards.
    def device_mock(tls: false, device: self.device)
      mock = TrmnlSim::MockTrmnl.new(tls:)
      unless device.default_bmp?
        w, h = device.size
        number = panel_number("0", device:)
        mock.images["default.png"] = I.png_image(->(x, y) { number.(x, y) ? 0 : 1 }, w, h)
      end
      return mock unless block_given?

      begin
        yield mock
      ensure
        mock.close
      end
    end

    # ---- the screen ----------------------------------------------------------------------------------

    # The lines of text on screen from row `top` down: [first row, last row, leftmost ink column,
    # rightmost ink column] for each run of rows with ink, split at blank rows.
    def text_lines(sim, top)
      lines = []
      cur = nil
      Screen.gray_rows(sim.screenshot).each_with_index.drop(top).each do |row, y|
        first = row.index { _1 < 128 }
        if first.nil?
          lines << cur if cur
          cur = nil
          next
        end
        last = row.rindex { _1 < 128 }
        cur = cur ? [cur[0], y, [cur[2], first].min, [cur[3], last].max] : [y, y, first, last]
      end
      cur ? lines << cur : lines
    end

    private

    # big_number centres on 800x480; shift it onto a w x h panel's centre.
    def shift(pixel, w, h)
      dx = (800 - w) / 2
      dy = (480 - h) / 2
      ->(x, y) { pixel.(x + dx, y + dy) }
    end
  end
end
