# frozen_string_literal: true

require "fileutils"
require "json"
require "net/http"
require "tmpdir"
require "uri"

module TrmnlSim
  # Launches trmnl-sim (headless unless `gui: true`) and drives it over its control API.
  #
  #   env:        the PlatformIO environment the firmware was built with (picks the board).
  #   firmware:   the merged flash image (merged_firmware.bin), with its ELF at the same path with
  #               .elf; or a PlatformIO build dir, for its merged_firmware.bin (see
  #               TrmnlSim.merged_image).
  #   flash:      flash image path; defaults to a fresh temp file (deleted on close).
  #   erase:      start from erased flash (factory reset). Implied for a new temp flash.
  #   mac:        device MAC, e.g. "D8:3B:DA:F5:28:3C" (the server identity).
  #   turbo:      run unthrottled (virtual time faster than wall time when idle).
  #   gui:        show the window too.
  #   binary:     the trmnl-sim executable (default: $SIM_BIN or target/release/trmnl-sim).
  #   extra_args: more command line arguments.
  #   faults:     faults injected from the start (as for `set_faults`), e.g.
  #               { power_loss: { partition: "nvs" } }.
  #   networks:   the access points in range (as for `set_networks`).
  #   memcheck:   run with the memory checker: "halt" (stop at the first violation) or "log"
  #               (report and carry on). Leaving an `open` block then fails if there were
  #               violations; `memcheck` returns the full report.
  #   memcheck_suppress: functions whose (known) memory bugs to tolerate: a violation with one
  #               of them in its backtrace, allocation or free stack is only counted.
  #   name:       label for artifacts. If `Simulator.artifacts_dir` is set, the log and final
  #               screen of every simulator are saved there on close.
  #   restore:    start from this save point file (see `save_point`) instead of booting.
  #   host_ports: { guest port => host port }; the device's connections to 10.0.2.2:<guest port>
  #               go to the host port instead (`--host-port`), e.g. for a device onboarded
  #               against a server that has since moved to another port.
  #   coverage:   record firmware code coverage and write an lcov tracefile here when the
  #               simulator exits (`--coverage`). If `Simulator.coverage_dir` is set, every
  #               simulator writes one there (`<build>/<name>-*.info`); merge them with
  #               TrmnlSim::Lcov. Source paths are relative to `Simulator.coverage_root` if set
  #               (`--coverage-root`), else absolute.
  class Simulator
    class << self
      # Where every simulator writes a coverage tracefile unless given `coverage:` (nil: none).
      attr_accessor :coverage_dir
      # The directory coverage source paths are written relative to (nil: absolute paths).
      attr_accessor :coverage_root
      # Where every simulator saves its log and final screen on close (nil: nowhere).
      attr_accessor :artifacts_dir
    end

    # Console line index after the last matched line (where `wait(console:)` searches from).
    attr_accessor :cursor

    # A simulator for the duration of the block: closed afterwards, and if it ran with
    # memcheck and the block succeeded, checked for memory errors first.
    def self.open(*, **kw, &block)
      sim = new(*, **kw)
      block ? sim.session(&block) : sim
    end

    attr_reader :env, :firmware, :build_dir, :flash, :log_path, :name, :coverage_path, :base, :pid, :memcheck_mode

    def initialize(env, firmware, flash: nil, erase: false, mac: nil, turbo: false, gui: false, binary: nil,
                   extra_args: [], faults: nil, networks: nil, startup_timeout: 30, name: nil, restore: nil,
                   host_ports: {}, coverage: nil, memcheck: nil, memcheck_suppress: [])
      @env = env.to_s
      @firmware = TrmnlSim.merged_image(firmware)
      @build_dir = File.dirname(@firmware)
      @tmpdir = Dir.mktmpdir("trmnl-sim-")
      @flash = flash ? flash.to_s : File.join(@tmpdir, "flash.bin")
      @log_path = File.join(@tmpdir, "sim.log")
      @cursor = 0
      @name = name || "sim"
      @memcheck_mode = memcheck
      binary = (binary || TrmnlSim.binary).to_s
      raise Error, "#{binary} not found; run `cargo build --release` in #{REPO}" unless File.exist?(binary)

      args = [binary, @firmware, "--env", @env, "--control", "127.0.0.1:0", "--portal-port", "0", "--flash", @flash]
      args << "--headless" unless gui
      args << "--erase" if erase || !flash
      args += ["--mac", mac] if mac
      args << "--turbo" if turbo
      args += ["--restore", File.expand_path(restore.to_s)] if restore
      host_ports.each { |guest, host| args += ["--host-port", "#{guest}=#{host}"] }
      @coverage_path = (coverage || default_coverage_path)&.to_s
      args += ["--coverage", @coverage_path] if @coverage_path
      args += ["--coverage-root", self.class.coverage_root.to_s] if @coverage_path && self.class.coverage_root
      args += ["--faults", JSON.generate(faults)] if faults
      args += ["--wifi-networks", JSON.generate(networks)] if networks
      if memcheck
        args << "--memcheck=#{memcheck}"
        args += ["--memcheck-suppress", memcheck_suppress.join(",")] if memcheck_suppress.any?
      end
      args += extra_args.map(&:to_s)
      @pid = Process.spawn(*args, out: @log_path, err: %i[child out], pgroup: true)
      begin
        @base = discover_url(startup_timeout)
      rescue Exception # rubocop:disable Lint/RescueException -- don't leave the process behind
        kill
        raise
      end
    end

    # ---- lifecycle ------------------------------------------------------------------------------------

    # Yield self, then close; if it runs with memcheck and the block succeeded, check for memory
    # errors first. Returns the block's value.
    def session
      result = yield self
      assert_no_memory_errors if memcheck_mode && running?
      result
    ensure
      close
    end

    # Everything the simulator printed (serial output, [sim] messages).
    def log = File.exist?(log_path) ? File.binread(log_path).scrub("?") : ""

    def running?
      return false if @exit_status

      done, status = Process.waitpid2(@pid, Process::WNOHANG)
      @exit_status = status if done
      !done
    end

    def close
      return if @closed

      @closed = true
      save_artifacts
      if running?
        begin
          post("/quit")
          # Writing the coverage report reads the ELF's line tables first.
          exited_within?(coverage_path ? 60 : 10) or kill
        rescue StandardError
          kill
        end
      end
      FileUtils.rm_rf(@tmpdir)
    end

    # ---- coverage ------------------------------------------------------------------------------------

    # Write the firmware code coverage so far as an lcov tracefile (default: the `coverage`
    # path) and return its totals (lines_found, lines_hit, functions_found, functions_hit, files,
    # path). `reset` starts over afterwards. Needs `coverage:` (or `Simulator.coverage_dir`).
    def write_coverage(path = nil, reset: false)
      body = { reset: }
      body[:path] = File.expand_path(path.to_s) if path
      post("/coverage", body, timeout: 120)
    end

    # ---- UI actions ----------------------------------------------------------------------------------

    def status = get_json("/status")

    # Hold (true) or release (false) the physical button.
    def button(down)
      post("/button", { down: })
    end

    # Press and hold the button for `ms` of virtual time, then release.
    def press(ms = 100)
      post("/press", { ms: }, timeout: (ms / 1000.0 * 20) + 30)
    end

    # Two presses of `ms`, `gap_ms` apart, timed in virtual time.
    def double_click(ms: 80, gap_ms: 150)
      post("/press", { ms:, count: 2, gap_ms: }, timeout: ((ms + gap_ms) * 2 / 1000.0 * 20) + 30)
    end

    # Tap the touch bar ("left", "center" or "right") for `ms` of virtual time (TRMNL X).
    def touch(zone, ms: 120)
      post("/touch", { zone:, ms: }, timeout: (ms / 1000.0 * 20) + 30)
    end

    # Put a finger on a touch bar zone and keep it there (several may be down).
    def touch_down(zone)
      post("/touch", { zone:, down: true })
    end

    def touch_up(zone)
      post("/touch", { zone:, down: false })
    end

    # A slide along the touch bar: "swipe_next", "swipe_back", "flick_next" or "flick_back"
    # (only reported in slide mode).
    def gesture(name)
      post("/gesture", { gesture: name })
    end

    # Put the device on / take it off its magnetic dock (TRMNL X). Returns once applied.
    def dock(docked = true)
      post("/dock", { docked: })
      poll_until(10, "dock state did not change (is the simulator paused?)") { status["docked"] == docked }
    end

    # Dump CPU state and board diagnostics; returns the dumped lines.
    def debug
      since = status["console_total"]
      post("/debug")
      sleep 0.3
      console(since)
    end

    def reset = post("/reset")
    def power_cycle = post("/power-cycle")

    # End a deep sleep now, as if its timer expired.
    def wake = post("/wake")

    def set_wifi(available)
      post("/wifi", { available: })
    end

    # Replace the access points in range: hashes with ssid and optionally password (nil: any),
    # rssi, channel, open, internet (see --wifi-networks).
    def set_networks(networks)
      post("/wifi", { networks: })
    end

    # Whether the host's portal client joins the setup access point (the default). It keeps the
    # simulation at wall-clock pace, even in turbo mode; without it an unattended portal runs
    # ahead (e.g. to its 15-minute timeout).
    def set_portal_client(on)
      post("/wifi", { portal_client: on })
    end

    def set_battery(mv)
      post("/battery", { mv: })
    end

    def set_turbo(on = true)
      post("/turbo", { on: })
    end

    # Pause (or resume) the simulation; with a block, paused for the block's duration, so
    # what it does happens at the same instant of virtual time.
    def pause(on = true)
      post("/pause", { on: })
      return unless block_given?

      begin
        yield
      ensure
        post("/pause", { on: false })
      end
    end

    # ---- save points ----------------------------------------------------------------------------------

    # Take a save point: in deep sleep the full device state, otherwise only what survives a
    # battery pull (flash, screen). Kept in memory (see `save_points`) and, with `path`, written
    # to that file for `restore` or `Simulator.new(restore:)`. Returns its info (id, label,
    # deep_sleep, sim_time_s, wake_at_s, path, bytes).
    def save_point(path = nil, label: nil)
      body = {}
      body[:path] = File.expand_path(path.to_s) if path
      body[:label] = label if label
      post("/savepoint", body, timeout: 150)["savepoint"]
    end

    # Replace the device with a save point from a file or an in-memory slot `id`.
    def restore(path = nil, id: nil)
      body = path ? { path: File.expand_path(path.to_s) } : { id: }
      post("/restore", body, timeout: 150)["savepoint"]
    end

    # The in-memory save points, oldest first.
    def save_points = get_json("/savepoints")["savepoints"]

    # ---- preferences ----------------------------------------------------------------------------------

    # The firmware's NVS preferences in `namespace` as read from flash, as {key => value}
    # (strings as-is, integers as decimal strings, blobs as hex).
    def preferences(namespace = "data")
      get_json("/preferences")["entries"].select { _1["namespace"] == namespace }.to_h { [_1["key"], _1["value"]] }
    end

    # ---- faults ---------------------------------------------------------------------------------------

    # Current faults: faults, summary, power_losses, flash (programs, erases),
    # partitions ([{label, type, subtype, offset, size}]).
    def faults = get_json("/faults")

    # Merge faults into the current ones (keys left out are kept, nil clears one):
    #
    #   net: {latency_ms, loss (0..1), bandwidth_bps, dns ("servfail" | "nxdomain" | "empty" |
    #         "timeout"), no_internet, offline, tcp_cut: {after_bytes, stall, port}}
    #   power_loss: {op ("any" | "program" | "erase"), partition ("nvs", "otadata", "ota_0",
    #                "spiffs", ...), range: [start, end], nth, cut ("before" | "torn" | "after")}
    #   i2c_absent: [0x55, ...]
    #   panel_busy_stuck: bool
    #   modem_unresponsive: bool
    #   modem_at_errors: ["AT+CWMODE", ...] (TRMNL X: answer ERROR to these commands)
    #   touch_bar: "reset" | "lockup" | "ati_error" (TRMNL X IQS323)
    #   gauge_reset: true (TRMNL X BQ27427 power-on reset, once, when set)
    #   chip_temp_c: die temperature the ESP32-C3's sensor reads (default 25)
    def set_faults(faults = {}, **kw)
      post("/faults", faults.merge(kw))
    end

    # Shortcut for set_faults(net: {...}), e.g. set_net_faults(latency_ms: 300, loss: 0.1).
    def set_net_faults(**kw)
      set_faults(net: kw)
    end

    # Cut power at the `nth` flash `op` into `partition` / `range` (one-shot). With cut: "torn"
    # the interrupted program/erase is left half done.
    def arm_power_loss(partition = nil, op: "any", nth: 1, cut: "before", range: nil)
      spec = { op:, nth:, cut: }
      spec[:partition] = partition if partition
      spec[:range] = range.to_a if range
      set_faults(power_loss: spec)
    end

    def clear_faults = delete("/faults")

    # ---- observation ---------------------------------------------------------------------------------

    # The console lines from index `since` on.
    def console(since = 0) = get_json("/console?since=#{since}")["lines"].map { |l| l["text"] }

    # Block until all conditions hold: console (a Regexp or regex string, searched from `since`,
    # default `cursor`), state ("running", "deep_sleep", "light_sleep", "halted", ...),
    # min_refreshes, min_boots, display_idle, wifi_connected, portal (captive portal up),
    # settle_ms. Returns the answer: {"status" => ..., "line" => {"i", "text"}}.
    def wait(timeout: 60, **cond)
      if cond.key?(:console)
        cond[:console] = cond[:console].source if cond[:console].is_a?(Regexp)
        cond[:since] ||= cursor
      end
      code, _, data = request(:post, "/wait", body: cond.merge(timeout_s: timeout), timeout: timeout + 30)
      out = JSON.parse(data)
      unless code == 200
        tail = log.lines.last(40).join
        raise Error, "wait(#{cond}) failed: #{out['error']}\nstatus: #{out['status']}\n--- log tail ---\n#{tail}"
      end
      line = out["line"]
      self.cursor = line["i"] + 1 if line.is_a?(Hash)
      out
    end

    # The first console line from `cursor` on matching `regex`.
    def wait_for_console(regex, timeout: 60) = wait(console: regex, timeout:)["line"]["text"]

    # Wait for the device to be in deep sleep (and any other conditions); returns its status.
    def wait_for_deep_sleep(timeout: 90, **cond) = wait(state: "deep_sleep", timeout:, **cond)["status"]

    # Wait for the next completed display refresh (BUSY released).
    def wait_for_refresh(timeout: 90)
      wait(min_refreshes: status["display_refreshes"] + 1, display_idle: true, settle_ms: 300, timeout:)
    end

    # PNG of the e-paper (8-bit gray, 0 = ink, 255 = paper, or RGB on color panels);
    # optionally cropped to region [x, y, w, h] and written to `path`.
    def screenshot(path = nil, region: nil)
      code, _, data = request(:get, "/screenshot#{region_query(region, '?')}")
      raise Error, "screenshot failed: #{data[0, 200].inspect}" unless code == 200

      if path
        FileUtils.mkdir_p(File.dirname(path.to_s))
        File.binwrite(path.to_s, data)
      end
      data
    end

    # Compare the screen (or a region [x, y, w, h] of it) to a reference PNG (bytes or a path).
    # Returns match / diff statistics: {"match", "diff_pixels", "diff_ratio", ...}.
    def compare_screen(reference, region: nil, tolerance: 48, max_ratio: 0.001)
      png = reference.is_a?(String) && reference.start_with?(Images::PNG_SIGNATURE)
      ref = png ? reference : File.binread(reference.to_s)
      query = "?tolerance=#{tolerance}&max_ratio=#{max_ratio}#{region_query(region, '&')}"
      code, _, data = request(:post, "/screenshot/compare#{query}", raw: ref)
      out = JSON.parse(data)
      raise Error, "compare failed: #{out['error']}" unless code == 200

      out
    end

    # ---- memory checking ------------------------------------------------------------------------------

    # The --memcheck report: violations (each with kind, address, backtrace, report lines, and for
    # heap errors the block's allocation and free stacks), suppressed (the same, for violations
    # matching --memcheck-suppress), heap statistics (live and peak bytes per memory) and stacks
    # (per-task high-water marks, low if within stack_margin bytes of overflowing).
    # {"enabled" => false} without --memcheck.
    def memcheck = get_json("/memcheck")

    # Fail with the full reports if memcheck found violations.
    def assert_no_memory_errors
      violations = memcheck.fetch("violations", [])
      return if violations.empty?

      text = violations.flat_map { |v| v["report"] }.join("\n")
      raise MemoryErrors, "memcheck found #{violations.size} violation(s):\n#{text}"
    end

    class MemoryErrors < Error; end

    # ---- captive portal --------------------------------------------------------------------------------

    def portal_url(timeout: 60) = wait(portal: true, timeout:)["status"]["portal_url"].chomp("/")

    CONNECTION_ERRORS = [SystemCallError, IOError, EOFError, Net::OpenTimeout, Net::ReadTimeout].freeze

    # HTTP request to the device's own web server (while in setup mode): a POST of JSON `body`,
    # else a GET. Connection failures are retried for `retry_for` seconds: the device may still
    # be (re)starting its AP. Returns [status code, body].
    def portal_request(path, body = nil, timeout: 30, retry_for: 30)
      deadline = monotonic + retry_for
      begin
        code, _, data = http(portal_url + path, body ? :post : :get, body: body && JSON.generate(body),
                                                                     json: !body.nil?, timeout:)
        [code, data]
      rescue *CONNECTION_ERRORS
        raise if monotonic > deadline

        sleep 0.5
        retry
      end
    end

    # The setup page's network list; like the page, asks again while the device answers 202
    # (its first scan is still running).
    def portal_scan(timeout: 30)
      deadline = monotonic + timeout
      loop do
        code, data = portal_request("/scan")
        next sleep(0.5) if code == 202 && monotonic <= deadline
        raise Error, "/scan -> #{code}" unless code == 200

        return JSON.parse(data)
      end
    end

    # Submit WiFi credentials through the setup page, like a phone would.
    def portal_connect(ssid, password = "", server: "https://trmnl.app")
      code, data = portal_request("/connect", { ssid:, pswd: password, server: })
      raise Error, "/connect -> #{code}: #{data[0, 200].inspect}" unless code == 200

      JSON.parse(data)
    end

    private

    # ---- HTTP ------------------------------------------------------------------------------------------

    def request(method, path, body: nil, raw: nil, timeout: 30)
      data = raw || (body && JSON.generate(body))
      http(base + path, method, body: data, json: !body.nil?, timeout:)
    end

    def post(path, body = {}, timeout: 30)
      checked(:post, path, request(:post, path, body:, timeout:))
    end

    def delete(path) = checked(:delete, path, request(:delete, path))

    def get_json(path)
      code, _, data = request(:get, path)
      raise Error, "GET #{path} -> #{code}: #{data[0, 200].inspect}" unless code == 200

      JSON.parse(data)
    end

    def checked(method, path, response)
      code, _, data = response
      out = data.to_s.empty? ? {} : JSON.parse(data)
      unless code == 200
        raise Error,
              "#{method.upcase} #{path} -> #{code}: #{out.is_a?(Hash) ? out.fetch('error', out) : out}"
      end

      out
    end

    def http(url, method, body: nil, json: false, timeout: 30)
      uri = URI(url)
      req = Net::HTTPGenericRequest.new(method.to_s.upcase, !body.nil?, true, uri.request_uri)
      if body
        req.body = body
        req["Content-Type"] = json ? "application/json" : "application/octet-stream"
      end
      Net::HTTP.start(uri.host, uri.port, open_timeout: timeout, read_timeout: timeout,
                                          write_timeout: timeout) do |conn|
        res = conn.request(req)
        [res.code.to_i, res["Content-Type"].to_s, res.body.to_s.b]
      end
    end

    def region_query(region, sep)
      return "" unless region

      x, y, w, h = region
      "#{sep}x=#{x}&y=#{y}&w=#{w}&h=#{h}"
    end

    def discover_url(timeout)
      deadline = monotonic + timeout
      while monotonic < deadline
        raise Error, "simulator exited early:\n#{log}" unless running?

        url = log[%r{control API on (http://[\d.:]+)/}, 1]
        return url if url

        sleep 0.05
      end
      raise Error, "simulator did not start its control API"
    end

    def default_coverage_path
      dir = self.class.coverage_dir
      return unless dir

      # one directory per build (named after its environment), for per-device reports
      build_cov = File.join(dir, File.basename(build_dir))
      FileUtils.mkdir_p(build_cov)
      path = nil
      path = File.join(build_cov, "#{safe_name}-#{rand(36**8).to_s(36)}.info") while path.nil? || File.exist?(path)
      FileUtils.touch(path)
      path
    end

    def save_artifacts
      out = self.class.artifacts_dir
      return unless out

      FileUtils.mkdir_p(out)
      n = 0
      n += 1 while File.exist?(File.join(out, "#{safe_name}-#{n}.log"))
      begin
        screenshot(File.join(out, "#{safe_name}-#{n}.png")) if running?
      rescue StandardError
        nil
      end
      FileUtils.cp(log_path, File.join(out, "#{safe_name}-#{n}.log")) if File.exist?(log_path)
    end

    def safe_name = name.gsub(/[^\w.-]+/, "_")

    def exited_within?(seconds)
      deadline = monotonic + seconds
      while monotonic < deadline
        return true unless running?

        sleep 0.05
      end
      false
    end

    def kill
      Process.kill("KILL", @pid)
      Process.wait(@pid)
    rescue Errno::ESRCH, Errno::ECHILD
      nil
    ensure
      @exit_status = true if @exit_status.nil?
    end

    def poll_until(seconds, message)
      deadline = monotonic + seconds
      until yield
        raise Error, message if monotonic > deadline

        sleep 0.02
      end
    end

    def monotonic = Process.clock_gettime(Process::CLOCK_MONOTONIC)
  end
end
