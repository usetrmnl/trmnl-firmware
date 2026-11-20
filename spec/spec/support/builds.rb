# frozen_string_literal: true

# Where the firmware builds are, and the settings every simulator of a run shares.
module Builds
  HERE = File.expand_path("../..", __dir__) # the suite's root (spec/ in the firmware)
  # What a run leaves behind: coverage (CoverageReport) and artifacts (Artifacts). Gitignored.
  OUT = File.join(HERE, "out")
  # The firmware checkout (FIRMWARE_REPO, else the one this suite lives in).
  FIRMWARE = File.expand_path(ENV.fetch("FIRMWARE_REPO", File.join(HERE, "..")))
  # PlatformIO build directories of the firmware checkout, one per environment.
  DIR = File.join(FIRMWARE, ".pio/build")

  TEST_MAC = "7C:DF:A1:00:00:01"
  # Turbo (network-aware fast-forward) unless REALTIME=1.
  TURBO = ENV["REALTIME"] != "1"
  # SLOW=1: also run the examples marked `slow:`.
  SLOW = ENV["SLOW"] == "1"
  # Every simulator runs with --memcheck=halt: a test fails on any memory error (the simulator
  # halts at it, or leaving its block raises). An example can ask for another mode (`memcheck:`).
  MEMCHECK = "halt"
  UPDATE_GOLDEN = ENV["UPDATE_GOLDEN"] == "1"

  module_function

  # The build of PlatformIO environment `env` (it may not exist: see `built?`).
  def for_env(env) = File.expand_path(File.join(DIR, env.to_s))

  # The simulator runs the merged image and its ELF.
  def built?(env) = %w[merged_firmware.bin merged_firmware.elf].all? { File.exist?(File.join(for_env(env), _1)) }

  # Why `env`'s tests can't run: its build is missing (nil: they can).
  def missing(env)
    "no #{env} merged_firmware.bin + .elf at #{for_env(env)} (pio run -e #{env})" unless built?(env)
  end

  # The newest release tag below this firmware's version: the stable release the fleet updates from
  # (nil: no tags).
  def previous_release = releases_below.last

  # The release before that: the one most TRMNL X are unboxed on in shipment mode.
  def factory_release = releases_below[-2]

  # Release tags below this firmware's version, oldest first. A shallow checkout (CI) has no tags:
  # then origin's.
  def releases_below
    @releases_below ||= begin
      current = Gem::Version.new(fw_version)
      tags = `git -C #{FIRMWARE} tag -l "v*"`.split
      tags = `git -C #{FIRMWARE} ls-remote --tags --refs origin "v*"`.scan(%r{refs/tags/(\S+)}).flatten if tags.empty?
      tags.grep(/\Av\d+\.\d+\.\d+\z/).select { Gem::Version.new(_1[1..]) < current }.sort_by { Gem::Version.new(_1[1..]) }
    end
  end

  # The TRMNL_X build of release `tag` (rake "firmware:build_release[<tag>]").
  def release(tag) = File.join(HERE, "tmp/releases", tag, "TRMNL_X")

  # Why the tests that boot release `tag` can't run (nil: they can).
  def release_missing(tag)
    return "not enough release tags below #{fw_version} in #{FIRMWARE}" if tag.nil?
    return if %w[merged_firmware.bin merged_firmware.elf].all? { File.exist?(File.join(release(tag), _1)) }

    "no #{tag} build at #{release(tag)} (rake \"firmware:build_release[#{tag}]\")"
  end

  # The environment of one of these builds (`for_env`).
  def env_of(build) = File.basename(File.expand_path(build.to_s))

  # The device a build directory is for (nil: unknown).
  def device_of(build) = Devices::BY_ENV[env_of(build)]

  # The firmware's version ("1.8.19"), as FW_VERSION_STRING in include/config.h makes it.
  def fw_version
    config = File.read(File.join(FIRMWARE, "include/config.h"))
    %w[MAJOR MINOR PATCH].map do |part|
      config[/^#define FW_#{part}_VERSION\s+(\d+)/, 1] or raise "no FW_#{part}_VERSION"
    end.join(".")
  end
end
