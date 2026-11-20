# frozen_string_literal: true

require "fileutils"

# Every run records firmware coverage: each simulator writes an lcov tracefile into this run's
# directory under tmp/coverage/, one subdirectory per build. When the run is over (all of
# parallel_rspec's processes), the firmware's own code is merged into out/cov/merged.info and
# out/cov/html/ (TrmnlSim::Lcov) and its coverage printed. Third-party code (the framework,
# .pio/libdeps, vendored drivers) is left out: the tests aren't meant to cover it.
#
# The setup flows run once and are cached (SetupCache): their tracefiles are kept in the cache
# entry and copied into every run that uses it, so a cached run reports the same lines.
module CoverageReport
  DIR = File.join(Builds::OUT, "cov")
  # This run's tracefiles: parallel_rspec's workers share its pid file, a plain rspec is alone.
  RUN = File.join(Builds::HERE, "tmp/coverage",
                  ENV["PARALLEL_PID_FILE"] ? File.basename(ENV["PARALLEL_PID_FILE"]) : "pid-#{Process.pid}")
  # The firmware's own sources, in the tracefiles' paths (relative to its checkout). lib/ also
  # holds vendored drivers (BMA530_SensorAPI from Bosch, BQ27427 from SparkFun, IQS323 from
  # Azoteq), so only TRMNL's libraries are listed.
  FIRMWARE_SOURCES = %w[src/ lib/trmnl/ lib/trmnl_x/ lib/wificaptive/].freeze

  module_function

  def install
    TrmnlSim::Simulator.coverage_dir = RUN
    TrmnlSim::Simulator.coverage_root = Builds::FIRMWARE
  end

  # Simulators started in the block write their tracefiles into `dir` (a setup cache entry's).
  def recording_into(dir)
    before = TrmnlSim::Simulator.coverage_dir
    TrmnlSim::Simulator.coverage_dir = dir
    TrmnlSim::Simulator.coverage_root = Builds::FIRMWARE
    yield
  ensure
    TrmnlSim::Simulator.coverage_dir = before
  end

  # Copy a setup cache entry's tracefiles into this run. Their names are fixed, so workers using
  # the same entry write the same files and the setup is counted once.
  def adopt(entry, dir)
    Dir[File.join(dir, "*/*.info")].each do |f|
      out = File.join(RUN, File.basename(File.dirname(f)), "setup-#{entry}-#{File.basename(f)}")
      FileUtils.mkdir_p(File.dirname(out))
      FileUtils.cp(f, out)
    end
  end

  def merged = File.join(DIR, TrmnlSim::Lcov::MERGED)

  # Merge this run's tracefiles into the report (at the end of a run).
  def print
    return unless Dir.exist?(RUN)

    write(RUN)
    builds = Dir[File.join(RUN, "*/")]
    if builds.size > 1
      warn "\nPer build (the lines of the firmware's own code each build compiles):"
      builds.sort.each do |b|
        warn "  #{File.basename(b).ljust(32)} #{TrmnlSim::Lcov::Report.load([b], include: FIRMWARE_SOURCES).total_line}"
      end
    end
  ensure
    FileUtils.rm_rf(RUN)
  end

  # Rebuild the report from the last run's merged tracefile (`rake coverage`).
  def reprint
    abort "no coverage report at #{merged}: run the specs first" unless File.exist?(merged)
    write(merged)
  end

  # Write `input` (a tracefile or a directory of them) as cov/merged.info and cov/html/, and
  # print the summary.
  def write(input)
    report = TrmnlSim::Lcov::Report.load([input], include: FIRMWARE_SOURCES)
    FileUtils.rm_rf(File.join(DIR, "html"))
    FileUtils.mkdir_p(DIR)
    report.write_lcov(merged)
    # Relative paths in the tracefiles are relative to the firmware checkout.
    report.write_html(File.join(DIR, "html"), root: Builds::FIRMWARE)
    warn "\nFirmware coverage (#{FIRMWARE_SOURCES.join(', ')}); full report in #{File.join(DIR, 'html')}"
    puts report.summary, "(#{report.tracefiles} tracefiles)"
  end
end
