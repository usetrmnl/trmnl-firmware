# frozen_string_literal: true

require "fileutils"

# Every simulator saves its log and final screen in out/artifacts/ when it closes, named after
# its example. A run replaces the previous run's: the first process removes the files that are
# older than itself, so parallel_rspec's other workers keep theirs.
module Artifacts
  DIR = File.join(Builds::OUT, "artifacts")
  STARTED = Time.now

  module_function

  def install
    TrmnlSim::Simulator.artifacts_dir = DIR
    return unless ParallelTests.first_process?

    Dir[File.join(DIR, "*")].each { |f| FileUtils.rm_f(f) if File.mtime(f) < STARTED }
  rescue Errno::ENOENT
    nil # another worker's file, already gone
  end
end
