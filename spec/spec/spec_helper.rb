# frozen_string_literal: true

# Integration tests: real firmware builds in the simulator against mock servers.
#
#   bundle exec rspec                      # the devices ENVS lists (see spec/support/selection.rb)
#   bundle exec parallel_rspec             # the same, a process per CPU (rake spec)
#   ENVS=trmnl:full bundle exec rspec spec/general/setup/portal_spec.rb:42

$LOAD_PATH.unshift(File.expand_path("../lib", __dir__))
require "parallel_tests"
require "trmnl_sim"

%w[devices builds selection setup_cache sims provisioned_device trmnl_x integration general screen
   flash golden matchers metadata fixtures].each { |f| require_relative "support/#{f}" }
Dir[File.join(__dir__, "support/*.rb")].each { |f| require f }

RSpec.configure do |config|
  config.extend Integration::DeviceContext
  config.include Integration::Helpers
  config.include Matchers
  config.extend Fixtures
  Metadata.install(config)

  config.before(:suite) do
    CoverageReport.install
    Artifacts.install
    unless File.executable?(TrmnlSim.binary)
      abort "no simulator at #{TrmnlSim.binary}: build it (rake sim:build, or cargo build --release) or set SIM_BIN"
    end
    missing = Selection.missing
    if missing.any?
      message = "ENVS lists devices whose build is missing; their examples are left out: " \
                "#{missing.map { Builds.missing(_1.env) }.join('; ')}"
      abort message if ENV["CI"]
      warn message if ParallelTests.first_process?
    end
  end

  config.after(:suite) do
    if ParallelTests.first_process?
      ParallelTests.wait_for_other_processes_to_finish
      CoverageReport.print
    end
  end

  config.expect_with :rspec do |c|
    c.include_chain_clauses_in_custom_matcher_descriptions = true
    c.max_formatted_output_length = 2000
  end
  config.mock_with(:rspec) { |m| m.verify_partial_doubles = true }
  config.shared_context_metadata_behavior = :apply_to_host_groups
  config.disable_monkey_patching!
  config.warnings = false
  config.order = :defined
  config.filter_run_when_matching :focus
end
