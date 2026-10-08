# frozen_string_literal: true

# Starting simulators the way the tests need them.
module Sims
  module_function

  # The name of the running example (or group), for artifacts.
  def current_name
    RSpec.current_example&.full_description || RSpec.current_scope.to_s
  end

  # A simulator of `build`. A device that doesn't get from an
  # erased flash to its setup portal on its own (the X: factory flow, shipment mode, dock) boots
  # its "unboxed" state for `erase: true` instead, and a device without a button maps presses to
  # its equivalent (see TrmnlX::XSim). With a block: yields it and closes it afterwards (see
  # TrmnlSim::Simulator#session), returning the block's value.
  def start(build, **kw, &block)
    kw = { name: current_name, mac: Builds::TEST_MAC, turbo: Builds::TURBO, memcheck: Builds::MEMCHECK }.merge(kw)
    s = if Builds.device_of(build)&.env == "TRMNL_X"
          TrmnlX.general_sim(build, **kw)
        else
          TrmnlSim::Simulator.new(Builds.env_of(build), build, **kw)
        end
    block ? s.session(&block) : s
  end
end
