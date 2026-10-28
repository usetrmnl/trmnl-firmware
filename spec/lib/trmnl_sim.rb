# frozen_string_literal: true

# Clients for the trmnl-sim control API and a mock TRMNL server, for integration tests and
# scripts (standard library only). Typical use:
#
#     require "trmnl_sim"                              # with this lib/ on the load path
#
#     TrmnlSim::Simulator.open("trmnl", "../.pio/build/trmnl/merged_firmware.bin", erase: true) do |sim|
#       sim.wait(portal: true, timeout: 60)              # device is in WiFi setup mode
#       sim.wait(display_idle: true, min_refreshes: 1)
#       sim.portal_connect("TRMNL-Sim", "secret")
#       sim.wait(wifi_connected: true)
#       sim.press(1200)                                  # 1.2 s button hold (virtual time)
#       sim.wait_for_console(/deep sleep/)
#       sim.set_net_faults(dns: "servfail")              # inject faults (see set_faults)
#     end
module TrmnlSim
  class Error < StandardError; end
  class TimeoutError < Error; end

  # The trmnl-sim checkout (for its default binary, target/release/trmnl-sim): SIM_REPO, else
  # ../trmnl-sim next to the firmware checkout.
  REPO = File.expand_path(ENV.fetch("SIM_REPO", nil) || File.join(__dir__, "../../../trmnl-sim"))

  # The simulator executable: SIM_BIN, else the checkout's release build.
  def self.binary = ENV.fetch("SIM_BIN", nil) || File.join(REPO, "target/release/trmnl-sim")

  # The merged flash image the simulator runs, for a path to one or a PlatformIO build dir
  # (its merged_firmware.bin). The simulator loads the ELF at the image's path with .elf.
  def self.merged_image(path)
    path = File.expand_path(path.to_s)
    File.directory?(path) ? File.join(path, "merged_firmware.bin") : path
  end
end

require_relative "trmnl_sim/headers"
require_relative "trmnl_sim/images"
require_relative "trmnl_sim/mock_trmnl"
require_relative "trmnl_sim/simulator"
require_relative "trmnl_sim/lcov"
