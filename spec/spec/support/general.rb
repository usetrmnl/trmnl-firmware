# frozen_string_literal: true

# The general specs: what every device should do, defined once for each device the run covers
# (see Selection).
#
#   General.describe "Refresh cycle" do
#     fixture(:dev) { ProvisionedDevice.new(build) }
#
#     it "reports the device identity", :smoke do
#       ... device.model ...
#     end
#   end
#
# is `RSpec.describe "Refresh cycle"` holding a group per device ("Refresh cycle TRMNL X ...")
# with that device's `env:`, in which `device` and `build` are that device's (also in the group
# body: `if device.button?`). Which of them run depends on how the device is listed (see
# Selection): a group marked `general: :own` holds one device's own examples in a general file
# (defined under `next unless device.env == ...`), which run like its own spec files'.
module General
  module_function

  def describe(description, **meta, &block)
    # The groups' location is the spec file's (for rerun commands and ids), not this file's.
    at = caller
    RSpec.describe(description, **meta, caller: at) do
      Selection.general.each_key do |device|
        describe(device.name, env: device.env, general: true, caller: at, &block)
      end
    end
  end
end
