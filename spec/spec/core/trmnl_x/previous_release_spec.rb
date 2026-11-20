# frozen_string_literal: true

# The update the fleet gets: an X onboarded on the newest release before this firmware
# (Builds.previous_release, built by `rake "firmware:build_release[<tag>]"`) updates to it.

RSpec.describe "TRMNL X on the previous release",
               env: "TRMNL_X", skip: Builds.release_missing(Builds.previous_release) do
  include_context "X updates to this firmware"

  fixture(:shipped) { TrmnlX::ShippedX.new(Builds.release(Builds.previous_release)) }
  # onboarded on 5 GHz: the update goes through the modem
  fixture(:dev) { TrmnlX::ProvisionedX.new(shipped, image: "white") }
  fixture(:dev_2ghz) { TrmnlX::ProvisionedX.new(shipped, ssid: TrmnlX::SSID_24, image: "white") }

  let(:previous_version) { Builds.previous_release.delete_prefix("v") }

  describe "FleetUpdate" do
    it "updates to this firmware through the modem on 5 GHz", :smoke do
      dev.reset
      expect_update(dev, from: previous_version)
    end

    it "updates to this firmware on 2.4 GHz" do
      dev_2ghz.reset
      expect_update(dev_2ghz, from: previous_version)
    end
  end
end
