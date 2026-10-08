# frozen_string_literal: true

# The TRMNL X touch bar in the setup portal: holding both edges asks whether to power off (back
# to shipment mode); a middle hold confirms, a tap or 15 s without an answer cancels. Shared
# helpers: spec/support/touchbar.rb.

RSpec.describe "TRMNL X touch bar: power off", env: "TRMNL_X" do
  include Touchbar

  fixture(:shipped) { TrmnlX::ShippedX.new }

  describe "PowerOffConfirmation" do
    # Boot the shipped X into the portal and ask to power off; yields the simulator and the
    # console index before. The host's portal client leaves once the portal is up, so turbo
    # runs (a portal client keeps the simulation at wall-clock pace).
    def ask
      shipped.boot do |s|
        s.wait_for_console(/Entering shipment mode light sleep loop/, timeout: 30)
        s.dock(true)
        s.wait(portal: true, timeout: 30)
        s.set_portal_client(false)
        s.dock(false)
        s.wait(display_idle: true, settle_ms: 200, timeout: 15)
        c = s.status["console_total"]
        hold_edges(s, "left", "right")
        s.wait(console: /Entering power-off confirmation mode/, since: c, timeout: 15)
        lift(s, "left", "right")
        s.wait(console: /display_show_msg end/, since: c, timeout: 15)
        s.wait(display_idle: true, settle_ms: 100, timeout: 15)
        yield s, c
      end
    end

    def expect_cancelled(s, c)
      s.wait(console: /Entering power-off confirmation mode/, since: c, timeout: 1)
      s.wait(display_idle: true, settle_ms: 200, timeout: 30)
      s.set_portal_client(true) # the portal carries on: the client can join it again
      expect(s.wait(portal: true, timeout: 30)["status"]["portal_url"]).not_to be_nil
    end

    it "middle hold powers off" do
      ask do |s, c|
        boots = s.status["boot_count"]
        s.touch("center", ms: 1500)
        s.wait(console: /Confirmed - holding middle button in tap mode/, since: c, timeout: 15)
        deadline = Process.clock_gettime(Process::CLOCK_MONOTONIC) + 30
        while s.status["boot_count"] == boots
          expect(Process.clock_gettime(Process::CLOCK_MONOTONIC)).to be < deadline, "did not restart"
          sleep 0.2
        end
        # shipment status cleared: back to shipment mode until the charger is connected
        c = s.status["console_total"]
        s.wait(console: /Entering shipment mode light sleep loop/, since: c, timeout: 60)
      end
    end

    it "a tap cancels" do
      ask do |s, c|
        s.touch("left", ms: 150) # the portal runs the touch bar in tap mode
        s.wait(console: /Confirmation cancelled - outer button in tap mode/, since: c, timeout: 15)
        expect_cancelled(s, c)
      end
    end

    it "no answer cancels after 15 seconds" do
      ask do |s, c|
        s.wait(console: /Confirmation timeout - cancelling/, since: c, timeout: 30)
        expect_cancelled(s, c)
      end
    end
  end
end
