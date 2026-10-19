# frozen_string_literal: true

# The TRMNL X factory ship check (src/touchbar_actions.cpp, touchbar_poll_ship_check): in the setup
# portal, a 2 s middle press shows a screen to tap the left, center and right of the touch bar in turn
# (src/menu.cpp, menu_run_ship_check); once they all work the X shuts down for shipment, like Settings'
# Shut Down.

RSpec.describe "TRMNL X touch bar: ship check", env: "TRMNL_X" do
  fixture(:shipped) { TrmnlX::ShippedX.new }

  # Boot the shipped X onto its dock, out of shipment mode into the setup portal, then off the dock.
  def boot_to_portal
    shipped.boot do |s|
      s.wait_for_console(/Entering shipment mode light sleep loop/, timeout: 30)
      s.dock(true)
      s.wait(portal: true, timeout: 90)
      s.dock(false)
      s.wait(display_idle: true, timeout: 30)
      yield s
    end
  end

  it "shuts down for shipment once each zone has been tapped in turn" do
    boot_to_portal do |s|
      c = s.status["console_total"]
      s.touch("center", ms: 2500)
      s.wait(console: /tutorial_show_screen end/, since: c, timeout: 30)
      expect(s.console(c).join("\n")).to match(/opening the ship check/)

      TrmnlX.menu_tap(s, "right", /Tutorial: tapped Right, waiting for Left/)
      TrmnlX.menu_tap(s, "left", /Tutorial: tried Left/)
      TrmnlX.menu_tap(s, "center", /Tutorial: tried Center/)
      c = s.status["console_total"]
      s.touch("right", ms: 150)
      s.wait(console: /Ship check passed - shutting down/, since: c, timeout: 15)
      s.wait(console: /Entering shipment mode light sleep loop/, since: c, timeout: 60)
      expect(s.status["portal_url"]).to be_nil
    end
  end

  it "ignores a short middle press" do
    boot_to_portal do |s|
      c = s.status["console_total"]
      s.touch("center", ms: 1000)
      t0 = s.status["sim_time_s"]
      sleep 0.2 while s.status["sim_time_s"] < t0 + 3
      expect(s.console(c).join("\n")).not_to match(/ship check/)
      expect(s.status["portal_url"]).not_to be_nil
    end
  end

  it "goes back to the setup portal when the check times out" do
    boot_to_portal do |s|
      c = s.status["console_total"]
      s.touch("center", ms: 2500)
      s.wait(console: /tutorial_show_screen end/, since: c, timeout: 30)
      s.set_portal_client(false) # so turbo can run to the timeout
      s.wait(console: /Ship check: timeout - back to the setup portal/, since: c, timeout: 120)
      s.set_portal_client(true) # portal_url is only reported while a client is connected
      expect(s.wait(portal: true, timeout: 90)["status"]["portal_url"]).not_to be_nil
    end
  end
end
