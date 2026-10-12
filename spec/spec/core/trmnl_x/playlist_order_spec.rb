# frozen_string_literal: true

# The TRMNL X's playlist order: the NVS list of cached images ("playlist_order") the touch bar
# browses. Each download updates it (update_playlist_order in src/filesystem.cpp): a new version
# of a plugin keeps its place, a new plugin goes in after the image shown before it. Shared
# helpers: spec/support/touchbar.rb.

RSpec.describe "TRMNL X playlist order", env: "TRMNL_X" do
  fixture(:shipped) { TrmnlX::ShippedX.new }
  fixture(:dev) { TrmnlX::ProvisionedX.new(shipped) }

  describe "PlaylistOrder" do
    include_context "provisioned X"

    # The cache file of image `name` without its timestamp: the 14 chars that identify the
    # plugin (MockTrmnl#stamp, filesystem_fix_filename).
    def plugin(name) = "/#{dev.mock.filenames.fetch(name)[0, 13]}"

    # playlist_order, by plugin.
    def order(s) = s.preferences.fetch("playlist_order").split("|").map { _1[0, 14] }

    # Serve a new version of image `name` (its timestamp one second on, so it isn't cached),
    # wake the device to download and show it, and wait for it to be back asleep.
    def show_new(s, name)
      m = dev.mock
      m.filenames[name] = m.filenames.fetch(name).sub(/\d+\z/) { (_1.to_i + 1).to_s }
      m.display = { image: name, refresh_rate: 300 }
      n = m.cursor
      s.wake
      m.wait_for_request("/images/#{name}.png", after: n, timeout: 15)
      s.wait(state: "deep_sleep", timeout: 15, settle_ms: 500)
    end

    # The X with "1" and "2" cached, then a new version of "1" and a new plugin "3" shown after
    # it; yields the simulator and the expected screens.
    def three_images
      boot_two_images do |s, one, two|
        three = set_digits(dev.mock, "three", "3")
        expect(order(s)).to eq([plugin("one"), plugin("two")])
        show_new(s, "one")
        expect(order(s)).to eq([plugin("one"), plugin("two")])
        show_new(s, "three")
        yield s, one, two, three
      end
    end

    it "a new plugin goes after the image shown before it" do
      three_images do |s, _one, _two, three|
        expect(order(s)).to eq([plugin("one"), plugin("three"), plugin("two")])
        expect(s).to show_image(three, tolerance: 64, max_ratio: 0)
        expect(s.preferences["browse_path"]).to start_with(plugin("three"))
      end
    end

    # Browsing shows cached images without asking the server, then sleeps
    # (show_cached_image_by_offset in src/touchbar_actions.cpp).
    it "browsing walks the playlist order" do
      three_images do |s, one, two, three|
        n = dev.mock.cursor
        [["right", two], ["right", one], ["left", two], ["left", three]].each do |zone, image|
          touch_and_sleep(s, zone, 150, zone == "right" ? /Next button tapped/ : /Back button tapped/)
          expect(s).to show_image(image, tolerance: 64, max_ratio: 0)
        end
        expect(dev.mock.cursor).to eq(n)
        expect(order(s)).to eq([plugin("one"), plugin("three"), plugin("two")])
      end
    end
  end
end
