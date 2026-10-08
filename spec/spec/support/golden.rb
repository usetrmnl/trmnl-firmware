# frozen_string_literal: true

require "fileutils"

# Golden screenshots: golden/<name> for the TRMNL OG (and 800x480 panels that draw like it),
# golden/<env>/<name> for devices with their own. UPDATE_GOLDEN=1 rewrites them from
# the screen instead of comparing; look at every new or rewritten golden before committing it.
module Golden
  DIR = File.join(Builds::HERE, "golden")

  # The golden screenshots of devices other than the TRMNL OG (whose goldens and regions the
  # tests name): golden/<env>/<name>, compared in the region given here ({env => {name =>
  # [x, y, w, h]}}).
  REGIONS = {
    "TRMNL_X" => {
      # all but the "TRMNL firmware <version> (<git hash>)" line
      "setup_screen_body.png" => [0, 64, 1872, 1340],
      "setup_screen_top_right.png" => [640, 0, 1232, 64],
      # 'Connect your phone or computer to "TRMNL-000001" Wi-Fi'
      "setup_ssid_line.png" => [400, 1236, 1072, 48],
      # "Can't establish WiFi connection. Will keep trying." (the X's font has the apostrophe)
      "wifi_failed_message.png" => [480, 1160, 912, 48],
      # "WiFi connected, unable connect to API." and how to retry (tap the touch bar)
      "api_unable_to_connect.png" => [580, 306, 712, 136],
      # "The image format is incorrect"
      "format_error.png" => [640, 1184, 592, 64]
    },
    # the 960x540 FastEPD panels: Inter_18 at the OG's rows (the centred logo runs into them)
    "TRMNL_X_PAPERS3" => { "api_unable_to_connect.png" => [0, 304, 960, 144] },
    "TRMNL_X_LILYGO_T5PRO" => { "api_unable_to_connect.png" => [0, 304, 960, 144] },
    # 400x600: the centred logo runs into the text too
    "m5_paper_color" => { "api_unable_to_connect.png" => [0, 304, 400, 96] }
  }.freeze

  # A screen that doesn't match; the message says where the actual screen was saved.
  class Mismatch < StandardError; end

  module_function

  def path(*parts) = File.join(DIR, *parts)

  # The device's own region for golden `name` (nil: it has none).
  def own_region(name, device) = REGIONS.dig(device.env, name)

  # The golden screenshot `name` for `device`: its own in golden/<env>/ if it has that
  # directory, else the OG's in golden/ for other 800x480 panels (same layout), else
  # golden/<env>/name (missing: see `check`).
  def for_device(name, device)
    return path(name) if device.env == "trmnl"

    own = path(device.env, name)
    File.directory?(File.dirname(own)) || device.size != [800, 480] ? own : path(name)
  end

  # Compare the screen of `sim` (or its `region`) to the PNG at `file`. With `write_missing`,
  # a missing golden is written from the screen instead; with `update` (UPDATE_GOLDEN)
  # every golden is. Raises Mismatch, saving the actual screen next to the golden as
  # `<name>.<actual_suffix>.png`.
  def check(sim, file, region: nil, actual_suffix: "actual", write_missing: false, update: Builds::UPDATE_GOLDEN,
            **compare)
    if update || (write_missing && !File.exist?(file))
      sim.screenshot(file, region:)
      return
    end
    unless File.exist?(file)
      raise Mismatch,
            "no golden #{file}: make it with UPDATE_GOLDEN=1 and check it before committing it"
    end

    result = sim.compare_screen(file, region:, **compare)
    return if result["match"]

    actual = file.sub(/\.png\z/, ".#{actual_suffix}.png")
    sim.screenshot(actual, region:)
    raise Mismatch, format("screen differs from %<file>s: %<px>d px (%<ratio>.4f%%); actual saved to %<actual>s",
                           file:, px: result["diff_pixels"], ratio: result["diff_ratio"] * 100, actual:)
  end

  # Golden::check against `for_device(name)`, in the device's own region for it if it has one.
  # A missing golden fails instead of being written unless UPDATE_GOLDEN=1: a new
  # device's goldens are made on purpose, and looked at before they are committed. Only the
  # device's own goldens are rewritten (another device's run leaves the OG's alone).
  def check_device(sim, name, device, region: nil, **)
    file = for_device(name, device)
    update = Builds::UPDATE_GOLDEN && (device.env == "trmnl" || File.basename(File.dirname(file)) == device.env)
    check(sim, file, region: own_region(name, device) || region, actual_suffix: "#{device.env}.actual", update:, **)
  end

  # Centred message text (an error, the QA results...), given the OG's golden `name` and its
  # `region`: the device's own golden if REGIONS has one, else, on panels drawing in the OG's
  # font at the OG's rows (Device#og_font?), the OG's golden moved to stay centred on the panel.
  def check_text(sim, name, region, device, **)
    if (own = own_region(name, device))
      check(sim, path(device.env, name), region: own, write_missing: true, **)
    elsif device.og_font?
      x, y, w, h = region
      check(sim, path(name), region: [x + ((device.width - 800) / 2), y, w, h], write_missing: true, **)
    else
      raise Mismatch,
            "no #{name} golden for #{device.name}: add golden/#{device.env}/#{name} and its Golden::REGIONS entry"
    end
  end
end
