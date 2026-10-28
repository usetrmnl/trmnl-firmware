# frozen_string_literal: true

# Matchers for simulators (their screens) and recorded requests.
#
#   expect(s).to show_image(expected_png, tolerance: 64, max_ratio: 0)
#   expect(s).to match_golden("setup_screen.png", region: [0, 0, 800, 64])
#   expect(s).to match_screenshot(Golden.path("TRMNL_X", "boot.png"))
#   expect(s).to show_message("api_unable_to_connect.png", [180, 300, 440, 60])
#   expect(req).to have_header("Update-Source", "timer")
#   expect(req).to have_header("Battery-Voltage", a_value_within(0.05).of(3.7))
#   expect(req).not_to have_header("Panel-Rev")
module Matchers
  extend RSpec::Matchers::DSL

  # The screen (or `region` [x, y, w, h] of it) matches a PNG (bytes or a file), within
  # `tolerance` per pixel and `max_ratio` of the pixels differing (Simulator#compare_screen).
  matcher :show_image do |expected, **compare|
    match do |sim|
      @result = sim.compare_screen(expected, **compare)
      @result["match"]
    end

    failure_message do
      "expected the screen to show the image#{" in #{compare[:region]}" if compare[:region]}: " \
        "#{@result['diff_pixels']} px differ (#{format('%.4f', @result['diff_ratio'] * 100)}%): #{@result}"
    end

    failure_message_when_negated { "expected the screen not to show the image: #{@result}" }
  end

  # Mismatch-raising golden checks as matchers.
  def self.golden_matcher(name, &check)
    matcher name do |*args, **kw|
      match do |sim|
        instance_exec(sim, *args, **kw, &check)
        true
      rescue Golden::Mismatch => e
        @message = e.message
        false
      end

      failure_message { @message }
    end
  end

  # The device's golden screenshot `name` (Golden.check_device); a missing golden fails.
  golden_matcher(:match_golden) { |sim, name, region: nil, **kw| Golden.check_device(sim, name, device, region:, **kw) }

  # The golden at `path` (Golden.check); a missing one is written from the screen.
  golden_matcher(:match_screenshot) { |sim, path, **kw| Golden.check(sim, path, write_missing: true, **kw) }

  # Centred message text, given the OG's golden `name` and `region` (Golden.check_text).
  golden_matcher(:show_message) { |sim, name, region, **kw| Golden.check_text(sim, name, region, device, **kw) }

  # A request header (names are case-insensitive), optionally with a value: a string, or any
  # matcher (matching the header's text, or its number: `a_value_within(0.05).of(3.7)`).
  matcher :have_header do |name, *value|
    match do |req|
      @actual_value = req.headers[name]
      next false if @actual_value.nil?
      next true if value.empty?

      expected = value.first
      number = Float(@actual_value, exception: false)
      values_match?(expected, @actual_value) || (!number.nil? && values_match?(expected, number))
    end

    description do
      "have header #{name}#{" #{RSpec::Support::ObjectFormatter.format(value.first)}" unless value.empty?}"
    end

    failure_message do |req|
      return "expected a #{name} header, got #{req.headers.to_h}" if @actual_value.nil?

      "expected #{name} to be #{RSpec::Support::ObjectFormatter.format(value.first)}, got #{@actual_value.inspect}"
    end

    failure_message_when_negated do
      "expected no #{name} header#{" of #{value.first.inspect}" unless value.empty?}, got #{@actual_value.inspect}"
    end
  end
end
