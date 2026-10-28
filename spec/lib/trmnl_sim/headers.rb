# frozen_string_literal: true

module TrmnlSim
  # Request headers with case-insensitive lookup (HTTP header names are case-insensitive; the
  # TRMNL X modem path delivers them lowercased).
  class Headers
    include Enumerable

    def initialize(pairs = {})
      @h = {}
      pairs.each { |k, v| self[k] = v }
    end

    def [](name) = @h[name.to_s.downcase]

    def []=(name, value)
      @h[name.to_s.downcase] = value
    end

    def fetch(name, *default, &) = @h.fetch(name.to_s.downcase, *default, &)
    def values_at(*names) = names.map { self[_1] }
    def key?(name) = @h.key?(name.to_s.downcase)
    alias include? key?
    def each(&) = @h.each(&)
    def keys = @h.keys
    def to_h = @h.dup
    def inspect = "#<Headers #{@h.inspect}>"
  end
end
