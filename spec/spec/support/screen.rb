# frozen_string_literal: true

require "zlib"

# Reading screenshots.
module Screen
  module_function

  # Bytes per pixel of the 8-bit PNG color types a Simulator#screenshot uses: gray on black-and-white
  # panels, RGB on color ones.
  CHANNELS = { 0 => 1, 2 => 3 }.freeze

  # The rows of a screenshot as gray levels (RGB as its luma), each an array of pixel values.
  def gray_rows(png)
    width, height, depth, color = png.byteslice(16, 10).unpack("NNCC")
    bpp = CHANNELS[color] if depth == 8
    raise ArgumentError, "not an 8-bit gray or RGB PNG: depth #{depth}, color type #{color}" unless bpp

    idat = +"".b
    pos = 8
    while pos < png.bytesize
      n = png.byteslice(pos, 4).unpack1("N")
      idat << png.byteslice(pos + 8, n) if png.byteslice(pos + 4, 4) == "IDAT"
      pos += 12 + n
    end
    raw = Zlib::Inflate.inflate(idat).bytes
    stride = width * bpp
    prev = Array.new(stride, 0)
    Array.new(height) do |y|
      filter = raw[y * (stride + 1)]
      line = raw[(y * (stride + 1)) + 1, stride]
      stride.times do |i|
        a = i >= bpp ? line[i - bpp] : 0
        b = prev[i]
        c = i >= bpp ? prev[i - bpp] : 0
        line[i] = (line[i] + unfilter(filter, a, b, c)) & 0xFF
      end
      prev = line
      bpp == 1 ? line : line.each_slice(3).map { |r, g, b| ((299 * r) + (587 * g) + (114 * b)) / 1000 }
    end
  end

  def unfilter(filter, a, b, c)
    case filter
    when 1 then a
    when 2 then b
    when 3 then (a + b) / 2
    when 4
      p = a + b - c
      pa = (p - a).abs
      pb = (p - b).abs
      pc = (p - c).abs
      if pa <= pb && pa <= pc then a
      elsif pb <= pc then b
      else
        c
      end
    else 0
    end
  end
end
