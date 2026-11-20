# frozen_string_literal: true

require "zlib"

module TrmnlSim
  # Test images in the formats TRMNL servers produce, and the screenshots the simulator should
  # give for them. A picture is a callable `pixel.(x, y)`: true for ink (1-bit images), a gray
  # level (`level`), or an [r, g, b] color (`color`).
  module Images
    WIDTH = 800
    HEIGHT = 480

    BWRY_RGB = { black: [0, 0, 0], white: [255, 255, 255], yellow: [255, 255, 0], red: [255, 0, 0] }.freeze
    BWR_RGB = BWRY_RGB.slice(:black, :white, :red).freeze
    SPECTRA6_RGB = BWRY_RGB.merge(blue: [0, 0, 255], green: [0, 255, 0]).freeze

    PNG_SIGNATURE = "\x89PNG\r\n\x1a\n".b

    # 5x7 digits
    FONT = {
      "0" => "01110100011001110101110011000101110", "1" => "00100011000010000100001000010001110",
      "2" => "01110100010000100010001000100011111", "3" => "11111000100010000010000011000101110",
      "4" => "00010001100101010010111110001000010", "5" => "11111100001111000001000011000101110",
      "6" => "00110010001000011110100011000101110", "7" => "11111000010001000100010000100001000",
      "8" => "01110100011000101110100011000101110", "9" => "01110100011000101111000010001001100"
    }.freeze

    module_function

    # ---- encoders -------------------------------------------------------------------------------

    # 800x480 1-bit BMP in the format TRMNL serves. `pixel.(x, y)` is true for black.
    def bmp_1bit(pixel)
      row = WIDTH / 8 # 100 bytes, already 4-byte aligned
      data = Array.new(row * HEIGHT, 0)
      HEIGHT.times do |y|
        base = (HEIGHT - 1 - y) * row # bottom-up
        WIDTH.times do |x|
          data[base + (x / 8)] |= 0x80 >> (x % 8) unless pixel.(x, y) # palette index 1 = white
        end
      end
      palette = [0, 0, 0, 0, 255, 255, 255, 0].pack("C*")
      offset = 14 + 40 + palette.bytesize
      header = "BM".b + [offset + data.size, 0, 0, offset].pack("VvvV")
      info = [40, WIDTH, HEIGHT, 1, 1, 0, data.size, 2835, 2835, 2, 2].pack("Vl<l<vvVVl<l<VV")
      header + info + palette + data.pack("C*")
    end

    # The PNG the simulator's screenshot of `bmp_1bit(pixel)` should match (0 = ink, 255 = paper).
    def png_gray(pixel)
      gray8(WIDTH, HEIGHT) { |x, y| pixel.(x, y) ? 0 : 255 }
    end

    # A grayscale PNG of `bits` depth (1, 2, 4 or 8) as TRMNL servers produce them.
    # `level.(x, y)` is 0 (black) .. 2**bits - 1 (white).
    def png_image(level, width, height, bits: 1)
      raise ArgumentError, "bits must be 1, 2, 4 or 8" unless [1, 2, 4, 8].include?(bits)

      mask = (1 << bits) - 1
      raw = packed_rows(width, height, bits) { |x, y| level.(x, y) & mask }
      png(ihdr(width, height, bits, 0), raw)
    end

    # The 8-bit PNG a simulator screenshot of `png_image(level, ...)` should match (0 = ink).
    def expected_gray(level, width, height, bits: 1)
      top = (1 << bits) - 1
      gray8(width, height) { |x, y| (255.0 * (level.(x, y) & top) / top).round }
    end

    # A truecolor PNG; `color.(x, y)` is [r, g, b].
    def png_rgb(color, width = WIDTH, height = HEIGHT)
      rgb8(width, height) { |x, y| color.(x, y) }
    end

    # A truecolor PNG with an (opaque) alpha channel; `color.(x, y)` is [r, g, b].
    def png_rgba(color, width = WIDTH, height = HEIGHT)
      raw = rows(height) { |y| Array.new(width) { |x| [*color.(x, y), 255] }.flatten }
      png(ihdr(width, height, 8, 6), raw)
    end

    # An indexed PNG (as TRMNL serves color images), by default 2 bits per pixel for up to 4
    # colors, else 4; `color.(x, y)` must return one of the `palette` colors.
    def png_palette(color, palette, width = WIDTH, height = HEIGHT, bits: nil)
      index = palette.each_with_index.to_h
      bits ||= palette.size <= 4 ? 2 : 4
      raise ArgumentError, "#{palette.size} colors don't fit #{bits} bits" if palette.size > (1 << bits)

      raw = packed_rows(width, height, bits) { |x, y| index.fetch(color.(x, y).to_a) }
      png(ihdr(width, height, bits, 3), raw, "PLTE" => palette.flatten.pack("C*"))
    end

    # ---- color reduction, as the firmware does it -----------------------------------------------------

    # The TRMNL BWRY firmware's color reduction (GetBWYRPixel in display.cpp).
    def bwry_quantize(r, g, b)
      gr = (b + r + (g * 2)) >> 2
      if r > b || g > b
        return BWRY_RGB[:black] if gr < 90 && r < 80 && g < 80
        return BWRY_RGB[:red] if r - b > 32 && r - g > r / 2
        return BWRY_RGB[:yellow] if r - b > 32 && g - b > 32

        return BWRY_RGB[:white]
      end
      gr >= 100 ? BWRY_RGB[:white] : BWRY_RGB[:black]
    end

    # Black/white/red color reduction (GetBWRPixel in display.cpp; the firmware defines it but
    # no image path calls it yet).
    def bwr_quantize(r, g, b)
      gr = (b + r + (g * 2)) >> 2
      if r > g && r > b
        return BWR_RGB[:black] if gr < 100 && r < 80

        return r - b > 32 && r - g > 32 ? BWR_RGB[:red] : BWR_RGB[:white]
      end
      gr >= 128 ? BWR_RGB[:white] : BWR_RGB[:black]
    end

    SPECTRA6_REFERENCE_INKS = [[0, 0, 0], [192, 192, 192], [192, 192, 0], [192, 0, 0], [0, 0, 192], [0, 192, 0]].freeze

    # The Spectra 6 firmware's color reduction (GetSpectraPixel in display.cpp): the nearest of
    # its reference inks to the color's RGB333 value.
    def spectra6_quantize(r, g, b)
      c = [r, g, b].map { |v| (v >> 5) * 36 }
      dist = SPECTRA6_REFERENCE_INKS.map { |ink| ink.zip(c).sum { |i, v| (v - i)**2 } }
      SPECTRA6_RGB.values[dist.index(dist.min)]
    end

    # The RGB PNG a simulator screenshot of an image of `color` should match on a BWRY panel.
    def expected_bwry(color, width = WIDTH, height = HEIGHT)
      rgb8(width, height) { |x, y| bwry_quantize(*color.(x, y)) }
    end

    # ...on a black/white/red panel.
    def expected_bwr(color, width = WIDTH, height = HEIGHT)
      rgb8(width, height) { |x, y| bwr_quantize(*color.(x, y)) }
    end

    # ...on a Spectra 6 panel (reTerminal E1002).
    def expected_spectra6(color, width = WIDTH, height = HEIGHT)
      rgb8(width, height) { |x, y| spectra6_quantize(*color.(x, y)) }
    end

    # ---- pictures -------------------------------------------------------------------------------

    # Vertical black/white/yellow/red bars, with a red/yellow checker band in the middle.
    def color_bars
      names = %i[black white yellow red]
      lambda do |x, y|
        next BWRY_RGB[names[2 + (((x / 40) + (y / 40)) % 2)]] if (200...280).cover?(y)

        BWRY_RGB[names[[3, x * 4 / 800].min]]
      end
    end

    # Vertical black/white/red bars (thirds of 800 px), with a red/black checker band.
    def bwr_bars
      names = BWR_RGB.keys
      lambda do |x, y|
        next BWR_RGB[%i[red black][((x / 40) + (y / 40)) % 2]] if (200...280).cover?(y)

        BWR_RGB[names[[2, x * 3 / 800].min]]
      end
    end

    # Vertical bars of the six Spectra 6 inks, with a blue/green checker band in the middle.
    def spectra_bars
      names = SPECTRA6_RGB.keys
      lambda do |x, y|
        next SPECTRA6_RGB[names[4 + (((x / 40) + (y / 40)) % 2)]] if (200...280).cover?(y)

        SPECTRA6_RGB[names[[5, x * 6 / 800].min]]
      end
    end

    # Vertical bands, one per gray level (0 = black at the left).
    def ramp(levels = 16)
      ->(x, _y) { [levels - 1, x * levels / 1872].min }
    end

    def checkerboard(size = 40)
      ->(x, y) { ((x / size) + (y / size)).even? }
    end

    # Vertical stripes; asymmetric (left half thin bars) so orientation bugs show up.
    def bars(n = 8)
      ->(x, y) { x < WIDTH / 2 ? (x / (WIDTH / n)).even? : (y / 60).even? }
    end

    # Digits drawn huge and centred on 800x480: easy to eyeball, distinct per test step.
    def big_number(text, scale: 24)
      text = text.to_s
      w = (text.size * 6 * scale) - scale
      x0 = (WIDTH - w) / 2
      y0 = (HEIGHT - (7 * scale)) / 2
      lambda do |x, y|
        cx = (x - x0).div(scale)
        cy = (y - y0).div(scale)
        next false if cx.negative? || cy.negative? || cy >= 7

        i, col = cx.divmod(6)
        next false if i >= text.size || col >= 5

        FONT.fetch(text[i])[(cy * 5) + col] == "1"
      end
    end

    # ---- PNG plumbing ----------------------------------------------------------------------------

    def ihdr(width, height, bits, color_type)
      [width, height, bits, color_type, 0, 0, 0].pack("NNCCCCC")
    end

    def png(ihdr, raw, extra_chunks = {})
      chunks = [["IHDR", ihdr], *extra_chunks.to_a, ["IDAT", Zlib::Deflate.deflate(raw)], ["IEND", "".b]]
      PNG_SIGNATURE + chunks.map { |tag, body| chunk(tag, body) }.join
    end

    def chunk(tag, body)
      tagged = tag.b + body.b
      [body.bytesize].pack("N") + tagged + [Zlib.crc32(tagged)].pack("N")
    end

    # Filter-type-0 scanlines: `row.(y)` returns the row's bytes as an array of integers.
    def rows(height)
      (0...height).map { |y| [0, *yield(y)].pack("C*") }.join
    end

    def gray8(width, height)
      png(ihdr(width, height, 8, 0), rows(height) { |y| Array.new(width) { |x| yield(x, y) } })
    end

    def rgb8(width, height)
      png(ihdr(width, height, 8, 2), rows(height) { |y| Array.new(width) { |x| yield(x, y) }.flatten })
    end

    # Rows of `bits`-per-pixel values packed MSB first.
    def packed_rows(width, height, bits)
      per_byte = 8 / bits
      rows(height) do |y|
        row = Array.new(((width * bits) + 7) / 8, 0)
        width.times { |x| row[x / per_byte] |= yield(x, y) << (8 - (bits * ((x % per_byte) + 1))) }
        row
      end
    end
  end
end
