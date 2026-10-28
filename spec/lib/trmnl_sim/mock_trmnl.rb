# frozen_string_literal: true

require "digest"
require "json"
require "monitor"
require "openssl"
require "socket"

require_relative "headers"
require_relative "images"

module TrmnlSim
  # A request the mock server received.
  # rubocop:disable Lint/StructNewOverride -- `method` is the HTTP method
  RecordedRequest = Struct.new(:method, :path, :headers, :body, :at, :tls_resumed, keyword_init: true) do
    def json = JSON.parse(body.to_s.empty? ? "null" : body)
  end
  # rubocop:enable Lint/StructNewOverride

  # A mock TRMNL API server for hermetic integration tests (standard library only).
  #
  # The simulated device reaches the host as 10.0.2.2, so point the device at `device_url`
  # (e.g. via `Simulator#portal_connect(..., server: mock.device_url)`).
  #
  #     mock = TrmnlSim::MockTrmnl.new
  #     mock.set_image("hello", Images.checkerboard(40))
  #     mock.display = { image: "hello", refresh_rate: 300 }
  #     req = mock.wait_for_request("/api/display")
  #     req.headers["Battery-Voltage"]  # => "4.10"
  #
  # HTTP-level faults are per path (exact, or a prefix ending in "*"):
  #
  #     mock.set_fault("/api/display", status: 500)          # every /api/display answers 500
  #     mock.set_fault("/images/*", truncate: 1000, times: 1) # the next image stops after 1000 bytes
  #     mock.clear_faults
  #
  # `MockTrmnl.new(tls: true)` serves HTTPS instead: TLS 1.2 only,
  # ECDHE-ECDSA-AES256-GCM-SHA384 with a throwaway P-384 certificate, so the handshake needs
  # SHA-384, ECDSA, ECDH and AES-GCM (the devices connect with certificate checks off, like
  # they do to trmnl.app).
  #
  # Attributes to change between steps:
  #   setup:         merged into the /api/setup answer (nil: answer "status": 404, not registered)
  #   display:       the next /api/display answer: image (name), refresh_rate, plus any raw
  #                  fields (update_firmware, firmware_url, special_function, ...)
  #   display_queue: such hashes consumed first, one per request
  class MockTrmnl
    FAULT_KEYS = %i[status body content_type delay hang truncate rate close redirect chunked times].freeze
    REASONS = { 200 => "OK", 301 => "Moved Permanently", 302 => "Found", 307 => "Temporary Redirect",
                308 => "Permanent Redirect", 404 => "Not Found", 500 => "Internal Server Error" }.freeze

    attr_reader :requests, :images, :filenames, :files, :faults, :port, :tls
    attr_accessor :api_key, :friendly_id, :setup, :display, :display_queue, :device_host

    # A mock for the duration of the block.
    def self.open(**)
      mock = new(**)
      return mock unless block_given?

      begin
        yield mock
      ensure
        mock.close
      end
    end

    def initialize(host: "127.0.0.1", port: 0, tls: false)
      @requests = []
      @images = {}
      @filenames = {}
      @files = {}
      @faults = {}
      @api_key = "sim-test-api-key"
      @friendly_id = "SIMTST"
      @setup = {}
      @display = { image: "default", refresh_rate: 900 }
      @display_queue = []
      # How the device names this server. Use a hostname (with the simulator's
      # `--dns NAME=10.0.2.2`) to make the device resolve it, e.g. for DNS fault tests.
      @device_host = "10.0.2.2"
      @tls = tls
      @lock = Monitor.new
      @changed = @lock.new_cond
      @closing = false
      @closed_cond = @lock.new_cond
      set_image("default", Images.big_number("0"))

      @server = TCPServer.new(host, port)
      @port = @server.addr[1]
      @ssl_context = tls_context(@device_host) if tls
      @connections = []
      @acceptor = Thread.new { accept_loop }
    end

    # URLs as seen from the device (10.0.2.2 is the host) and from the host.
    def device_url = "#{scheme}://#{device_host}:#{port}"
    def host_url = "#{scheme}://127.0.0.1:#{port}"

    def close
      @lock.synchronize do
        @closing = true
        @closed_cond.broadcast
      end
      @server.close unless @server.closed?
      @acceptor.join(5)
      @connections.each { |t| t.join(2) || t.kill }
    end

    def closed? = @closing

    # A cursor for `wait_for_request(after:)`: the number of requests so far.
    def cursor = @lock.synchronize { @requests.size }

    # Forget the requests, queued answers and faults, and answer /api/display with `display`.
    def reset(display: { image: "default", refresh_rate: 300 })
      @lock.synchronize do
        @requests.clear
        @display_queue.clear
        @faults.clear
        @display = display
      end
      self
    end

    # ---- images and files --------------------------------------------------------------------------

    # Register an 800x480 1-bit BMP (TRMNL OG); returns the PNG a matching screenshot should equal.
    def set_image(name, pixel = nil, &block)
      pixel ||= block
      images[name] = Images.bmp_1bit(pixel)
      stamp(name)
      Images.png_gray(pixel)
    end

    # Register a grayscale PNG (TRMNL X: 1872x1404, 1- or 4-bit); served as images/<name>.png.
    # Returns the expected screenshot PNG (exact for 1-bit; 4-bit grays are panel-model
    # approximations).
    def set_png(name, level, width: 1872, height: 1404, bits: 1)
      images["#{name}.png"] = Images.png_image(level, width, height, bits:)
      stamp(name)
      Images.expected_gray(level, width, height, bits:)
    end

    # Register a color image for a TRMNL BWRY, reduced to black/white/yellow/red and served as a
    # 2-bit palette PNG like the TRMNL server does (the OG-family firmware's PNG decoder can't
    # take 800 px wide truecolor rows). Returns the RGB PNG a screenshot should match.
    def set_color_png(name, color, width = Images::WIDTH, height = Images::HEIGHT)
      set_palette_png(name, color, width, height, Images::BWRY_RGB) { |c| Images.bwry_quantize(*c) }
      Images.expected_bwry(color, width, height)
    end

    # ...for a black/white/red panel, reduced to its three inks.
    def set_bwr_png(name, color, width = Images::WIDTH, height = Images::HEIGHT)
      set_palette_png(name, color, width, height, Images::BWR_RGB) { |c| Images.bwr_quantize(*c) }
      Images.expected_bwr(color, width, height)
    end

    # ...for a Spectra 6 panel (reTerminal E1002), reduced to its six inks (a 4-bit palette PNG).
    def set_spectra6_png(name, color, width = Images::WIDTH, height = Images::HEIGHT)
      set_palette_png(name, color, width, height, Images::SPECTRA6_RGB) { |c| Images.spectra6_quantize(*c) }
      Images.expected_spectra6(color, width, height)
    end

    # Server-style filename for an image: plugin-<6 hex id>-<epoch>. The TRMNL X caches images
    # under it: the first 14 chars identify the plugin (a new version replaces the old one) and
    # files whose timestamp is over 24 h old are purged.
    def stamp(name)
      filenames[name] = "plugin-#{Digest::SHA1.hexdigest(name)[0, 6]}-#{Time.now.to_i}"
    end

    # Serve arbitrary bytes (e.g. a firmware binary for OTA tests); returns the device URL.
    def set_file(path, content_type, data)
      files[path] = [content_type, data.b]
      device_url + path
    end

    # The path /api/display sends the device to for image `name` (a PNG if one is registered
    # under that name, else the BMP).
    def image_path(name) = "/images/#{name}.#{images.key?("#{name}.png") ? 'png' : 'bmp'}"

    # The key in `images` of the image served at `path` (an image_path).
    def image_key(path) = path.delete_prefix("/images/").delete_suffix(".bmp")

    # The size of the image served at `path` (an image_path).
    def image_size(path) = images.fetch(image_key(path)).bytesize

    # ---- faults ---------------------------------------------------------------------------------------

    # Make requests to `path` (exact, or a prefix ending in "*", e.g. "/images/*") misbehave:
    #
    #   status:       answer with this HTTP status (and a short text body)
    #   body:         answer with this body instead (e.g. malformed JSON: '{"status": 0, "image_')
    #   content_type: override the Content-Type
    #   delay:        seconds to wait before answering (longer than the device's timeout = a timeout)
    #   hang:         never answer (until the mock is closed)
    #   truncate:     send only this many bytes of the body (Content-Length is still the full
    #                 size), then close the connection
    #   rate:         send the body at this many bytes per second (a slow download)
    #   close:        close the connection without answering
    #   redirect:     answer with a redirect to this URL (status 307 unless `status` is given)
    #   chunked:      send the body chunked, without a Content-Length (with `truncate`: stop
    #                 after that many bytes without the final chunk)
    #   times:        only the next N matching requests (default: until cleared)
    def set_fault(path, **spec)
      unknown = spec.keys - FAULT_KEYS
      raise ArgumentError, "unknown fault #{unknown.join(', ')}" if unknown.any?

      @lock.synchronize { faults[path] = spec }
    end

    def clear_faults = @lock.synchronize { faults.clear }

    # ---- observation -------------------------------------------------------------------------------

    # Wait for request number >= `after` to `path` (use `cursor` as a cursor).
    def wait_for_request(path, after: 0, timeout: 60)
      deadline = monotonic + timeout
      @lock.synchronize do
        loop do
          found = @requests.drop(after).find { |r| r.path == path }
          return found if found

          left = deadline - monotonic
          raise TimeoutError, "no request to #{path} within #{timeout}s (seen: #{@requests.map(&:path)})" if left <= 0

          @changed.wait(left)
        end
      end
    end

    # The first request to `path` made after the block started (the block does what makes the
    # device ask, e.g. `sim.wake`).
    def next_request(path, timeout: 60)
      after = cursor
      yield if block_given?
      wait_for_request(path, after:, timeout:)
    end

    def count(path) = @lock.synchronize { @requests.count { |r| r.path == path } }

    # The requests' paths, in order.
    def paths = @lock.synchronize { @requests.map(&:path) }

    private

    def scheme = tls ? "https" : "http"
    def monotonic = Process.clock_gettime(Process::CLOCK_MONOTONIC)

    def set_palette_png(name, color, width, height, inks)
      reduce = ->(x, y) { yield(color.(x, y)) }
      images["#{name}.png"] = Images.png_palette(reduce, inks.values, width, height)
      stamp(name)
    end

    # Server context with a fresh self-signed P-384 ECDSA certificate (a v3 one: mbedTLS rejects v1).
    def tls_context(host)
      key = OpenSSL::PKey::EC.generate("secp384r1")
      cert = OpenSSL::X509::Certificate.new
      cert.version = 2
      cert.serial = rand(1 << 64)
      cert.subject = cert.issuer = OpenSSL::X509::Name.parse("/CN=#{host}")
      cert.public_key = key
      cert.not_before = Time.now - 60
      cert.not_after = Time.now + (2 * 86_400)
      ext = OpenSSL::X509::ExtensionFactory.new(cert, cert)
      san = host.match?(/\A[\d.]+\z/) ? "IP:#{host}" : "DNS:#{host}"
      cert.add_extension(ext.create_extension("basicConstraints", "CA:FALSE"))
      cert.add_extension(ext.create_extension("subjectAltName", san))
      cert.sign(key, OpenSSL::Digest.new("SHA384"))

      ctx = OpenSSL::SSL::SSLContext.new
      ctx.min_version = ctx.max_version = OpenSSL::SSL::TLS1_2_VERSION
      ctx.ciphers = "ECDHE-ECDSA-AES256-GCM-SHA384"
      ctx.cert = cert
      ctx.key = key
      ctx.session_id_context = "trmnl-mock"
      ctx
    end

    def accept_loop
      loop do
        sock = @server.accept
        @connections << Thread.new { serve_connection(sock) }
        @connections.reject!(&:stop?) if @connections.size > 64
      end
    rescue IOError, Errno::EBADF
      nil # closed
    end

    def serve_connection(sock)
      io = sock
      tls_resumed = nil
      if @ssl_context
        # Handshake here, in the connection's thread, not in the accept loop.
        io = OpenSSL::SSL::SSLSocket.new(sock, @ssl_context)
        io.sync_close = true
        io.accept
        tls_resumed = io.session_reused?
      end
      serve(io, tls_resumed)
    rescue IOError, SystemCallError, OpenSSL::SSL::SSLError # EOFError is an IOError
      nil # the device went away (e.g. a power-loss fault mid-download)
    ensure
      begin
        io&.close
      rescue StandardError
        nil
      end
    end

    def serve(io, tls_resumed)
      request_line = io.gets("\r\n") or return
      method, target = request_line.split
      headers = Headers.new
      while (line = io.gets("\r\n")) && line != "\r\n"
        name, value = line.chomp("\r\n").split(":", 2)
        headers[name] = value.to_s.strip
      end
      length = headers["Content-Length"].to_i
      body = length.positive? ? io.read(length) : "".b
      rec = RecordedRequest.new(method:, path: target.split("?").first, headers:, body:, at: Time.now, tls_resumed:)
      @lock.synchronize do
        @requests << rec
        @changed.broadcast
      end
      respond(io, rec, take_fault(rec.path) || {})
    end

    def respond(io, rec, fault)
      return pause(300) if fault[:hang]

      pause(fault[:delay]) if fault[:delay]
      return if fault[:close]

      if fault[:redirect]
        return write_head(io, fault[:status] || 307, "Location" => fault[:redirect], "Content-Length" => "0")
      end

      code, ctype, payload = answer(rec)
      if fault[:status]
        code = fault[:status]
        ctype = "text/plain"
        payload = "fault: HTTP #{fault[:status]}"
      end
      payload = fault[:body] if fault[:body]
      payload = payload.b
      ctype = fault[:content_type] || ctype
      truncate = fault[:truncate]

      if fault[:chunked]
        # No Content-Length: the body is sent chunked.
        write_head(io, code, "Content-Type" => ctype, "Transfer-Encoding" => "chunked")
        sent = truncate ? payload.byteslice(0, truncate) : payload
        sent.bytes.each_slice(4096) { |part| io.write("#{part.size.to_s(16)}\r\n#{part.pack('C*')}\r\n") }
        io.write("0\r\n\r\n") unless truncate
        return
      end

      # A truncated body still announces its full length, like a connection that dies.
      write_head(io, code, "Content-Type" => ctype, "Content-Length" => payload.bytesize.to_s)
      payload = payload.byteslice(0, truncate) if truncate
      if (rate = fault[:rate])
        step = [1, rate / 20].max
        (0...payload.bytesize).step(step).each do |i|
          io.write(payload.byteslice(i, step))
          io.flush
          break if pause(step.to_f / rate) # the mock is closing
        end
      else
        io.write(payload)
      end
    end

    # Wait `seconds`, or less if the mock closes; true if it closed.
    def pause(seconds)
      @lock.synchronize do
        @closed_cond.wait(seconds) unless @closing
        @closing
      end
    end

    def write_head(io, code, headers)
      head = "HTTP/1.1 #{code} #{REASONS.fetch(code, 'Status')}\r\n"
      headers.merge("Connection" => "close").each { |k, v| head << "#{k}: #{v}\r\n" }
      io.write("#{head}\r\n")
    end

    def take_fault(path)
      @lock.synchronize do
        pattern, spec = faults.find { |p, _| p == path || (p.end_with?("*") && path.start_with?(p.chomp("*"))) }
        next unless spec

        if spec[:times]
          spec[:times] -= 1
          faults.delete(pattern) if spec[:times] <= 0
        end
        spec
      end
    end

    def answer(rec)
      case rec.path
      when "/api/setup" then setup_answer(rec)
      when "/api/display" then json(display_answer)
      when "/api/log" then json(status: 200)
      when %r{\A/images/(.+\.png)\z} then image_answer(images[Regexp.last_match(1)], "image/png")
      when %r{\A/images/(.+)\.bmp\z} then image_answer(images[Regexp.last_match(1)], "image/bmp")
      else
        ctype, data = files[rec.path]
        data ? [200, ctype, data] : [404, "text/plain", "not found"]
      end
    end

    def setup_answer(rec)
      if setup.nil?
        # as trmnl.app answers an unknown MAC: HTTP 200, "status": 404 in the body (the
        # firmware only reads the message from an HTTP 200, src/api-client/setup.cpp)
        return json(status: 404, api_key: nil, friendly_id: nil, image_url: nil,
                    message: "MAC #{rec.headers['ID']} not registered - send to support@trmnl.com to activate " \
                             "your TRMNL")
      end

      json({ status: 200, api_key:, friendly_id:, image_url: "#{device_url}/images/default.bmp",
             message: "Register at usetrmnl.com/signup with Device ID 'SIMTST'" }.merge(setup.transform_keys(&:to_sym)))
    end

    def display_answer
      d = @lock.synchronize { display_queue.shift || display }.transform_keys(&:to_sym)
      image = d.fetch(:image, "default")
      { status: 0, image_url: device_url + image_path(image), filename: filenames.fetch(image, image),
        refresh_rate: 900, update_firmware: false, firmware_url: nil, reset_firmware: false,
        special_function: "sleep" }.merge(d.except(:image))
    end

    def image_answer(data, ctype) = data ? [200, ctype, data] : [404, "text/plain", "no such image"]
    def json(obj) = [200, "application/json", JSON.generate(obj)]
  end
end
