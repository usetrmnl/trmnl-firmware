# frozen_string_literal: true

require "json"
require "openssl"

module TrmnlSim
  # An independent ATT central. Characteristic handles come from discovery, never guest memory.
  class Bluetooth
    VERSION = "7c3e00014e914f829a6320be63ec51d4"
    SECURITY = "7c3e00024e914f829a6320be63ec51d4"
    CONTROL = "7c3e00034e914f829a6320be63ec51d4"

    class AttError < Error
      attr_reader :packet

      def initialize(packet)
        @packet = packet
        super("ATT error: #{packet.unpack1('H*')}")
      end

      def code = packet.getbyte(4)
    end

    attr_reader :connection, :mtu, :characteristics

    def initialize(sim)
      @sim = sim
      @connection = sim.bluetooth_connect
      services = exchange([0x10, 1, 0xffff, 0x2800].pack("Cvvv"), 0x11)
      raise Error, "no primary services" if services.bytesize < 4

      @mtu = [517, exchange([2, 517].pack("Cv"), 3).unpack1("@1v")].min
      @characteristics = discover
    end

    def exchange(data, opcode)
      response = @sim.bluetooth_att(connection, data)
      raise AttError, response if response.getbyte(0) == 1
      raise Error, "unexpected ATT response #{response.unpack1('H*')}" unless response.getbyte(0) == opcode

      response
    end

    def disconnect = @sim.bluetooth_disconnect(connection)

    def endpoint(uuid, data)
      write(uuid, data)
      address = [characteristics.fetch(uuid)].pack("v")
      response = exchange("\x0a".b + address, 11)
      output = response.byteslice(1..)
      while response.bytesize == mtu
        response = exchange("\x0c".b + address + [output.bytesize].pack("v"), 13)
        output += response.byteslice(1..)
      end
      output
    end

    def write(uuid, data)
      address = [characteristics.fetch(uuid)].pack("v")
      if data.bytesize <= mtu - 3
        exchange("\x12".b + address + data, 0x13)
      else
        data.bytes.each_slice(mtu - 5).with_index do |bytes, index|
          fragment = address + [index * (mtu - 5)].pack("v") + bytes.pack("C*")
          response = exchange("\x16".b + fragment, 0x17)
          raise Error, "prepare-write echo differs" unless response == "\x17".b + fragment
        end
        exchange("\x18\x01".b, 0x19)
      end
    end

    private

    def discover
      start = 1
      found = {}
      while start <= 0xffff
        begin
          response = exchange([8, start, 0xffff, 0x2803].pack("Cvvv"), 9)
        rescue AttError => e
          raise unless e.code == 10

          break
        end
        size = response.getbyte(1)
        raise Error, "invalid characteristic declaration size" unless [7, 21].include?(size)

        response.byteslice(2..).bytes.each_slice(size) do |bytes|
          row = bytes.pack("C*")
          start = row.unpack1("v") + 1
          found[row.byteslice(5..).reverse.unpack1("H*")] = row.unpack1("@3v") if size == 21
        end
      end
      found
    end
  end

  # ESP-IDF session.proto/sec1.proto wire fields, limited to integers and byte strings.
  module ProtoFields
    module_function

    def varint(value)
      bytes = []
      while value > 127
        bytes << ((value & 127) | 128)
        value >>= 7
      end
      (bytes << value).pack("C*")
    end

    def scalar(field, value) = varint(field << 3) + varint(value)
    def blob(field, value) = varint((field << 3) | 2) + varint(value.bytesize) + value

    def decode(data)
      bytes = data.bytes
      result = {}
      until bytes.empty?
        tag = read_varint(bytes)
        case tag & 7
        when 0 then value = read_varint(bytes)
        when 2
          size = read_varint(bytes)
          raise Error, "truncated protobuf bytes" if size > bytes.size

          value = bytes.shift(size).pack("C*")
        else raise Error, "unsupported protobuf wire type"
        end
        raise Error, "duplicate protobuf field" if result.key?(tag >> 3)

        result[tag >> 3] = value
      end
      result
    end

    def read_varint(bytes)
      value = 0
      10.times do |index|
        byte = bytes.shift
        raise Error, "truncated protobuf varint" unless byte

        value |= (byte & 127) << (index * 7)
        return value if byte < 128
      end
      raise Error, "invalid protobuf varint"
    end
  end

  # Security 1 as a phone would perform it: X25519 + proof-of-possession, then a
  # continuous AES-CTR stream for the handshake and all control commands/responses.
  class Security1
    P = ProtoFields
    X25519_SPKI = ["302a300506032b656e032100"].pack("H*").freeze

    def initialize(central, session:, proof:)
      @central = central
      @session = session
      private_key = OpenSSL::PKey.generate_key("X25519")
      public_key = private_key.public_to_der.byteslice(-32, 32)
      hello = exchange(P.blob(20, P.blob(1, public_key)), 1, 21)
      peer = hello.fetch(2)
      iv = hello.fetch(3)
      raise Error, "invalid Security 1 public key or IV" unless peer.bytesize == 32 && iv.bytesize == 16

      shared = private_key.derive(OpenSSL::PKey.read(X25519_SPKI + peer))
      key = shared.bytes.zip(OpenSSL::Digest::SHA256.digest(proof).bytes).map { |a, b| a ^ b }.pack("C*")
      @stream = OpenSSL::Cipher.new("aes-256-ctr").encrypt
      @stream.key = key
      @stream.iv = iv
      verifier = exchange(P.scalar(1, 2) + P.blob(22, P.blob(2, @stream.update(peer))), 3, 23)
      raise Error, "device Security 1 verifier mismatch" unless @stream.update(verifier.fetch(3)) == public_key
    end

    def command(op, attempt: 0, padding: 0, **params)
      plain = JSON.generate({ v: 1, sid: @session, op:, attempt:, **params }) + (" " * padding)
      request(plain)
    end

    def request(plain)
      response = @central.endpoint(Bluetooth::CONTROL, @stream.update(plain))
      result = JSON.parse(@stream.update(response))
      raise Error, "wrong control version/session" unless result["v"] == 1 && result["sid"] == @session

      result
    end

    private

    def exchange(command, message_type, field)
      response = P.decode(@central.endpoint(Bluetooth::SECURITY, P.scalar(2, 1) + P.blob(11, command)))
      raise Error, "wrong security version" unless response[2] == 1

      payload = P.decode(response.fetch(11))
      raise Error, "wrong security message type" unless payload[1] == message_type

      result = P.decode(payload.fetch(field))
      raise Error, "Security 1 handshake rejected" unless result.fetch(1, 0).zero?

      result
    end
  end
end
