# frozen_string_literal: true

require "zlib"

# Reading flash images: the partition table and which app slot the bootloader boots.
module Flash
  # rubocop:disable Lint/StructNewOverride -- the partition table's field names
  Partition = Struct.new(:label, :type, :subtype, :offset, :size, keyword_init: true) do
    def app? = type.zero? && (subtype.zero? || (0x10...0x20).cover?(subtype))

    def ota_slot
      subtype - 0x10 if type.zero? && (0x10...0x20).cover?(subtype)
    end
  end
  # rubocop:enable Lint/StructNewOverride

  module_function

  # The partitions in a flash image (the table at 0x8000).
  def partition_table(flash)
    (0x8000...0x9000).step(32).map { |off| flash.byteslice(off, 32).unpack("vCCVVZ16") }
                     .take_while { |magic, *| magic == 0x50AA }
                     .map do |_, type, subtype, offset, size, label|
      Partition.new(label:, type:, subtype:, offset:,
                    size:)
    end
  end

  # The label of app slot ota_`n` in a flash image's partition table.
  def ota_slot_label(flash, n) = partition_table(flash).find { |p| p.ota_slot == n }.label

  # The label of the app partition the bootloader boots from a flash image, from otadata the
  # way the IDF bootloader reads it: the valid entry with the highest sequence number picks slot
  # (seq - 1) mod the number of slots; none valid, the factory app or ota_0.
  def boot_slot(flash)
    parts = partition_table(flash)
    apps = parts.select(&:ota_slot).sort_by(&:subtype)
    factory = parts.select { |p| p.type.zero? && p.subtype.zero? }
    otadata = parts.find { |p| p.type == 1 && p.subtype.zero? }
    seqs = [0, 0x1000].filter_map do |sector|
      seq, state, crc = flash.byteslice(otadata.offset + sector, 32).unpack("Vx20VV")
      # ESP_OTA_IMG_INVALID (3) / ABORTED (4) entries don't count
      seq if seq != 0xFFFFFFFF && crc == ota_crc(seq) && ![3, 4].include?(state)
    end
    return (factory.empty? ? apps : factory).first.label if seqs.empty?

    apps[(seqs.max - 1) % apps.size].label
  end

  # The IDF's otadata CRC: crc32_le(UINT32_MAX, &seq, 4), which zlib computes as crc32 with
  # 0xFFFFFFFF as its start value.
  def ota_crc(seq) = Zlib.crc32([seq].pack("V"), 0xFFFFFFFF)

  # Where a build's partition table (partitions.bin) puts app slot ota_`n`.
  def ota_offset(build, n = 1)
    table = File.binread(File.join(build, "partitions.bin"))
    part = (0...table.bytesize).step(32).map { |i| table.byteslice(i, 32).unpack("vCCV") }
                               .take_while { |magic, *| magic == 0x50AA }
                               .find { |_, type, subtype, _| type.zero? && subtype == 0x10 + n }
    raise "#{build}/partitions.bin has no ota_#{n} partition" unless part

    part[3]
  end
end
