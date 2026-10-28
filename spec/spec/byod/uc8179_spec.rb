# frozen_string_literal: true

# BYOD boards with the UC8179 panels the TRMNL OG family uses: 7.5" black and white and 7.3"
# Spectra 6, on XIAO ESP32-C3/S3 boards, the TRMNL DIY kits and the reTerminal E1001.

RSpec.describe "BYOD UC8179 boards" do
  describe "XiaoEsp32c3", env: "seeed_xiao_esp32c3" do
    byod_board name: "XIAO ESP32-C3 + 7.5\" panel", model: "seeed_esp32c3",
               battery_v: 0.0 # batt_pin 0xff: nothing to read
  end

  describe "XiaoEsp32s3", env: "seeed_xiao_esp32s3" do
    # main's seeed_xiao_esp32s3 env lacks `framework = arduino`; build it with that added
    byod_board name: "XIAO ESP32-S3 + 7.5\" panel", model: "seeed_esp32s3", battery_v: 0.0
  end

  describe "DiyKit75", env: "TRMNL_7inch5_OG_DIY_Kit" do
    byod_board name: "TRMNL 7.5\" DIY Kit", model: "xiao_epaper_display"
  end

  describe "DiyKitSpectra6", env: "TRMNL_7inch5_OG_DIY_Kit_6CLR" do
    byod_board name: "TRMNL 7.3\" Spectra 6 DIY Kit", model: "xiao_epaper_6clr", inks: "spectra6"
  end

  describe "ReTerminalE1001", env: "seeed_reTerminal_E1001" do
    byod_board name: "reTerminal E1001", model: "reterminal_e1001"
  end
end
