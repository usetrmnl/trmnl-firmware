# frozen_string_literal: true

# Environment sensors on the device's I2C bus (the TRMNL OG's I2C header; the simulator's
# --sensor): the readings go to the server in the SENSORS header of /api/display. Only boards
# with sensor pins in the firmware's device_list[] (Device#sensors) look for them.

General.describe "Sensors" do
  fixture(:dev) { ProvisionedDevice.new(build) }

  describe "Sensors" do
    before { dev.reset }

    # The SENSORS header of a timer wake's /api/display with `sensors` attached ("" without one).
    def sensor_header(*sensors)
      dev.boot_asleep(extra_args: sensors.flat_map { ["--sensor", _1] }) do |s|
        s.wait_for_deep_sleep(timeout: 15)
        req = dev.mock.next_request("/api/display", timeout: 15) { s.wake }
        s.wait_for_deep_sleep(timeout: 15)
        req.headers["SENSORS"].to_s
      end
    end

    it "scd41 reports co2 temperature and humidity", needs: :sensors do
      expect(sensor_header("scd41")).to include("model=SCD41;kind=carbon_dioxide;value=812;unit=parts_per_million",
                                                "model=SCD41;kind=temperature;value=22.4",
                                                "model=SCD41;kind=humidity;value=44;unit=percent")
    end

    it "aht20 reports temperature and humidity", needs: :sensors do
      expect(sensor_header("aht20")).to include("make=ASAIR;model=AHT20;kind=temperature;value=22.4",
                                                "model=AHT20;kind=humidity;value=44;unit=percent")
    end

    it "reports both sensors", needs: :sensors do
      expect(sensor_header("scd41", "aht20")).to include("model=SCD41;kind=carbon_dioxide",
                                                         "model=AHT20;kind=temperature")
    end

    it "no sensor no header" do
      expect(sensor_header).to eq("")
    end
  end
end
