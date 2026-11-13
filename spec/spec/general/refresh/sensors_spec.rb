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

  # The SCD41's 5 s sample wait in EnvironmentSensor::init (src/misc/sensor/environment_sensor.cpp)
  # is a light sleep, which replaces the wake cause with the timer; bl_init (src/bl.cpp) has to
  # read the wake cause and the button before it.
  describe "ButtonWithScd41" do
    before { dev.reset }

    it "a press sends a button wake", needs: %i[sensors button] do
      dev.boot_asleep(extra_args: ["--sensor", "scd41"]) do |s|
        s.wait_for_deep_sleep(timeout: 15)
        req = dev.mock.next_request("/api/display", timeout: 30) { s.press(150) }
        expect(req).to have_header("Update-Source", device.button_source)
        expect(req.headers["SENSORS"].to_s).to include("model=SCD41;kind=carbon_dioxide")
        s.wait_for_deep_sleep(timeout: 15)
      end
    end

    it "a long press resets WiFi", needs: %i[sensors button] do
      dev.boot_asleep(extra_args: ["--sensor", "scd41"]) do |s|
        s.wait_for_deep_sleep(timeout: 15)
        s.press(6000)
        s.wait(portal: true, timeout: 60)
      end
    end

    it "a double click runs the special function", needs: %i[sensors double_click] do
      dev.boot_asleep(extra_args: ["--sensor", "scd41"]) do |s|
        s.wait_for_deep_sleep(timeout: 15)
        dev.mock.display = { image: "default", refresh_rate: 300, special_function: "sleep" }
        dev.mock.next_request("/api/display", timeout: 30) { s.wake }
        s.wait_for_deep_sleep(timeout: 30)
        dev.mock.display = { image: "default", action: "sleep", refresh_rate: 1800 }
        req = dev.mock.next_request("/api/display", timeout: 30) { s.double_click(ms: 80, gap_ms: 150) }
        expect(req).to have_header("special_function", "true")
        s.wait_for_deep_sleep(timeout: 30)
      end
    end

    # getDeviceStatusStamp (src/bl.cpp) stamps each stored log with the wake cause bl_init read
    it "logs a button wake as a button wake", needs: %i[sensors button] do
      dev.boot_asleep(extra_args: ["--sensor", "scd41"]) do |s|
        s.wait_for_deep_sleep(timeout: 15)
        dev.mock.set_fault("/api/display", status: 500)
        log = dev.mock.next_request("/api/log", timeout: 90) { s.press(150) }
        expect(log.body).to include(%("wake_reason":"#{device.button_source}"))
        s.wait_for_deep_sleep(timeout: 60)
      end
    end
  end
end
