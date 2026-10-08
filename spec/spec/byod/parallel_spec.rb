# frozen_string_literal: true

# BYOD boards with a 16-gray parallel e-paper panel driven by FastEPD (the firmware's
# PARALLEL_EPD builds other than the TRMNL X): the M5Stack PaperS3 and the LilyGo T5 4.7" S3 Pro,
# both with the 4.7" ED047TC1 960x540 panel, and the Sensoria C5 (an ESP32-C5 with 8 MB PSRAM
# feeding a 1280x720 panel over PARLIO).

images = TrmnlSim::Images

# The 4.7" ED047TC1.
ed047 = { size: [960, 540], inks: "gray16" }
# 16 vertical bands, black at the left, across a panel `w` wide.
ramp16 = ->(w) { ->(x, _y) { [15, x * 16 / w].min } }
# A blank 4x120 px strip: the margin either side of centred text.
paper = images.expected_gray(->(_x, _y) { 1 }, 4, 120)

RSpec.describe "BYOD parallel boards" do
  shared_examples "a parallel board" do
    it "shows 16 grays" do
      w, h = board.size
      expected = dev.mock.set_png("ramp", ramp16.(w), width: w, height: h, bits: 4)
      dev.mock.display = { image: "ramp", refresh_rate: 300 }
      dev.boot_asleep do |s|
        wait_until_asleep(s)
        refresh(s, "/images/ramp.png")
        expect(s).to show_image(expected, tolerance: 40, max_ratio: 0.001)
      end
    end

    it "setup screen text fits the panel" do
      sim(Builds.for_env(board.env), erase: true, extra_args: ["--offline"]) do |s|
        s.wait(portal: true, timeout: 90)
        s.wait(display_idle: true, timeout: 30)
        aggregate_failures do
          [0, board.width - 4].each do |x| # 4 px margins on both sides of the instructions
            expect(s).to show_image(paper, region: [x, 350, 4, 120], tolerance: 16, max_ratio: 0)
          end
        end
      end
    end

    it "reports the battery it measures" do
      dev.boot_asleep do |s|
        wait_until_asleep(s)
        s.set_battery(3700)
        expect(wake_request(s)).to have_header("Battery-Voltage", a_value_within(0.06).of(3.7))
        wait_until_asleep(s, timeout: 90)
      end
    end
  end

  describe "M5PaperS3", env: "TRMNL_X_PAPERS3" do
    byod_board name: "M5Stack PaperS3", model: "m5_papers3", **ed047
    it_behaves_like "a parallel board"

    it "has no wake button" do
      # device_list[] has interrupt_pin 0xff: only the timer wakes it
      dev.boot_asleep do |s|
        expect(s.status["board"]["has_button"]).to be_falsey
        expect(wait_until_asleep(s)["state"]).to eq("deep_sleep")
      end
    end
  end

  describe "LilyGoT5Pro", env: "TRMNL_X_LILYGO_T5PRO" do
    byod_board name: "LilyGo T5 4.7\" S3 Pro", model: "lilygo_t5pro", **ed047
    it_behaves_like "a parallel board"

    it "button wakes it" do
      dev.boot_asleep do |s|
        wait_until_asleep(s)
        dev.mock.next_request("/api/display", timeout: 60) do
          s.press(200)
          s.wait_for_console(/woken by button/, timeout: 30)
        end
        wait_until_asleep(s, timeout: 90)
      end
    end
  end

  # The Sensoria C5 can't be provisioned like the other boards (onboarding, then deep sleep): it
  # never sleeps (see sensoria_sleep above).
  describe "SensoriaC5", env: "TRMNL_X_SENSORIAC5" do
    let(:size) { [1280, 720] }
    let(:c5_build) { Builds.for_env("TRMNL_X_SENSORIAC5") }

    # A factory-fresh Sensoria C5, with a block.
    def fresh_c5(&) = sim(c5_build, erase: true, extra_args: ["--offline"], &)

    # Onboard a fresh device against `mock` (serving a 16-gray ramp); returns the /api/display
    # request once the image is on the screen.
    def onboard(sim, mock)
      w, h = size
      mock.set_png("ramp", ->(x, _y) { [15, x * 16 / w].min }, width: w, height: h, bits: 4)
      mock.display = { image: "ramp", refresh_rate: 300 }
      sim.wait(portal: true, timeout: 90)
      sim.portal_connect("TRMNL-Sim", "password", server: mock.device_url)
      req = mock.wait_for_request("/api/display", timeout: 120)
      sim.wait_for_console(/BL done, going to sleep/, timeout: 120)
      req
    end

    it "setup screen fits the panel" do
      fresh_c5 do |s|
        expect(s.status["board"]["name"]).to eq("Sensoria C5")
        s.wait(portal: true, timeout: 90)
        s.wait(display_idle: true, timeout: 30)
        aggregate_failures do
          [0, size[0] - 4].each do |x|
            expect(s).to show_image(paper, region: [x, 520, 4, 120], tolerance: 16, max_ratio: 0)
          end
        end
      end
    end

    it "onboards and shows 16 grays" do
      w, h = size
      TrmnlSim::MockTrmnl.open do |mock|
        fresh_c5 do |s|
          req = onboard(s, mock)
          expect(req).to have_header("Model", "sensoria_c5")
          expect(req.headers.values_at("Width", "Height").map(&:to_i)).to eq(size)
          expect(req).to have_header("Battery-Voltage", 0.0) # batt_pin 0xff
          expected = images.expected_gray(ramp16.(w), w, h, bits: 4)
          expect(s).to show_image(expected, tolerance: 40, max_ratio: 0.001)
        end
      end
    end

    it "sleeps after a refresh" do
      TrmnlSim::MockTrmnl.open do |mock|
        fresh_c5 do |s|
          onboard(s, mock)
          boots = s.status["boot_count"]
          s.wait(state: "deep_sleep", timeout: 30)
          expect(s.status["boot_count"]).to eq(boots)
        end
      end
    end
  end
end
