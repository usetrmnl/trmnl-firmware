# frozen_string_literal: true

# HTTPS on the device under test (see devices.rb) against the mock server with TLS (ECDHE-ECDSA,
# P-384): plain WiFiClientSecure for other servers, and for trmnl.app the resumable client that
# keeps its TLS session in RTC memory across deep sleep.

General.describe "HTTPS" do
  # A fresh device joined to `mock` through the portal; yields it asleep after its first refresh.
  def onboard(mock, *extra_args)
    sim(erase: true, extra_args: ["--offline", *extra_args]) do |s|
      s.wait(portal: true, timeout: 40)
      s.portal_connect("TRMNL-Sim", "any-password", server: mock.device_url)
      mock.wait_for_request("/api/display", timeout: 40)
      s.wait(state: "deep_sleep", timeout: 30)
      yield s
    end
  end

  # Wake the device (by its timer, or `wake`) and return the requests of that refresh.
  def refresh(s, mock, wake: s.method(:wake))
    n = mock.cursor
    mock.next_request("/api/display", timeout: 30) { wake.() }
    s.wait(state: "deep_sleep", timeout: 30)
    mock.requests.drop(n)
  end

  describe "OtherServer" do
    it "onboarding and refreshes over https", :smoke do
      device_mock(tls: true) do |mock|
        onboard(mock) do |s|
          # the setup logo (always the mock's BMP), then the default screen
          expect(mock.paths).to eq(["/api/setup", "/images/default.bmp", "/api/display", mock.image_path("default")])
          expect(mock.requests[2]).to have_header("Access-Token", mock.api_key)
          # a full handshake every time: only trmnl.app sessions are resumed
          expect(refresh(s, mock).map(&:tls_resumed).uniq).to eq([false])
        end
      end
    end
  end

  describe "TrmnlApp" do
    it "tls session is resumed across deep sleep" do
      device_mock(tls: true) do |mock|
        mock.device_host = "trmnl.app"
        onboard(mock, "--dns", "trmnl.app=10.0.2.2") do |s|
          # /api/setup and the setup logo use a plain client; /api/display's resumable client
          # starts the session the later requests resume
          expect(mock.requests.map { [_1.path, _1.tls_resumed] })
            .to eq([["/api/setup", false], ["/images/default.bmp", false], ["/api/display", false],
                    [mock.image_path("default"), true]])
          expect(refresh(s, mock).map(&:tls_resumed).uniq).to eq([true])
          # a power cycle loses RTC memory, and with it the session: full handshake
          first = refresh(s, mock, wake: s.method(:power_cycle)).first
          expect([first.path, first.tls_resumed]).to eq(["/api/display", false])
          expect(refresh(s, mock).map(&:tls_resumed).uniq).to eq([true])
        end
      end
    end
  end
end
