# Integration tests

Integration tests for the [TRMNL firmware](https://github.com/usetrmnl/trmnl-firmware): real,
unmodified builds boot in [trmnl-sim](https://github.com/usetrmnl/trmnl-sim) against mock TRMNL
servers, and RSpec checks what they send, show and do, on the TRMNL devices and the BYOD
boards.

![setup screen as rendered by the simulator](golden/setup_screen.png)

## Setup

Ruby 3.2+ with Bundler. The suite lives in the firmware's `spec/` and runs its builds; the
simulator is a sibling checkout of the firmware, or a release binary from
[trmnl-sim releases](https://github.com/usetrmnl/trmnl-sim/releases) (override with
`FIRMWARE_REPO`, `SIM_REPO` or `SIM_BIN`):

```
trmnl/
├─ trmnl-firmware/   pio run -e <env> builds .pio/build/<env>
│  └─ spec/          this suite (run its commands from here)
└─ trmnl-sim/        cargo build --release (or rake sim:build)
```

```sh
bundle install
rake sim:build                  # build ../../trmnl-sim
rake "firmware:build[trmnl]"    # pio run -e trmnl in the firmware checkout (default: the core devices)
rake check                      # rubocop, and every spec loads for every device (no firmware needed)
```

## Running

```sh
# parallel processes:
ENVS="trmnl:full trmnl_x" rake spec                   # rake sim:build, then parallel_rspec
rake "spec[-n 4 spec/core]"                           # arguments pass through to parallel_rspec

bundle exec parallel_rspec                            # a process per CPU
ENVS="xteink_x4:full" bundle exec parallel_rspec      # everything for one device

# one process:
bundle exec rspec                                     # default ENVS
bundle exec rspec spec/general/setup/portal_spec.rb:42
```

`ENVS` lists PlatformIO environments or families (`core`: the TRMNL-branded devices; `byod`:
every other board; `all`), each optionally `:full` or `:smoke`. A listed device runs its own
specs plus the general `:smoke` examples (one per area); `:full` runs every general example on
it too, and `:smoke` only its `:smoke` examples. The default is
`trmnl:full TRMNL_X trmnl_4clr trmnl_gen2 trmnl_gen2_4clr`. Unlisted devices and missing builds
are skipped (missing builds fail under CI).

Specs live in [spec/general](spec/general) (every device, by area),
[spec/core](spec/core) and [spec/byod](spec/byod). Onboarded devices are cached in
`tmp/spec-cache/`, keyed by firmware, simulator and support code, so the first run after a
change is slower. Every simulator's log and final screen are saved in `out/artifacts/`
(gitignored), replacing the previous run's. `rake -T` lists the tasks (quote them in zsh).

## Environment variables

| Env var | |
|---|---|
| `ENVS` | Devices a run covers (see above) |
| `FIRMWARE_REPO` | Firmware checkout; builds are in its `.pio/build/<env>` (default: the checkout this suite is in) |
| `SIM_REPO` | trmnl-sim checkout (default `../trmnl-sim` next to the firmware checkout) |
| `SIM_BIN` | Simulator binary (default: the checkout's `target/release/trmnl-sim`) |
| `SLOW=1` | Also run examples marked `slow:` |
| `NO_CACHE=1` | Run every setup flow in full, as CI does |
| `REALTIME=1` | Run without turbo |
| `UPDATE_GOLDEN=1` | Rewrite golden screenshots |

## Code coverage

Every run records firmware coverage and, when it ends, writes it to `out/cov/merged.info` and
`out/cov/html/` (gitignored) and prints the totals. Only the firmware's own code is reported
(`src/` and TRMNL's libraries in `lib/`), not the framework, `.pio/libdeps` or the vendored
drivers in `lib/` (see `FIRMWARE_SOURCES` in [coverage_report.rb](spec/support/coverage_report.rb)). Cached
setup flows keep their coverage, so a cached run reports the same lines as a full one.
`rake coverage` rebuilds the HTML report from the last run's `out/cov/merged.info` and prints
the summary again.

## Memory checking

Every simulator runs with `--memcheck=halt` (see the trmnl-sim README, "Memory checking"): any
memory error fails its example. [memcheck_spec.rb](spec/general/tooling/memcheck_spec.rb) also
checks the IDF tasks' stack headroom and a few paths the other specs don't take.

## Writing tests

Each spec file has one top-level group declaring the environment it runs (`env: "<env>"`, or
`General.describe` for one group per listed device) and a nested group per scenario:

```ruby
General.describe "Refresh cycle" do
  fixture(:dev) { ProvisionedDevice.new(build) }           # onboarded once (cached), shared
  before { dev.reset }

  describe "RefreshCycle" do
    it "fetches the next image on a timer wake", :smoke do
      _, expected = device_image(dev.mock, "two", device_number("2"))
      dev.mock.display = { image: "two", refresh_rate: 300 }
      dev.boot_asleep do |s|                               # resumed from a save point
        req = dev.mock.next_request("/api/display") { s.wake }
        expect(req).to have_header("Update-Source", "timer")
        s.wait(state: "deep_sleep", display_idle: true, timeout: 120)
        expect(s).to show_image(expected, tolerance: 64)
      end
    end

    it "resets WiFi on a long press", needs: :button do
      ...
    end
  end
end
```

Metadata ([metadata.rb](spec/support/metadata.rb)): `env:`, `needs:` (a `Device` feature),
`only_on:` / `skip_if:` with `why:`, `needs_build:`, `slow:`, `:smoke`. There are no expected
failures: an example the firmware gets wrong fails (`known_failure:` and `pending:` are
refused). Helpers and matchers (`device_image`, `show_image`,
`match_golden`, `have_header`, ...) are in [spec/support](spec/support).

The client library in [lib](lib) (standard library only):

- **`TrmnlSim::Simulator`** drives a headless simulator over its control API: `wait(...)`
  on console output, state, refreshes, WiFi or portal; `press(ms)` / `touch(...)` in exact
  virtual time; `save_point` / `restore`; `set_faults`; `memcheck`.
- **`TrmnlSim::MockTrmnl`** is a fake TRMNL server (HTTP or TLS) that serves
  `/api/setup`, `/api/display`, images and firmware, records every request, generates BMP
  and PNG images in each device's format (returning the expected screen), and injects HTTP
  faults with `set_fault`.
- **`ProvisionedDevice`** ([provisioned_device.rb](spec/support/provisioned_device.rb)) and
  `TrmnlX::ShippedX` / `ProvisionedX` ([trmnl_x.rb](spec/support/trmnl_x.rb)) onboard once
  and boot copies, so tests start from a registered device in seconds.

See the trmnl-sim README for the control API, fault injection and memcheck details.

## CI

The firmware's [integration.yml](../.github/workflows/integration.yml) runs `rake check`, then
one job each for `trmnl` (`:full`), `TRMNL_X` and `trmnl_4clr` (BWRY): it builds the env and
runs `rake spec` uncached on it with the latest headless
[trmnl-sim release](https://github.com/usetrmnl/trmnl-sim/releases), uploading logs, screens
and diffs on failure and coverage always.
