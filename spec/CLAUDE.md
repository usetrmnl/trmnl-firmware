# Integration tests (spec/)

Integration tests for the TRMNL firmware: real builds of this firmware
(`../.pio/build/<env>`) run in the simulator (`../../trmnl-sim`, `target/release/trmnl-sim`,
or `SIM_BIN`) against mock TRMNL servers, checked with RSpec. README.md is the full reference
(running, `ENVS`, what is covered, environment variables, writing tests); the simulator itself
(its control API, fault injection, memcheck, coverage) is documented in the trmnl-sim README.

## Rules

- Run `rake check` (rubocop, and every spec loads for every device) before committing.
- Never run `pio` unless asked (`rake firmware:build` runs it); the user builds firmware. The
  firmware (`../src`, `../lib`, `../.pio/libdeps/<env>/`) and the IDF/Arduino sources under
  `~/.platformio/packages` are the reference for what the hardware must do. Never run
  `pio run -c <other.ini>` in the firmware checkout: a different project config makes
  PlatformIO wipe every env's `.pio/build`.
- A simulator bug is fixed in ../../trmnl-sim (with its own rules, see its CLAUDE.md), not
  worked around here.

## Running

- Plain RSpec: `bundle exec rspec [path[:line]] [-e ...]`, or `bundle exec parallel_rspec`
  (`rake spec` builds the simulator first, then runs it). `ENVS`
  (spec/support/selection.rb) lists the devices a run covers: PlatformIO environments or
  `core` / `byod` / `all`, each optionally `:full` or `:smoke`. A listed device runs its own
  specs and the general `:smoke` examples; `:full` runs every general example on it too,
  `:smoke` only its `:smoke` examples. Default
  `trmnl:full TRMNL_X trmnl_4clr trmnl_gen2 trmnl_gen2_4clr`; unlisted devices and missing
  builds are left out (a new example without `:smoke` needs its device listed with `:full`).
  `SLOW=1`, `NO_CACHE=1` as needed.
- While iterating, run ONE group or example at a time under a hard limit and clean up:
  `perl -e 'alarm 90; exec @ARGV' bundle exec rspec spec/general/setup/portal_spec.rb:42; pkill -f target/release/trmnl-sim`
  (`ENVS=<env>` or `ENVS=<env>:full` for one device). No full-suite runs for debugging; a
  full run is for a final regression check (run it in the background).
- Onboarded devices are cached in `tmp/spec-cache/` keyed by firmware, simulator and test
  support code, so the first run after a change is slower.

## Writing tests

- `spec/support/devices.rb` has a `Device` profile per environment (size, inks, chip,
  battery, button, core or BYOD, ...). Every group declares the environment it runs:
  `env: "<env>"` (device specs: spec/core/ for the TRMNL-branded devices, spec/byod/ for the
  rest), or `General.describe` for general specs (spec/general/), which is defined once per
  listed device and must adapt to its `Device` (`device` / `build` in the group; use
  `device_image`, `needs:` / `only_on:` metadata, `match_golden`). Metadata is documented in
  `spec/support/metadata.rb`; the client library (`TrmnlSim::Simulator`, `MockTrmnl`,
  `Images`, `Lcov`) is in `lib/trmnl_sim/`.
- A test that fails because the firmware is wrong stays in and fails: no `known_failure:`,
  `pending:`, skips, memcheck suppressions or expectations bent to match the bug. Verify the
  root cause in the firmware source first. Never weaken an assertion or work around a
  firmware bug.
- A test that fails because the simulator is wrong gets the simulator fixed in ../../trmnl-sim:
  general, minimal changes that keep the other chips and boards behaving identically.
- Golden screenshots: per-device ones live in `golden/<env>/`. Look at every new or
  rewritten golden (open the PNG) before committing it; wrong-looking output is a firmware
  bug to record, not a golden.
- Style: rubocop (`.rubocop.yml`: double quotes, 120 columns). Match the surrounding code's
  comment density and naming; comments explain device behaviour and why, with firmware
  file:line references where relevant.

## Adding a board

After the board exists in the simulator (a `BoardSpec` row in ../../trmnl-sim): add a `Device`
in `spec/support/devices.rb` and a board group in spec/byod/ using the BYOD support
(`spec/support/byod.rb`); then run `ENVS=<env>:full bundle exec parallel_rspec` and classify
every failure as above. Update the README's coverage notes.
