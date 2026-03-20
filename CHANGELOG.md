# Changelog

All notable changes to this project are documented in this file.

## 0.1.8 - 2026-03-20

- Added the next clean-core parity wrappers for SPIS, QDEC, and I2S plus the
  required register definitions and CMSIS compatibility headers for the Seeed
  Zephyr core.
- Copied the next clean-core peripheral examples into the packaged Zephyr core:
  `I2sTxWrapperInterrupt`, `I2sRxWrapperInterrupt`,
  `I2sDuplexWrapperInterrupt`, `RawI2sTxInterrupt`,
  `QdecRotaryReporter`, and `SpisTargetEcho`.
- Enabled Zephyr dynamic interrupts in the packaged base config and routed
  `I2S20_IRQn` through a shared Zephyr-owned connection for both the wrapper
  helpers and the raw I2S example, fixing the previous
  `Unhandled IRQn: 221` runtime failure after upload.
- Verified the new I2S paths on hardware with forced `pyocd` upload/reset:
  `RawI2sTxInterrupt`, `I2sTxWrapperInterrupt`, and
  `I2sDuplexWrapperInterrupt` all boot and advance their expected IRQ counters.

## 0.1.7 - 2026-03-19

- Added bundled clean-compat integration into the Seeed Zephyr core so
  `#include <nrf54l15_hal.h>` works both from packaged examples and from user
  sketches without relying on Arduino library auto-discovery.
- Added the `Nrf54L15-Clean-Implementation` parity wrappers for DPPIC, COMP,
  and LPCOMP plus the required register definitions for the Seeed Zephyr core.
- Copied the next clean-core parity examples into the packaged Zephyr core:
  `CompDifferentialProbe`, `CompThresholdMonitor`, `DppicHardwareBlink`, and
  `LpcompSystemOffWake`.
- Added a software fallback path inside `DppicHardwareBlink` so the example
  still runs on the current Zephyr shim even though hardware publish/subscribe
  routing is not exposed yet.
- Updated parity documentation and release packaging so the copied clean-core
  library examples compile directly from their installed Arduino IDE paths.

## 0.1.6 - 2026-03-03

- Added missing Arduino compatibility macros and aliases in `Arduino.h`:
  `min`, `max`, `bit`, `_BV`, `PROGMEM`, and `PSTR`.
- Added the 2-argument compatibility overloads for `pulseIn()` and
  `pulseInLong()`.
- Expanded `WString` API compatibility with base-aware numeric constructors,
  `toUpperCase()`, `indexOf()` overloads, `substring()`, and `toCharArray()`.
- Added AVR compatibility header `cores/nrf54l15/avr/pgmspace.h` with common
  `*_P` memory/string helpers and `pgm_read_*` macros.
- Added serial and I2C compatibility aliases (`Serial2`, `Wire1`) and variant
  metadata (`PIN_SERIAL2_TX/RX`, `SERIAL_PORT_HARDWARE2`, `HAVE_HWSERIAL2`)
  to improve third-party library compatibility.

## 0.1.5 - 2026-02-23

- Added a dedicated Arduino board profile:
  `xiao_nrf54l15_sense` / `XIAO nRF54L15 Sense (Zephyr-Based)`.
- Added Sense-specific Zephyr fragment
  `zephyr/arduino_base/conf/sense_features.conf` to enable IMU driver support
  (`CONFIG_SENSOR` + `CONFIG_LSM6DSL`) when Sense board profile is selected.
- Extended prebuild Zephyr hook wiring to accept Sense-specific Kconfig/overlay
  arguments from board properties.
- Added curated Sense examples:
  `examples/Sense/IMUOrientation` and `examples/Sense/MicrophoneLevel`.
- Refactored Sense examples so `.ino` files stay Arduino-only and Zephyr
  headers/devicetree logic live in local `.cpp` backends, fixing Arduino
  preprocessor compatibility on fresh installs.
- Removed outdated `examples/Expansion/*` sketches from packaged core examples
  to keep Arduino IDE categories focused on board-core features.
- Updated release docs, example category docs, CI smoke checks, and top-level
  README to include Sense board support and examples.

## 0.1.4 - 2026-02-23

- Added explicit RF switch chip power control API in `XiaoNrf54L15`:
  `setRfSwitchEnabled()` / `rfSwitchEnabled()`.
- Added variant-level C wrappers for RF switch power (`RF_SW_PWR` on P2.03)
  and kept antenna select control on `RF_SW_SEL` (P2.05).
- Added new board-specific curated examples:
  `examples/Power/BatteryMeasure` and `examples/Radio/RFChipDisable`.
- Updated fresh-machine smoke coverage and release documentation to include
  the new battery and RF-control examples.

## 0.1.3 - 2026-02-23

- Fixed Zephyr SDK bootstrap corruption handling in `tools/get_toolchain.py`:
  `_lzma.LZMAError` is now treated as a retryable extraction failure.
- Added archive-size guard rails in toolchain bootstrap so oversized cached
  downloads are deleted and re-fetched cleanly before extraction.
- Applied the same download/extract hardening to both packaged script trees:
  `hardware/nrf54l15/nrf54l15/tools/get_toolchain.py` and
  `hardware/seeed/nrf54l15/tools/get_toolchain.py`.

## 0.1.2 - 2026-02-23

- Curated packaged Arduino IDE examples to board-specific categories under
  `examples/` (`BLE`, `Radio`, `Power`, `GPIO`, `Expansion`) and removed
  generic Arduino-style Blink/Basic/Analog/Communication example sets.
- Updated Boards Manager fresh-machine smoke coverage and release docs to the
  curated example paths.
- Added compatibility APIs and board helpers in `XiaoNrf54L15` for antenna,
  radio profile, BLE TX power reporting, and battery measurement control.
- Added VBAT control helpers in variant init and updated battery example for
  `VBAT_EN` + `A7` measurement path.
- Hardened `tools/get_toolchain.py` download flow to recover from stale resume
  state (`HTTP 416`) by retrying with a clean archive download.

## 0.1.1 - 2026-02-15

- Added legacy Seeed example sets from `hardware/seeed/nrf54l15/examples`
  into the packaged core example tree at
  `hardware/nrf54l15/nrf54l15/examples` so they appear in Arduino IDE for
  Boards Manager installs.

## 0.1.0 - 2026-02-14

- Reset release line to `0.1.0` and consolidate published artifacts to the
  current working Zephyr-based implementation.
- Updated user-facing naming from "Bare-Metal" to "Zephyr-Based" across
  Boards Manager metadata and documentation.

- Reworked `tools/install/windows_prereqs.bat` to provision a full first-run
  Windows setup by default: dependency install (Python/Git), core copy to
  sketchbook, NCS workspace bootstrap, Zephyr SDK bootstrap, and one-time
  Zephyr warmup build.
- Added automatic CMake dependency provisioning to
  `tools/install/windows_prereqs.bat` to prevent `west zephyr-export` failures
  on fresh Windows hosts.
- Added clearer step-by-step progress logging in the Windows installer.
- Updated Windows core copy behavior to preserve existing `tools/ncs` and
  `tools/zephyr-sdk` caches on rerun, avoiding repeated bootstrap downloads.
- Fixed `NameError: log is not defined` in `tools/get_toolchain.py`
  `.7z` extraction fallback path.
- Changed `KEEP_ZEPHYR_SDK_ARCHIVE` default to keep SDK archive (`1`) so
  interrupted/failed setups can resume without re-downloading.
- Added Windows installer flags `--skip-core-copy`, `--skip-bootstrap`,
  and `--help`.
- Added self-healing west Python dependency bootstrap in `tools/get_nrf_connect.py`:
  if bundled `pydeps` is missing `west`/`colorama` dependencies on fresh hosts,
  the script now installs fallback packages into `tools/pydeps` automatically.
- Added automatic Zephyr Python build dependency bootstrap
  (`zephyr/scripts/requirements-base.txt`) into `tools/pydeps` to prevent
  missing-module failures (for example `jsonschema`) during first warmup builds.
- Fixed Windows warmup build path-length failures by switching default Zephyr
  build output to a shorter path under `%LOCALAPPDATA%\nrf54l15-build`
  (override with `ARDUINO_ZEPHYR_BUILD_DIR` / `ARDUINO_NRF54L15_BUILD_ROOT`).
- Updated toolchain bootstrap to accept a host `dtc` already on PATH, avoiding
  unnecessary Zephyr SDK host-tools setup attempts on Windows reruns.
- Added `tools/export_sync_bundle.py` to generate a clean sync folder (and
  optional zip) without local SDK/cache payloads for manual GitHub uploads.
- Improved `tools/get_toolchain.py` API-rate-limit fallback by synthesizing
  deterministic Zephyr SDK/host-tools asset URLs for the detected host when
  GitHub API metadata is unavailable.

## 1.1.9 - 2026-02-14

- Hardened Zephyr SDK asset resolution in `tools/get_toolchain.py`:
  if GitHub API lookup is rate-limited/unavailable, the script now falls back to
  parsing release-page asset links instead of failing immediately.
- Disabled `sysroots` pruning by default (can be re-enabled with
  `PRUNE_ZEPHYR_SDK_SYSROOTS=1`) to avoid missing C++ stdlib headers on Windows hosts.
- Retained multiarch-prune support for optional SDK size reduction.

## 1.1.8 - 2026-02-14

- Fixed clean-install dependency-discovery failures before `zephyr_lib` is generated:
  Zephyr header includes in Bluetooth/XiaoNrf54L15/IEEE802154 library sources are now
  guarded with `__has_include(...)` so Arduino preprocessor discovery can run before
  first Zephyr bootstrap/build.
- This allows BLE sketches to compile first on a fresh install without requiring a
  prior Blink/default build.

## 1.1.7 - 2026-02-14

- Fixed fresh-install BLE compile regression:
  `libraries/Bluetooth/src/Bluetooth.h` now includes `<zephyr/types.h>`
  only when available, so BLE sketches can compile on first run before
  generated Zephyr headers are populated.
- Kept 1.1.6 workspace/bootstrap lock hardening intact.

## 1.1.6 - 2026-02-14

- Added cross-process build lock in `build_zephyr_lib.py` to avoid concurrent `west build`
  collisions on the same Zephyr build directory.
- Added automatic one-shot workspace integrity recovery retry when `west build` fails and
  the shared NCS workspace is detected as incomplete mid-run.

## 1.1.5 - 2026-02-14

- Fixed recovery from partially initialized shared NCS workspaces:
  `build_zephyr_lib.py` now validates workspace integrity (`.west/config`,
  `nrf/west.yml`, and Zephyr west command directory) and re-runs bootstrap when incomplete.
  This prevents `west: unknown command "build"` errors after interrupted first-time setup.
- Added official Arm GNU Toolchain downloads link in docs as an optional manual fallback reference.

## 1.1.4 - 2026-02-14

- Added shared bootstrap cache handling for Boards Manager installs:
  NCS/Zephyr SDK payloads now live under the package vendor `tools/` directory,
  so changing board options or updating core versions does not trigger full re-downloads.
- Added cross-platform toolchain launcher wrappers (`toolchain-*` + `.cmd`) and
  `toolchain_exec.py` so first compile on a clean machine can bootstrap the Zephyr SDK
  before compiler invocation instead of failing with missing `arm-none-eabi-*` tools.
- Updated installed-core discovery to include `packages/nrf54l15/...` layout in
  addition to legacy vendor roots.
- Expanded CI fresh-machine matrix to include `macos-latest`.
- Updated documentation with cache behavior and `ARDUINO_NRF54L15_SHARED_TOOLS_DIR`.

## 1.1.3 - 2026-02-14

- Fixed `Tools > Radio Profile > BLE Only` build failure on nRF54L15:
  BLE profile now enforces 128 MHz CPU and applies the 128 MHz overlay, matching MPSL requirements.
- Updated docs to clarify that BLE radio profiles use a 128 MHz radio-safe clock configuration.

## 1.1.2 - 2026-02-14

- Added Linux host-tools fallback in `tools/get_toolchain.py`:
  when `dtc` is missing, the script now fetches `hosttools_linux-<arch>.tar.xz`,
  extracts the standalone installer into the SDK directory, and retries setup automatically.
- Kept host-tools standalone installer files during prune to preserve recovery path for subsequent runs.

## 1.1.1 - 2026-02-14

- Hardened `tools/get_toolchain.py` download flow for unstable networks:
  automatic retry on truncated/corrupt archives, size validation, and resume-capable downloads via `curl`/`wget`.
- Added host-tool (`dtc`) checks in SDK bootstrap path and clearer fallback messaging when host tools are not bundled.
- Increased default SDK download retry count (`ZEPHYR_SDK_DOWNLOAD_RETRIES`, default now `10`).

## 1.1.0 - 2026-02-14

- Added `examples/03.Board/HardwareValidationMatrix`:
  a single serial-menu bring-up sketch for ADC, I2C, SPI, UART1 loopback,
  watchdog, sleep, CPU frequency control, power profiles, BLE tests, and
  IEEE 802.15.4 tests.
- Added deterministic release archive mtimes in
  `tools/release_boards_manager.py` via `--source-date-epoch`.
- Added `tools/check_release_reproducible.py` to generate release artifacts
  twice and verify matching archive SHA-256 and package index JSON.
- Extended `tools/ci_fresh_machine_smoke.py` to validate
  `HardwareValidationMatrix` presence and compile coverage on default, BLE,
  and 802.15.4 profiles.
- Updated GitHub Actions workflow to run reproducibility checks before
  fresh-machine smoke.
