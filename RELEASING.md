# Releasing (Boards Manager)

This repository publishes `nrf54l15:nrf54l15` through Arduino Boards Manager.

## 1) Bump core version

Update the version in:

- `hardware/nrf54l15/nrf54l15/platform.txt`
- add a matching section to `CHANGELOG.md`

## 2) Build release archive + update package index

Run from repository root:

```bash
python3 tools/release_boards_manager.py --version <new_version> --repo lolren/NRF54L15-Arduino-core
```

This generates:

- `dist/nrf54l15-zephyr-based-<new_version>.tar.bz2`
- updated `package_nrf54l15_zephyr_based_index.json` (and mirror `package_nrf54l15_index.json`)

Optional local smoke check (simulates a fresh machine install through Boards Manager):

```bash
python3 tools/ci_fresh_machine_smoke.py
```

Optional reproducibility check (build release twice and compare outputs):

```bash
python3 tools/check_release_reproducible.py
```

This smoke test validates installation and compilation of packaged examples, including:

- `examples/GPIO/InterruptButton`
- `examples/Power/battery`
- `examples/Power/BatteryMeasure`
- `examples/Power/LowPowerFeatures`
- `examples/Power/LowPowerProfiles`
- `examples/Power/PeripheralPowerGating`
- `examples/Power/CpuFrequencyControl`
- `examples/Power/WatchdogSleepWake`
- `examples/Power/lowpower`
- `examples/Radio/AntennaControl`
- `examples/Radio/RadioProfileInfo`
- `examples/Radio/RFChipDisable`
- `libraries/Watchdog/examples/FeedWatchdog`
- `libraries/Bluetooth/examples/BLEScan` (BLE menu profile)
- `libraries/Bluetooth/examples/BLEScanForEach` (BLE menu profile)
- `libraries/Bluetooth/examples/BLECentralConnect` (BLE menu profile)
- `examples/BLE/BLEScanTest` (BLE menu profile)
- `examples/BLE/BLEAdvertiseTest` (BLE menu profile)
- `examples/BLE/BLEScanMonitor` (BLE menu profile)
- `examples/BLE/BLECentralMonitor` (BLE menu profile)
- `libraries/IEEE802154/examples/IEEE802154Config` (802.15.4 profile)
- `libraries/IEEE802154/examples/IEEE802154PassiveScan` (802.15.4 profile)
- `examples/Radio/IEEE802154FeatureProbe` (802.15.4 profile)
- `examples/Radio/zigbee_scan` (802.15.4 profile)
- `examples/Radio/zigbee_radio_config` (802.15.4 profile)

The packager excludes generated/heavy tool directories, including:

- `hardware/nrf54l15/nrf54l15/tools/.arduino-zephyr-build`
- `hardware/nrf54l15/nrf54l15/tools/ncs`
- `hardware/nrf54l15/nrf54l15/tools/zephyr-sdk`
- `hardware/nrf54l15/nrf54l15/variants/*/zephyr_lib`

## 3) Publish GitHub release asset

Create tag `v<new_version>` and upload:

- `dist/nrf54l15-zephyr-based-<new_version>.tar.bz2`

## 4) Push index update

Commit and push:

- `package_nrf54l15_zephyr_based_index.json`
- `package_nrf54l15_index.json`

Users can then add this URL in Arduino IDE:

- `https://raw.githubusercontent.com/lolren/NRF54L15-Arduino-core/main/package_nrf54l15_index.json`

## CI checks

GitHub Actions workflow `.github/workflows/fresh-machine-smoke.yml` runs the fresh-machine Boards Manager smoke test on:

- Linux (`ubuntu-latest`)
- Windows (`windows-latest`)
- includes deterministic release reproducibility verification

## Licensing note

NCS/Zephyr SDK components are downloaded by bootstrap scripts at build time and are not bundled in this repository's release archive.
Keep upstream license texts and attribution notices intact in your published source tree.
