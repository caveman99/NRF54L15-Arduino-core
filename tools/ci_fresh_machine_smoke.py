#!/usr/bin/env python3
from __future__ import annotations

import contextlib
import fnmatch
import http.server
import os
import socket
import subprocess
import sys
import tempfile
import threading
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parent.parent
PLATFORM_TXT = REPO_ROOT / "hardware" / "nrf54l15" / "nrf54l15" / "platform.txt"
RELEASE_SCRIPT = REPO_ROOT / "tools" / "release_boards_manager.py"

FORBIDDEN_PATTERNS = [
    "*/tools/ncs/*",
    "*/tools/ncs.partial/*",
    "*/tools/zephyr-sdk/*",
    "*/tools/.arduino-zephyr-build/*",
    "*/variants/*/zephyr_lib/*",
]


def run(cmd: list[str], *, cwd: Path | None = None, env: dict[str, str] | None = None) -> None:
    print("+", " ".join(cmd), flush=True)
    subprocess.run(cmd, cwd=str(cwd) if cwd else None, env=env, check=True)


def read_core_version(platform_txt: Path) -> str:
    for line in platform_txt.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line.startswith("version="):
            return line.split("=", 1)[1].strip()
    raise RuntimeError(f"Unable to find version= in {platform_txt}")


def choose_free_port() -> int:
    with contextlib.closing(socket.socket(socket.AF_INET, socket.SOCK_STREAM)) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def quote_yaml_path(path: Path) -> str:
    return str(path).replace("\\", "\\\\")


def assert_archive_excludes(archive_path: Path) -> None:
    import tarfile

    with tarfile.open(archive_path, mode="r:bz2") as tar:
        names = [m.name for m in tar.getmembers() if m.isfile()]

    for name in names:
        normalized = name.replace("\\", "/")
        for pattern in FORBIDDEN_PATTERNS:
            if fnmatch.fnmatch(normalized, pattern):
                raise RuntimeError(f"Forbidden packaged path found: {normalized} (pattern: {pattern})")


class QuietHandler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, _format: str, *_args: object) -> None:
        return


@contextlib.contextmanager
def http_server(root: Path, port: int):
    handler = lambda *args, **kwargs: QuietHandler(*args, directory=str(root), **kwargs)
    httpd = http.server.ThreadingHTTPServer(("127.0.0.1", port), handler)
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    try:
        yield
    finally:
        httpd.shutdown()
        httpd.server_close()
        thread.join(timeout=5)


def main() -> int:
    if not PLATFORM_TXT.is_file():
        raise RuntimeError(f"Missing {PLATFORM_TXT}")
    if not RELEASE_SCRIPT.is_file():
        raise RuntimeError(f"Missing {RELEASE_SCRIPT}")

    version = read_core_version(PLATFORM_TXT)
    package_name = f"nrf54l15-zephyr-based-{version}.tar.bz2"

    with tempfile.TemporaryDirectory(prefix="nrf54l15-smoke-") as temp_dir_str:
        temp_dir = Path(temp_dir_str)
        dist_dir = temp_dir / "dist"
        dist_dir.mkdir(parents=True, exist_ok=True)

        port = choose_free_port()
        archive_url = f"http://127.0.0.1:{port}/dist/{package_name}"
        index_path = temp_dir / "package_nrf54l15_zephyr_based_test.json"

        run(
            [
                sys.executable,
                str(RELEASE_SCRIPT),
                "--version",
                version,
                "--repo",
                "example/example",
                "--dist-dir",
                str(dist_dir),
                "--index-path",
                str(index_path),
                "--archive-url",
                archive_url,
            ],
            cwd=REPO_ROOT,
        )

        archive_path = dist_dir / package_name
        if not archive_path.is_file():
            raise RuntimeError(f"Release archive missing: {archive_path}")
        assert_archive_excludes(archive_path)

        data_dir = temp_dir / "arduino-data"
        user_dir = temp_dir / "arduino-user"
        data_dir.mkdir(parents=True, exist_ok=True)
        user_dir.mkdir(parents=True, exist_ok=True)

        cfg_path = temp_dir / "arduino-cli.yaml"
        cfg_path.write_text(
            "\n".join(
                [
                    "board_manager:",
                    "  additional_urls:",
                    f"    - http://127.0.0.1:{port}/{index_path.name}",
                    "directories:",
                    f"  data: \"{quote_yaml_path(data_dir)}\"",
                    f"  user: \"{quote_yaml_path(user_dir)}\"",
                    "",
                ]
            ),
            encoding="utf-8",
        )

        env = dict(os.environ)

        with http_server(temp_dir, port):
            run(["arduino-cli", "--config-file", str(cfg_path), "core", "update-index"], env=env)
            run(
                [
                    "arduino-cli",
                    "--config-file",
                    str(cfg_path),
                    "core",
                    "install",
                    f"nrf54l15:nrf54l15@{version}",
                ],
                env=env,
            )

            result = subprocess.run(
                ["arduino-cli", "--config-file", str(cfg_path), "core", "list"],
                check=True,
                capture_output=True,
                text=True,
                env=env,
            )

        if "nrf54l15:nrf54l15" not in result.stdout:
            raise RuntimeError("Installed core not found in `arduino-cli core list` output")

        installed_core_root = (
            data_dir / "packages" / "nrf54l15" / "hardware" / "nrf54l15" / version
        )
        if not installed_core_root.is_dir():
            raise RuntimeError(f"Installed core directory missing: {installed_core_root}")

        interrupt_example = installed_core_root / "examples" / "GPIO" / "InterruptButton"
        battery_example = installed_core_root / "examples" / "Power" / "battery"
        low_power_example = installed_core_root / "examples" / "Power" / "LowPowerFeatures"
        low_power_profiles_example = installed_core_root / "examples" / "Power" / "LowPowerProfiles"
        peripheral_power_gating_example = installed_core_root / "examples" / "Power" / "PeripheralPowerGating"
        cpu_freq_example = installed_core_root / "examples" / "Power" / "CpuFrequencyControl"
        watchdog_example = installed_core_root / "examples" / "Power" / "WatchdogSleepWake"
        lowpower_simple_example = installed_core_root / "examples" / "Power" / "lowpower"

        antenna_example = installed_core_root / "examples" / "Radio" / "AntennaControl"
        radio_profile_example = installed_core_root / "examples" / "Radio" / "RadioProfileInfo"
        ieee802154_probe_example = installed_core_root / "examples" / "Radio" / "IEEE802154FeatureProbe"
        zigbee_scan_example = installed_core_root / "examples" / "Radio" / "zigbee_scan"
        zigbee_radio_config_example = installed_core_root / "examples" / "Radio" / "zigbee_radio_config"

        ble_scan_test_example = installed_core_root / "examples" / "BLE" / "BLEScanTest"
        ble_advertise_test_example = installed_core_root / "examples" / "BLE" / "BLEAdvertiseTest"
        ble_scan_monitor_example = installed_core_root / "examples" / "BLE" / "BLEScanMonitor"
        ble_central_monitor_example = installed_core_root / "examples" / "BLE" / "BLECentralMonitor"

        watchdog_lib_example = installed_core_root / "libraries" / "Watchdog" / "examples" / "FeedWatchdog"
        ble_scan_example = installed_core_root / "libraries" / "Bluetooth" / "examples" / "BLEScan"
        ble_scan_foreach_example = installed_core_root / "libraries" / "Bluetooth" / "examples" / "BLEScanForEach"
        ble_central_example = installed_core_root / "libraries" / "Bluetooth" / "examples" / "BLECentralConnect"
        ieee802154_example = installed_core_root / "libraries" / "IEEE802154" / "examples" / "IEEE802154Config"
        ieee802154_scan_example = (
            installed_core_root / "libraries" / "IEEE802154" / "examples" / "IEEE802154PassiveScan"
        )

        required_examples = [
            interrupt_example,
            battery_example,
            low_power_example,
            low_power_profiles_example,
            peripheral_power_gating_example,
            cpu_freq_example,
            watchdog_example,
            lowpower_simple_example,
            antenna_example,
            radio_profile_example,
            ieee802154_probe_example,
            zigbee_scan_example,
            zigbee_radio_config_example,
            ble_scan_test_example,
            ble_advertise_test_example,
            ble_scan_monitor_example,
            ble_central_monitor_example,
            watchdog_lib_example,
            ble_scan_example,
            ble_scan_foreach_example,
            ble_central_example,
            ieee802154_example,
            ieee802154_scan_example,
        ]
        for example in required_examples:
            if not example.is_dir():
                raise RuntimeError(f"Missing installed example: {example}")

        fqbn_default = "nrf54l15:nrf54l15:xiao_nrf54l15"
        fqbn_ble = (
            "nrf54l15:nrf54l15:xiao_nrf54l15:"
            "clean_radio_profile=ble_only,"
            "clean_bt_controller=zephyr_ll_sw"
        )
        fqbn_802154 = (
            "nrf54l15:nrf54l15:xiao_nrf54l15:"
            "clean_radio_profile=ieee802154_only"
        )
        fqbn_dual = (
            "nrf54l15:nrf54l15:xiao_nrf54l15:"
            "clean_radio_profile=dual,"
            "clean_bt_controller=zephyr_ll_sw"
        )

        def compile_example(fqbn: str, sketch_dir: Path) -> None:
            run(
                [
                    "arduino-cli",
                    "--config-file",
                    str(cfg_path),
                    "compile",
                    "--clean",
                    "-b",
                    fqbn,
                    str(sketch_dir),
                ],
                env=env,
            )

        default_examples = [
            interrupt_example,
            battery_example,
            low_power_example,
            low_power_profiles_example,
            peripheral_power_gating_example,
            cpu_freq_example,
            watchdog_example,
            lowpower_simple_example,
            antenna_example,
            radio_profile_example,
            watchdog_lib_example,
        ]
        for sketch in default_examples:
            compile_example(fqbn_default, sketch)

        ble_examples = [
            ble_scan_example,
            ble_scan_foreach_example,
            ble_central_example,
            ble_scan_test_example,
            ble_advertise_test_example,
            ble_scan_monitor_example,
            ble_central_monitor_example,
        ]
        for sketch in ble_examples:
            compile_example(fqbn_ble, sketch)

        ieee802154_examples = [
            ieee802154_example,
            ieee802154_scan_example,
            ieee802154_probe_example,
            zigbee_scan_example,
            zigbee_radio_config_example,
        ]
        for sketch in ieee802154_examples:
            compile_example(fqbn_802154, sketch)

        compile_example(fqbn_dual, radio_profile_example)

    print("Fresh machine Boards Manager smoke test passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
