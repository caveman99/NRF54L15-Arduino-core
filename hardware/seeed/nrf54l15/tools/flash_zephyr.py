#!/usr/bin/env python3
from __future__ import annotations

import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import List

from zephyr_common import (
    core_paths,
    prepend_sdk_tool_paths,
    read_metadata,
    resolve_program,
    run,
    sdk_tool,
    west_cmd_with_zephyr_base,
    with_west_pythonpath,
)


def parse_supported_runners(runners_file: Path) -> List[str]:
    if not runners_file.is_file():
        return ["openocd", "jlink", "nrfutil", "nrfjprog"]

    runners: List[str] = []
    in_section = False
    for raw in runners_file.read_text(encoding="utf-8").splitlines():
        line = raw.rstrip()
        if line.strip() == "runners:":
            in_section = True
            continue
        if in_section and not line.strip():
            break
        if in_section and line.lstrip().startswith("- "):
            runners.append(line.split("- ", 1)[1].strip())

    return runners if runners else ["openocd", "jlink", "nrfutil", "nrfjprog"]


def print_result(result: subprocess.CompletedProcess[str]) -> None:
    if result.stdout:
        print(result.stdout, end="")
    if result.stderr:
        print(result.stderr, file=sys.stderr, end="")


def looks_like_locked_target(result: subprocess.CompletedProcess[str]) -> bool:
    details = ((result.stdout or "") + "\n" + (result.stderr or "")).lower()
    indicators = (
        "approtect",
        "memory transfer fault",
        "fault ack",
        "failed to read memory",
        "dp initialisation failed",
        "ap write error",
        "locked",
        "no cores were discovered",
    )
    return any(token in details for token in indicators)


def looks_like_nrf54l_mass_erase_timeout(result: subprocess.CompletedProcess[str]) -> bool:
    details = ((result.stdout or "") + "\n" + (result.stderr or "")).lower()
    indicators = (
        "mass erase timeout waiting for eraseallstatus",
        "no cores were discovered",
    )
    return any(token in details for token in indicators)


def append_uid(cmd: list[str], uid: str | None) -> list[str]:
    if uid:
        cmd.extend(["-u", uid])
    return cmd


def detect_pyocd_command() -> list[str] | None:
    pyocd_exe = resolve_program(["pyocd", "pyocd.exe"])
    if pyocd_exe:
        return [pyocd_exe]

    probe = subprocess.run(
        [sys.executable, "-m", "pyocd", "--version"],
        check=False,
        text=True,
        capture_output=True,
    )
    if probe.returncode == 0:
        return [sys.executable, "-m", "pyocd"]

    return None


def force_nrf54l_unlock_workaround(pyocd_cmd: list[str], uid: str | None) -> subprocess.CompletedProcess[str]:
    print("OpenOCD indicates a protected/locked target; attempting pyOCD CTRL-AP unlock workaround...")

    script_lines = [
        "initdp",
        "writeap 2 0x04 1",
        "sleep 500",
        "readap 2 0x08",
        "sleep 500",
        "readap 2 0x08",
        "writeap 2 0x00 2",
        "writeap 2 0x00 0",
        "sleep 500",
        "readap 2 0x08",
        "readap 0 0x00",
        "readap 1 0x00",
    ]

    with tempfile.NamedTemporaryFile("w", suffix=".txt", delete=False) as script_file:
        script_file.write("\n".join(script_lines))
        script_path = script_file.name

    try:
        cmd = append_uid(
            [*pyocd_cmd, "commander", "-N", "-O", "auto_unlock=false", "-x", script_path],
            uid,
        )
        result = run(cmd, check=False, capture_output=True)
        print_result(result)
        return result
    finally:
        try:
            os.unlink(script_path)
        except OSError:
            pass


def pyocd_flash_hex(pyocd_cmd: list[str], hex_path: Path, uid: str | None) -> int:
    target = "nrf54l"
    connect_mode = "under-reset"

    def run_cmd(cmd: list[str]) -> subprocess.CompletedProcess[str]:
        result = run(cmd, check=False, capture_output=True)
        print_result(result)
        return result

    print(f"Flashing {hex_path}")
    print("Runner: pyocd")
    print(f"Probe UID: {uid or 'auto-select'}")

    load_cmd = append_uid(
        [*pyocd_cmd, "load", "-W", "-t", target, "-M", connect_mode, str(hex_path), "--format", "hex"],
        uid,
    )
    load_result = run_cmd(load_cmd)

    if load_result.returncode != 0 and looks_like_locked_target(load_result):
        erase_cmd = append_uid(
            [*pyocd_cmd, "erase", "-W", "--chip", "-t", target, "-M", connect_mode],
            uid,
        )
        erase_result = run_cmd(erase_cmd)
        if erase_result.returncode == 0:
            load_result = run_cmd(load_cmd)
        elif looks_like_nrf54l_mass_erase_timeout(erase_result):
            workaround = force_nrf54l_unlock_workaround(pyocd_cmd, uid)
            if workaround.returncode == 0:
                load_cmd_no_unlock = append_uid(
                    [
                        *pyocd_cmd,
                        "load",
                        "-W",
                        "-t",
                        target,
                        "-M",
                        connect_mode,
                        "-O",
                        "auto_unlock=false",
                        str(hex_path),
                        "--format",
                        "hex",
                    ],
                    uid,
                )
                load_result = run_cmd(load_cmd_no_unlock)

    if load_result.returncode != 0:
        return load_result.returncode

    reset_cmd = append_uid(
        [*pyocd_cmd, "reset", "-W", "-t", target, "-M", connect_mode, "-O", "auto_unlock=false"],
        uid,
    )
    reset_result = run_cmd(reset_cmd)
    return 0 if reset_result.returncode == 0 else reset_result.returncode


def has_nrfjprog_probe() -> bool:
    if not resolve_program(["nrfjprog", "nrfjprog.exe"]):
        return False
    try:
        out = subprocess.check_output(["nrfjprog", "--ids"], text=True, stderr=subprocess.DEVNULL)
        return bool(out.strip())
    except Exception:
        return False


def runner_available(runner: str, sdk_dir: Path) -> bool:
    if runner == "nrfjprog":
        return resolve_program(["nrfjprog", "nrfjprog.exe"]) is not None
    if runner == "openocd":
        if resolve_program(["openocd", "openocd.exe"]):
            return True
        return sdk_tool(sdk_dir, "openocd") is not None
    if runner == "jlink":
        return (
            resolve_program(["JLinkExe", "JLink.exe", "JLinkGDBServer", "JLinkGDBServer.exe"]) is not None
        )
    if runner == "nrfutil":
        binary = resolve_program(["nrfutil", "nrfutil.exe"])
        if not binary:
            return False
        try:
            subprocess.check_call([binary, "--help"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return True
        except Exception:
            return False
    return False


def default_runner(supported: List[str], sdk_dir: Path) -> str:
    if "nrfjprog" in supported and runner_available("nrfjprog", sdk_dir) and has_nrfjprog_probe():
        return "nrfjprog"
    for candidate in ["openocd", "jlink", "nrfutil"]:
        if candidate in supported and runner_available(candidate, sdk_dir):
            return candidate
    return supported[0] if supported else "openocd"


def main() -> int:
    if len(sys.argv) < 2:
        print("Usage: flash_zephyr.py <variant_dir> [runner]", file=sys.stderr)
        return 1

    variant_dir = Path(sys.argv[1]).resolve()
    runner_input = sys.argv[2] if len(sys.argv) > 2 else os.environ.get("ARDUINO_ZEPHYR_RUNNER", "auto")
    platform_dir = (variant_dir / ".." / "..").resolve()
    paths = core_paths(Path(__file__))
    sdk_dir = paths["sdk_dir"]
    ncs_dir = paths["ncs_dir"]

    zephyr_lib_dir = variant_dir / "zephyr_lib"
    metadata_file = zephyr_lib_dir / "metadata.env"
    metadata = read_metadata(metadata_file)
    build_dir = Path(metadata.get("ARDUINO_ZEPHYR_BUILD_DIR", str(zephyr_lib_dir / "build")))
    runners_file = build_dir / "zephyr" / "runners.yaml"

    if not build_dir.is_dir():
        run([sys.executable, str(platform_dir / "tools" / "build_zephyr_lib.py")], check=True)
        metadata = read_metadata(metadata_file)
        build_dir = Path(metadata.get("ARDUINO_ZEPHYR_BUILD_DIR", str(zephyr_lib_dir / "build")))
        runners_file = build_dir / "zephyr" / "runners.yaml"

    if not build_dir.is_dir():
        raise RuntimeError(f"Missing Zephyr build directory: {build_dir}")

    supported_runners = parse_supported_runners(runners_file)
    runner = runner_input
    if runner_input == "auto":
        runner = default_runner(supported_runners, sdk_dir)
        print(f"Auto-selected upload runner: {runner}")

    if runner == "pyocd":
        pyocd_cmd = detect_pyocd_command()
        if not pyocd_cmd:
            raise RuntimeError("pyocd runner selected but pyocd is not available in PATH.")
        dev_id = os.environ.get("ARDUINO_ZEPHYR_DEV_ID", "") or None
        hex_path = build_dir / "zephyr" / "zephyr.hex"
        if not hex_path.is_file():
            raise RuntimeError(f"Missing Zephyr HEX output: {hex_path}")
        return pyocd_flash_hex(pyocd_cmd, hex_path, dev_id)

    if runner not in supported_runners:
        raise RuntimeError(
            f"Runner '{runner}' is not supported by this build. Supported runners: {' '.join(supported_runners)}"
        )

    env = with_west_pythonpath(platform_dir)
    env["ZEPHYR_SDK_INSTALL_DIR"] = str(sdk_dir)
    env["ZEPHYR_BASE"] = str((ncs_dir / "zephyr").resolve())
    prepend_sdk_tool_paths(env, sdk_dir)

    # Ensure west import and command path are valid before flash.
    run(west_cmd_with_zephyr_base(ncs_dir) + ["--version"], env=env, check=True, capture_output=True)

    extra_args: List[str] = []
    dev_id = os.environ.get("ARDUINO_ZEPHYR_DEV_ID", "")
    if dev_id and runner in ("nrfjprog", "nrfutil", "jlink"):
        extra_args += ["--dev-id", dev_id]

    flash_cmd = west_cmd_with_zephyr_base(ncs_dir) + ["flash", "-d", str(build_dir), "-r", runner, *extra_args]
    if os.environ.get("ARDUINO_ZEPHYR_FLASH_DRY_RUN", "0") == "1":
        print("Dry run command: " + " ".join([shlex_quote(x) for x in flash_cmd]))
        return 0

    result = run(flash_cmd, cwd=ncs_dir, env=env, check=False, capture_output=True)
    print_result(result)
    if result.returncode == 0:
        return 0

    # First-use boards can ship in a locked/debug-disabled state that causes OpenOCD
    # to fail with "no cores discovered"/"failed to read memory"/DP init errors.
    # If that happens, run the same pyOCD CTRL-AP workaround used by the clean core
    # and retry once.
    if runner == "openocd" and looks_like_locked_target(result):
        pyocd_cmd = detect_pyocd_command()
        if not pyocd_cmd:
            print(
                "HINT: Install pyocd (or select the pyOCD upload method) to recover locked nRF54L targets.",
                file=sys.stderr,
            )
        else:
            dev_id = os.environ.get("ARDUINO_ZEPHYR_DEV_ID", "") or None
            workaround = force_nrf54l_unlock_workaround(pyocd_cmd, dev_id)
            if workaround.returncode == 0:
                retry = run(flash_cmd, cwd=ncs_dir, env=env, check=False, capture_output=True)
                print_result(retry)
                return retry.returncode

    return result.returncode


def shlex_quote(s: str) -> str:
    if not s:
        return "''"
    if all(ch.isalnum() or ch in "._/-" for ch in s):
        return s
    return "'" + s.replace("'", "'\"'\"'") + "'"


if __name__ == "__main__":
    raise SystemExit(main())
