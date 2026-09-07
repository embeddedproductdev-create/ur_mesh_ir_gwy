"""
RELEASE merge -- run this only AFTER a build has been tested with
merge_test.py and confirmed working on real hardware.

Batch-merge ESP32-S3 firmware images for the Unimation BLE Mesh project,
organized into version-numbered folders for easy client handoff and
traceability back to the exact git commit that produced them.

For every serial-number NVS .bin found in the serial-number folder, this
script produces one fully self-contained merged firmware image containing:

    0x0      bootloader.bin
    0x8000   partition-table.bin
    0x1b000  <SERIAL>.bin        (unique per device)
    0x20000  app.bin             (e.g. IR_BLE_MESH.bin)

Output layout:

    merged_bin/
      v1.0.4-1-g468785a6/
        manifest.json
        merged-GWY00001.bin
        merged-GWY00002.bin
        ...
      v1.0.5/
        manifest.json
        merged-GWY00001.bin
        ...

The version folder name comes from `git describe --tags --always --dirty`,
run inside PROJECT_DIR -- the same mechanism ESP-IDF itself uses to stamp
CONFIG_APP_COMPILE_TIME_DATE / the app version shown at boot and in the
build log ("App IR_BLE_MESH version: V1.0.4-1-g468785a6"). This keeps the
folder name, the git history, and the running firmware's own version
string all pointing at the same commit.

Usage:
    python merge_release.py                  # normal run
    python merge_release.py --force           # allow a dirty working tree
    python merge_release.py --version v1.0.5  # override the detected version

Requires the 'esptool' package (pip install esptool).
"""

import argparse
import json
import os
import subprocess
import sys
from datetime import datetime, timezone

# ============================== CONFIG ==================================

PROJECT_DIR = r"D:\Freelance\Unimation\ble_mesh\ble_mesh_ur"

BOOTLOADER_BIN = os.path.join(PROJECT_DIR, "build", "bootloader", "bootloader.bin")
PARTITION_BIN  = os.path.join(PROJECT_DIR, "build", "partition_table", "partition-table.bin")
APP_BIN        = os.path.join(PROJECT_DIR, "build", "IR_BLE_MESH.bin")

# Folder containing one .bin per device serial number (from SerNoGen.py)
SERIAL_BIN_DIR = os.path.join(
    PROJECT_DIR, "main", "src", "NVS_Partition_Generator", "bin"
)

# Root folder for all versioned merged output
OUTPUT_ROOT = os.path.join(PROJECT_DIR, "merged_bin")

# Flash offsets (from ir_partitions.csv)
BOOTLOADER_OFFSET = "0x0"
PARTITION_OFFSET  = "0x8000"
SERIAL_OFFSET     = "0x1b000"
APP_OFFSET        = "0x30000"

CHIP        = "esp32s3"
FLASH_MODE  = "dio"
FLASH_FREQ  = "80m"
FLASH_SIZE  = "16MB"

# =========================================================================


def check_file(path, label):
    if not os.path.isfile(path):
        print(f"ERROR: {label} not found: {path}")
        sys.exit(1)


def run_git(args):
    """Run a git command in PROJECT_DIR, return stripped stdout or None on failure."""
    try:
        result = subprocess.run(
            ["git"] + args,
            cwd=PROJECT_DIR,
            capture_output=True,
            text=True,
            check=True,
        )
        return result.stdout.strip()
    except Exception:
        return None


def get_version_info():
    """
    Returns (version_string, commit_hash, branch, is_dirty).
    Falls back to a timestamp-based pseudo-version if this isn't a git repo.
    """
    describe = run_git(["describe", "--tags", "--always", "--dirty"])
    commit = run_git(["rev-parse", "--short", "HEAD"])
    branch = run_git(["rev-parse", "--abbrev-ref", "HEAD"])

    if describe is None:
        # Not a git repo (or git not installed) -- fall back so the script
        # still runs, but flag it clearly since traceability is lost.
        fallback = "unversioned-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        return fallback, None, None, True

    is_dirty = describe.endswith("-dirty")
    return describe, commit, branch, is_dirty


def sanitize_folder_name(name):
    """Strip characters that aren't safe in a Windows folder name."""
    invalid = '<>:"/\\|?*'
    for ch in invalid:
        name = name.replace(ch, "-")
    return name


def main():
    parser = argparse.ArgumentParser(description="Batch-merge versioned ESP32-S3 firmware images.")
    parser.add_argument("--force", action="store_true",
                         help="Proceed even if the git working tree is dirty (uncommitted changes).")
    parser.add_argument("--version", type=str, default=None,
                         help="Override the auto-detected version string for the output folder name.")
    args = parser.parse_args()

    check_file(BOOTLOADER_BIN, "Bootloader binary")
    check_file(PARTITION_BIN, "Partition table binary")
    check_file(APP_BIN, "App binary")

    if not os.path.isdir(SERIAL_BIN_DIR):
        print(f"ERROR: Serial bin directory not found: {SERIAL_BIN_DIR}")
        sys.exit(1)

    serial_bins = sorted(
        f for f in os.listdir(SERIAL_BIN_DIR)
        if f.lower().endswith(".bin")
    )
    if not serial_bins:
        print(f"ERROR: No .bin files found in {SERIAL_BIN_DIR}")
        sys.exit(1)

    version, commit, branch, is_dirty = get_version_info()
    if args.version:
        version = args.version

    if commit is None:
        print("WARNING: This does not appear to be a git repository (or git is not on PATH).")
        print("         Merged binaries will NOT be traceable to a specific commit.")
        print(f"         Using fallback version folder: {version}\n")
    elif is_dirty and not args.force:
        print("ERROR: Git working tree has uncommitted changes (version string ends in '-dirty').")
        print(f"       Detected version: {version}")
        print("       Commit or stash your changes before producing a client build, so the")
        print("       merged binaries can always be traced back to an exact commit.")
        print("       Re-run with --force to override and proceed anyway.")
        sys.exit(1)
    elif is_dirty and args.force:
        print(f"WARNING: Proceeding with a DIRTY working tree (--force). Version: {version}\n")

    version_folder = sanitize_folder_name(version)
    output_dir = os.path.join(OUTPUT_ROOT, version_folder)

    if os.path.isdir(output_dir):
        print(f"NOTE: Output folder already exists and will be reused/overwritten: {output_dir}")

    os.makedirs(output_dir, exist_ok=True)

    print(f"Version        : {version}")
    print(f"Git commit     : {commit or 'N/A'}")
    print(f"Git branch     : {branch or 'N/A'}")
    print(f"Output folder  : {output_dir}")
    print(f"Devices found  : {len(serial_bins)}\n")

    failures = []
    merged_devices = []

    for serial_bin_name in serial_bins:
        serial_bin_path = os.path.join(SERIAL_BIN_DIR, serial_bin_name)
        device_name = os.path.splitext(serial_bin_name)[0]  # e.g. GWY00001
        output_path = os.path.join(output_dir, f"merged-{device_name}.bin")

        cmd = [
            sys.executable, "-m", "esptool",
            "--chip", CHIP,
            "merge_bin",
            "-o", output_path,
            "--flash_mode", FLASH_MODE,
            "--flash_freq", FLASH_FREQ,
            "--flash_size", FLASH_SIZE,
            BOOTLOADER_OFFSET, BOOTLOADER_BIN,
            PARTITION_OFFSET, PARTITION_BIN,
            SERIAL_OFFSET, serial_bin_path,
            APP_OFFSET, APP_BIN,
        ]

        print(f"[{device_name}] Merging -> {output_path}")
        result = subprocess.run(cmd, capture_output=True, text=True)

        if result.returncode != 0:
            print(f"  FAILED:\n{result.stderr}")
            failures.append(device_name)
        else:
            print("  OK")
            merged_devices.append(device_name)

    manifest = {
        "version": version,
        "git_commit": commit,
        "git_branch": branch,
        "git_dirty": is_dirty,
        "build_date_utc": datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC"),
        "chip": CHIP,
        "flash_mode": FLASH_MODE,
        "flash_freq": FLASH_FREQ,
        "flash_size": FLASH_SIZE,
        "offsets": {
            "bootloader": BOOTLOADER_OFFSET,
            "partition_table": PARTITION_OFFSET,
            "serial_number": SERIAL_OFFSET,
            "app": APP_OFFSET,
        },
        "source_files": {
            "bootloader": BOOTLOADER_BIN,
            "partition_table": PARTITION_BIN,
            "app": APP_BIN,
        },
        "devices_merged": merged_devices,
        "devices_failed": failures,
    }

    manifest_path = os.path.join(output_dir, "manifest.json")
    with open(manifest_path, "w") as f:
        json.dump(manifest, f, indent=2)

    print("\n=== Summary ===")
    print(f"Version       : {version}")
    print(f"Total devices : {len(serial_bins)}")
    print(f"Succeeded     : {len(merged_devices)}")
    print(f"Failed        : {len(failures)}")
    if failures:
        print("Failed devices:", ", ".join(failures))
    print(f"Manifest      : {manifest_path}")
    print(f"\nAll merged binaries written to: {output_dir}")

    if failures:
        sys.exit(1)


if __name__ == "__main__":
    main()
