"""
Quick merge for TESTING ONLY -- not for client delivery.

Merges bootloader + partition table + one serial-number bin (or all of them)
+ app into a single output folder. No versioning, no git checks. Always
overwrites the previous test output so you can iterate quickly.

Once a build has been tested and passes, use merge_release.py instead to
produce the properly versioned, client-ready output.

Usage:
    python merge_test.py                # merge ALL serial bins found
    python merge_test.py GWY00001        # merge just one device, by name
"""

import os
import subprocess
import sys

# ============================== CONFIG ==================================

PROJECT_DIR = r"D:\Freelance\Unimation\ble_mesh\ble_mesh_ur"

BOOTLOADER_BIN = os.path.join(PROJECT_DIR, "build", "bootloader", "bootloader.bin")
PARTITION_BIN  = os.path.join(PROJECT_DIR, "build", "partition_table", "partition-table.bin")
APP_BIN        = os.path.join(PROJECT_DIR, "build", "IR_BLE_MESH.bin")

SERIAL_BIN_DIR = os.path.join(
    PROJECT_DIR, "main", "src", "NVS_Partition_Generator", "bin"
)

# Fixed test output folder -- gets overwritten every run, on purpose.
OUTPUT_DIR = os.path.join(PROJECT_DIR, "merged_bin_test")

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


def merge_one(serial_bin_path, device_name, output_dir):
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
        return False

    print("  OK")
    return True


def main():
    check_file(BOOTLOADER_BIN, "Bootloader binary")
    check_file(PARTITION_BIN, "Partition table binary")
    check_file(APP_BIN, "App binary")

    if not os.path.isdir(SERIAL_BIN_DIR):
        print(f"ERROR: Serial bin directory not found: {SERIAL_BIN_DIR}")
        sys.exit(1)

    requested_device = sys.argv[1] if len(sys.argv) > 1 else None

    all_serial_bins = sorted(
        f for f in os.listdir(SERIAL_BIN_DIR)
        if f.lower().endswith(".bin")
    )

    if requested_device:
        # Allow passing either "GWY00001" or "GWY00001.bin"
        target = requested_device if requested_device.lower().endswith(".bin") else requested_device + ".bin"
        if target not in all_serial_bins:
            print(f"ERROR: '{requested_device}' not found in {SERIAL_BIN_DIR}")
            print(f"Available: {', '.join(os.path.splitext(f)[0] for f in all_serial_bins)}")
            sys.exit(1)
        serial_bins = [target]
    else:
        serial_bins = all_serial_bins

    if not serial_bins:
        print(f"ERROR: No .bin files found in {SERIAL_BIN_DIR}")
        sys.exit(1)

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    print("TEST MERGE -- not versioned, not for client delivery")
    print(f"Output folder : {OUTPUT_DIR}")
    print(f"Devices       : {len(serial_bins)}\n")

    failures = []
    for serial_bin_name in serial_bins:
        serial_bin_path = os.path.join(SERIAL_BIN_DIR, serial_bin_name)
        device_name = os.path.splitext(serial_bin_name)[0]
        if not merge_one(serial_bin_path, device_name, OUTPUT_DIR):
            failures.append(device_name)

    print("\n=== Summary ===")
    print(f"Total   : {len(serial_bins)}")
    print(f"OK      : {len(serial_bins) - len(failures)}")
    print(f"Failed  : {len(failures)}")
    if failures:
        print("Failed devices:", ", ".join(failures))
        sys.exit(1)

    print(f"\nReady to flash for testing, e.g.:")
    print(f'  python -m esptool --chip esp32s3 --port COMx --baud 921600 write_flash 0x0 "{OUTPUT_DIR}\\merged-{os.path.splitext(serial_bins[0])[0]}.bin"')


if __name__ == "__main__":
    main()
