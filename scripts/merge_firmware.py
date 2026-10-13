"""Merge the ESP-IDF flash plan into a complete Controller image at address 0."""
import argparse
import json
import subprocess
import sys
from pathlib import Path


def merge(build_dir: Path, output: Path) -> None:
    build_dir = build_dir.resolve()
    plan = json.loads((build_dir / "flasher_args.json").read_text(encoding="utf-8"))
    chip = plan["extra_esptool_args"]["chip"]
    if chip != "esp32c3":
        raise ValueError(f"Expected esp32c3, got {chip}")
    flash_files = sorted(plan["flash_files"].items(), key=lambda entry: int(entry[0], 0))
    if not flash_files or int(flash_files[0][0], 0) != 0:
        raise ValueError("The merged image must start with the bootloader at 0x0")
    required = {"bootloader.bin", "partition-table.bin", "ota_data_initial.bin", "drone_control.bin"}
    if not required.issubset({Path(filename).name for _, filename in flash_files}):
        raise ValueError("Flash plan is missing a required Controller image")
    output = output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(".bin.tmp")
    command = [sys.executable, "-m", "esptool", "--chip", chip, "merge_bin", "-o", str(temporary), "-f", "raw"]
    settings = plan["flash_settings"]
    for name in ("flash_mode", "flash_size", "flash_freq"):
        command.extend([f"--{name}", settings[name]])
    for address, filename in flash_files:
        image = (build_dir / filename).resolve()
        if not image.is_file():
            raise FileNotFoundError(image)
        command.extend([address, str(image)])
    try:
        subprocess.run(command, check=True)
        temporary.replace(output)
    finally:
        temporary.unlink(missing_ok=True)
    print(f"Complete flash image: {output} ({output.stat().st_size} bytes), flash at 0x0")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", type=Path, default=Path(__file__).resolve().parent.parent / "build")
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parent.parent / "flash_all.bin")
    args = parser.parse_args()
    merge(args.build_dir, args.output)
