#!/usr/bin/env python3
"""
Build target firmware binaries and optionally copy them to specified destination paths.

Run with --help for detailed usage information.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path


def check_idf_environment() -> None:
    """
    Check if ESP-IDF environment is properly set up.

    Verifies:
    - idf.py command is available (indicates toolchain is sourced)

    Raises:
        SystemExit: If ESP-IDF environment is not properly set up
    """
    # Check if idf.py is available (indicates toolchain is sourced)
    # This is the primary check - if idf.py works, ESP-IDF is available
    if not shutil.which("idf.py"):
        print("Error: idf.py command not found", file=sys.stderr)
        print("Please ensure ESP-IDF toolchain is sourced:", file=sys.stderr)
        print("  . $HOME/esp/esp-idf/export.sh", file=sys.stderr)
        print("  or source the ESP-IDF environment script", file=sys.stderr)
        sys.exit(1)


def build_firmware(chip: str, build_type: str, source_dir: Path) -> None:
    """
    Build firmware for a specific chip and build type.

    Args:
        chip: Chip name (e.g., "esp32", "esp32c3")
        build_type: Build type ("flash" or "ram")
        source_dir: Source directory containing the ESP-IDF project

    Raises:
        SystemExit: If any build step fails
    """
    if build_type not in ("flash", "ram"):
        print(
            f"Error: Invalid build_type '{build_type}'. Must be 'flash' or 'ram'",
            file=sys.stderr,
        )
        sys.exit(1)

    build_dir = f"build-{build_type}-{chip}"
    sdkconfig_defaults = f"sdkconfig.defaults.{build_type}"

    print(f"Building {chip} ({build_type} type)...")
    print(f"  Source directory: {source_dir}")
    print(f"  Build directory: {build_dir}")

    # Change to source directory
    original_cwd = os.getcwd()
    try:
        os.chdir(source_dir)

        # Step 1: Set target chip
        print(f"  Setting target to {chip}...")
        cmd = [
            "idf.py",
            "-B",
            build_dir,
            "-D",
            f"SDKCONFIG_DEFAULTS={sdkconfig_defaults}",
            "set-target",
            chip,
        ]
        result = subprocess.run(cmd, check=False, capture_output=True, text=True)
        if result.returncode != 0:
            print(f"Error: Failed to set target to {chip}", file=sys.stderr)
            print(result.stderr, file=sys.stderr)
            sys.exit(1)

        # Step 2: Build
        print("  Building...")
        cmd = ["idf.py", "-B", build_dir, "build"]
        result = subprocess.run(cmd, check=False, capture_output=True, text=True)
        if result.returncode != 0:
            print("Error: Build failed", file=sys.stderr)
            print(result.stderr, file=sys.stderr)
            sys.exit(1)

        print("  Build completed successfully")

    finally:
        os.chdir(original_cwd)


def normalize_path(example_name: str) -> str:
    """
    Normalize an example name by adding 'examples/' prefix and '/target-firmware' suffix if needed.

    Args:
        example_name: Example name that may be just the name or full path

    Returns:
        Full path with 'examples/' prefix and '/target-firmware' suffix
    """
    path = example_name
    # Remove 'examples/' prefix if present
    path = path.removeprefix("examples/")

    # Remove '/target-firmware' suffix if present
    path = path.removesuffix("/target-firmware")

    # Add prefix and suffix
    return f"examples/{path}/target-firmware"


def copy_binaries(
    chip: str, build_type: str, source_dir: Path, dest_paths: list[Path]
) -> None:
    """
    Copy binaries from build directory to destination paths.

    Args:
        chip: Chip name
        build_type: Build type ("flash" or "ram")
        source_dir: Source directory containing the ESP-IDF project
        dest_paths: List of destination directories where binaries should be copied

    Raises:
        SystemExit: If any required binary is missing or copy fails
    """
    build_dir = source_dir / f"build-{build_type}-{chip}"

    if not build_dir.is_dir():
        print(f"Error: Build directory not found: {build_dir}", file=sys.stderr)
        sys.exit(1)

    # Source files
    app_src = build_dir / "hello_world.bin"
    bootloader_src = build_dir / "bootloader" / "bootloader.bin"
    partition_src = build_dir / "partition_table" / "partition-table.bin"

    # Verify required files exist
    if not app_src.is_file():
        print(f"Error: Application binary not found: {app_src}", file=sys.stderr)
        sys.exit(1)

    if build_type == "flash":
        if not bootloader_src.is_file():
            print(
                f"Error: Bootloader binary not found: {bootloader_src}", file=sys.stderr
            )
            sys.exit(1)
        if not partition_src.is_file():
            print(
                f"Error: Partition table binary not found: {partition_src}",
                file=sys.stderr,
            )
            sys.exit(1)

    # Copy to each destination
    for dest_path in dest_paths:
        print(f"Copying binaries to {dest_path}...")
        dest_path.mkdir(parents=True, exist_ok=True)

        # Copy app binary
        shutil.copy2(app_src, dest_path / "app.bin")
        print("  Copied app.bin")

        # Copy flash binaries only for flash builds
        if build_type == "flash":
            shutil.copy2(bootloader_src, dest_path / "bootloader.bin")
            print("  Copied bootloader.bin")
            shutil.copy2(partition_src, dest_path / "partition-table.bin")
            print("  Copied partition-table.bin")


def main() -> int:
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Build target firmware binaries and optionally copy them to specified destination paths.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s esp32 flash esp32_example
  %(prog)s esp32c3 ram --build-only
  %(prog)s esp32 flash --copy-only esp32_example esp32_fast_reflash_example

Example names will be normalized to examples/<example_name>/target-firmware
        """,
    )

    parser.add_argument(
        "chip",
        help="Chip name (e.g., 'esp32', 'esp32c3', 'esp32c6', 'esp32h2')",
    )
    parser.add_argument(
        "build_type",
        choices=["flash", "ram"],
        help="Build type: 'flash' or 'ram'",
    )
    parser.add_argument(
        "--build-only",
        action="store_true",
        help="Only build, do not copy",
    )
    parser.add_argument(
        "--copy-only",
        action="store_true",
        help="Only copy, do not build (requires build artifacts to exist)",
    )
    parser.add_argument(
        "example_names",
        nargs="*",
        help="One or more example names where binaries should be copied. "
        "Example names will be normalized to examples/<example_name>/target-firmware",
    )

    args = parser.parse_args()

    build_only = args.build_only
    copy_only = args.copy_only
    example_names = args.example_names

    # Validate mutual exclusivity of flags
    if build_only and copy_only:
        parser.error("--build-only and --copy-only cannot be used together")

    # Check if example names are required
    if not build_only and not copy_only and not example_names:
        parser.error(
            "Either --build-only/--copy-only must be specified or at least one example name must be provided"
        )

    # If copy-only, example names are required
    if copy_only and not example_names:
        parser.error(
            "At least one example name must be provided when using --copy-only"
        )

    # Check ESP-IDF environment before proceeding
    check_idf_environment()

    # Get project directories
    script_dir = Path(__file__).parent.resolve()
    project_dir = script_dir.parent
    source_dir = project_dir / "test" / "target-example-src" / "hello-world-ESP32-src"

    if not source_dir.is_dir():
        print(f"Error: Source directory not found: {source_dir}", file=sys.stderr)
        sys.exit(1)

    # Build firmware (unless copy-only)
    if not copy_only:
        build_firmware(args.chip, args.build_type, source_dir)

    # Copy binaries (unless build-only)
    if not build_only:
        # Normalize example names (add examples/ prefix and /target-firmware suffix)
        normalized_paths = [normalize_path(name) for name in example_names]

        # Convert relative paths to absolute
        absolute_dest_paths = []
        for dest_path in normalized_paths:
            if Path(dest_path).is_absolute():
                absolute_dest_paths.append(Path(dest_path))
            else:
                absolute_dest_paths.append(project_dir / dest_path)

        copy_binaries(args.chip, args.build_type, source_dir, absolute_dest_paths)
        if copy_only:
            print(f"\nSuccessfully copied {args.chip} ({args.build_type}) binaries")
        else:
            print(
                f"\nSuccessfully built and copied {args.chip} ({args.build_type}) binaries"
            )
    else:
        print(f"\nSuccessfully built {args.chip} ({args.build_type}) binaries")

    return 0


if __name__ == "__main__":
    sys.exit(main())
