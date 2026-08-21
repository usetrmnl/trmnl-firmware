Import("env")
import subprocess
from pathlib import Path

def post_build(source, target, env):
    build_dir = Path(env.subst("$BUILD_DIR"))
    output = build_dir / "merged_firmware.bin"

    subprocess.run([
        "pio", "pkg", "exec", "-p", "tool-esptoolpy", "esptool.py", "--",
        "--chip", "ESP32C3",
        "merge_bin",
        "-o", str(output),
        "--flash_mode", "dio",
        "--flash_freq", "40m",
        "--flash_size", "4MB",
        "0x0000", str(build_dir / "bootloader.bin"),
        "0x8000", str(build_dir / "partitions.bin"),
        "0x10000", str(build_dir / "firmware.bin"),
    ], check=True)

    print(f"Merged firmware: {output}")


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", post_build)
