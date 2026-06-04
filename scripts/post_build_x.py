Import("env")
import subprocess
import urllib.request
from pathlib import Path

LITTLEFS_URL = "https://trmnl-fw.s3.us-east-2.amazonaws.com/littlefs.bin"

def post_build(source, target, env):
    build_dir = Path(env.subst("$BUILD_DIR"))
    output = build_dir / "merged_firmware.bin"
    littlefs = build_dir / "littlefs.bin"

    if not littlefs.exists():
        print(f"Downloading littlefs.bin from {LITTLEFS_URL} ...")
        urllib.request.urlretrieve(LITTLEFS_URL, littlefs)

    subprocess.run([
        "pio", "pkg", "exec", "-p", "tool-esptoolpy", "esptool.py", "--",
        "--chip", "ESP32S3",
        "merge_bin",
        "-o", str(output),
        "--flash_mode", "dio",
        "--flash_freq", "80m",
        "--flash_size", "16MB",
        "0x0000", str(build_dir / "bootloader.bin"),
        "0x8000", str(build_dir / "partitions.bin"),
        "0x20000", str(build_dir / "firmware.bin"),
        "0x620000", str(littlefs),
    ], check=True)

    print(f"Merged firmware: {output}")


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", post_build)
