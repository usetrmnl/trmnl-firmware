Import("env")
import subprocess
import urllib.request
from pathlib import Path

# Prebuilt LittleFS image (fonts/assets) shipped with the TRMNL X family.
LITTLEFS_URL = "https://trmnl-fw.s3.us-east-2.amazonaws.com/littlefs.bin"
LITTLEFS_OFFSET = "0x620000"

def post_build(source, target, env):
    build_dir = Path(env.subst("$BUILD_DIR"))
    output = build_dir / "merged_firmware.bin"

    images = [
        "0x0000", str(build_dir / "bootloader.bin"),
        "0x8000", str(build_dir / "partitions.bin"),
        "0x20000", str(build_dir / "firmware.bin"),
    ]

    # Only include the LittleFS image for envs that actually mount LittleFS
    # (board_build.filesystem = littlefs). Envs on SPIFFS, e.g. seeed_sticky,
    # share the same partition layout but format that partition on first boot,
    # so flashing the image would only bloat the merged binary.
    filesystem = env.BoardConfig().get("build.filesystem", "spiffs")
    if filesystem == "littlefs":
        littlefs = build_dir / "littlefs.bin"
        if not littlefs.exists():
            print(f"Downloading littlefs.bin from {LITTLEFS_URL} ...")
            urllib.request.urlretrieve(LITTLEFS_URL, littlefs)
        images += [LITTLEFS_OFFSET, str(littlefs)]
    else:
        print(f"board_build.filesystem is '{filesystem}'; skipping littlefs.bin in merged image")

    subprocess.run([
        "pio", "pkg", "exec", "-p", "tool-esptoolpy", "esptool.py", "--",
        "--chip", "ESP32S3",
        "merge_bin",
        "-o", str(output),
        "--flash_mode", "dio",
        "--flash_freq", "80m",
        "--flash_size", "16MB",
        *images,
    ], check=True)

    print(f"Merged firmware: {output}")


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", post_build)
