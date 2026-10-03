Import("env")
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(env.subst("$PROJECT_DIR")) / "scripts" / "extra"))
from versioned_copy import copy_versioned

def post_build(source, target, env):
    build_dir = Path(env.subst("$BUILD_DIR"))
    output = build_dir / "merged_firmware.bin"

    subprocess.run([
        # The build's Python: pioarduino installs esptool's dependencies only in its penv.
        env.subst("$PYTHONEXE"), str(Path(env.PioPlatform().get_package_dir("tool-esptoolpy")) / "esptool.py"),
        "--chip", "ESP32S3",
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
env.AddPostAction("buildprog", lambda source, target, env: copy_versioned(env))
