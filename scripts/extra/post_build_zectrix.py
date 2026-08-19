Import("env")

import subprocess
from pathlib import Path


def post_build(source, target, env):
    build_dir = Path(env.subst("$BUILD_DIR"))
    output = build_dir / "merged_firmware.bin"

    subprocess.run(
        [
            env.subst("$UPLOADER"),
            "--chip",
            "ESP32S3",
            "merge-bin",
            "-o",
            str(output),
            "--flash-mode",
            "dio",
            "--flash-freq",
            "80m",
            "--flash-size",
            "16MB",
            "0x0000",
            str(build_dir / "bootloader.bin"),
            "0x8000",
            str(build_dir / "partitions.bin"),
            "0x20000",
            str(build_dir / "firmware.bin"),
        ],
        check=True,
    )

    print(f"Merged firmware: {output}")


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", post_build)
