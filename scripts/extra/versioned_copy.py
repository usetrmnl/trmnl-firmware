"""Shared helper for post-build scripts: copy build outputs to FWx.y.z names."""
import re
import shutil
from pathlib import Path


def read_fw_version(project_dir):
    config = (Path(project_dir) / "include" / "config.h").read_text()
    parts = []
    for name in ("MAJOR", "MINOR", "PATCH"):
        match = re.search(rf"#define\s+FW_{name}_VERSION\s+(\d+)", config)
        if not match:
            raise RuntimeError(f"FW_{name}_VERSION not found in include/config.h")
        parts.append(match.group(1))
    return ".".join(parts)


def copy_versioned(env):
    build_dir = Path(env.subst("$BUILD_DIR"))
    version = read_fw_version(env.subst("$PROJECT_DIR"))

    prefix = f"FW{version}-{env.subst('$PIOENV')}"
    elf = env.subst("${PROGNAME}.elf")

    copies = (
        ("merged_firmware.bin", ".bin"),
        (elf, ".elf"),
        (env.subst("${PROGNAME}.bin"), "-ota.bin"),
        (elf, "-ota.elf"),
    )
    for src_name, suffix in copies:
        src = build_dir / src_name
        dst = build_dir / f"{prefix}{suffix}"
        shutil.copyfile(src, dst)
        print(f"Copied {src.name} -> {dst}")
