"""Ensure the pinned idf-component-manager version in the ESP-IDF Python venv.

The espressif32/pioarduino platforms provision one Python virtualenv per IDF
version (~/.platformio/penv/.espidf-<version>) and install their own pinned
idf-component-manager (2.x) into it -- but only when the venv is first
created; it is never upgraded afterwards. We rely on idf-component-manager
3.x for reproducible dependencies.lock handling, so this pre-script enforces
the version in two ways:

1. It wraps env.VerboseAction so that when the platform builder provisions a
   fresh venv (new machine, CI cache miss, IDF or Python bump), the builder's
   own "pip install" / "uv pip install" command installs our pin instead of
   the platform default. This keeps even the very first build consistent.

2. If the venv already exists with a different version (machines provisioned
   before this script existed), the package is upgraded in place before the
   ESP-IDF build is configured.

Runs as a "pre:" extra_script, i.e. before the platform's espidf builder.
"""

import os
import re
import subprocess
import sys
from pathlib import Path

Import("env")  # noqa: F821

PACKAGE = "idf-component-manager"
VERSION = "3.0.3"
PIN = f"{PACKAGE}=={VERSION}"


def _venv_executable(venv_dir, name):
    ext = ".exe" if os.name == "nt" else ""
    for subdir in ("bin", "Scripts"):
        exe = Path(venv_dir) / subdir / f"{name}{ext}"
        if exe.is_file():
            return str(exe)
    return None


def _idf_version():
    """IDF version (major.minor.patch), derived the same way the platform
    builder names the venv."""
    framework_dir = env.PioPlatform().get_package_dir("framework-espidf")
    if not framework_dir:
        return None
    version_file = Path(framework_dir) / "tools" / "cmake" / "version.cmake"
    try:
        parts = dict(
            re.findall(
                r"set\(IDF_VERSION_(MAJOR|MINOR|PATCH) (\d+)\)",
                version_file.read_text(),
            )
        )
        return "{MAJOR}.{MINOR}.{PATCH}".format(**parts)
    except (OSError, KeyError):
        return None


def _patch_fresh_venv_install():
    """Rewrite the platform builder's Python-deps install command so a freshly
    created venv gets our pin instead of the platform default."""
    orig_verbose_action = env.VerboseAction

    def verbose_action(_, act, actstr):
        if isinstance(act, str) and PACKAGE in act and " install " in act:
            act = re.sub(re.escape(PACKAGE) + r'[^"]*', PIN, act)
        return orig_verbose_action(act, actstr)

    env.AddMethod(verbose_action, "VerboseAction")


def _fix_existing_venv():
    idf_version = _idf_version()
    if not idf_version:
        return
    penv_dir = Path(env.subst("$PROJECT_CORE_DIR")) / "penv"
    venv_dir = penv_dir / f".espidf-{idf_version}"
    python_exe = _venv_executable(venv_dir, "python")
    if not python_exe:
        # venv doesn't exist yet; the fresh-install patch above handles it
        return

    result = subprocess.run(
        [
            python_exe,
            "-c",
            f"from importlib.metadata import version; print(version('{PACKAGE}'))",
        ],
        capture_output=True,
        text=True,
    )
    installed = result.stdout.strip() if result.returncode == 0 else None
    if installed == VERSION:
        return

    print(f"Updating {PACKAGE} {installed or '(missing)'} -> {VERSION} in {venv_dir}")
    uv_exe = _venv_executable(penv_dir, "uv")
    if uv_exe:
        cmd = [uv_exe, "pip", "install", "--python", python_exe, PIN]
    else:
        cmd = [python_exe, "-m", "pip", "install", PIN]
    if subprocess.run(cmd).returncode != 0:
        sys.stderr.write(f"Error: failed to install {PIN} into {venv_dir}\n")
        env.Exit(1)


if "espidf" in env.get("PIOFRAMEWORK", []):
    _patch_fresh_venv_install()
    _fix_existing_venv()
