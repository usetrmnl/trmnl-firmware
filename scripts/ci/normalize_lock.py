#!/usr/bin/env python3
"""Normalize dependencies.lock for cross-platform comparison and commit."""

import re
import sys
from pathlib import Path

RELATIVE_LOCAL_PATH = "components/esp-serial-flasher"


def normalize(content: str) -> str:
    content = re.sub(
        r"(?m)^(      path: ).*[/\\]components[/\\]esp-serial-flasher\s*$",
        rf"\1{RELATIVE_LOCAL_PATH}",
        content,
    )
    return content


def main() -> int:
    if len(sys.argv) > 1:
        content = Path(sys.argv[1]).read_text(encoding="utf-8")
    else:
        content = sys.stdin.read()
    sys.stdout.write(normalize(content))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
