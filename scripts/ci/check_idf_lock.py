#!/usr/bin/env python3
"""Compare a generated dependencies.lock to a committed baseline lockfile."""

import difflib
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

from normalize_lock import normalize


def main() -> int:
    if len(sys.argv) != 3:
        print(
            "usage: check_idf_lock.py <baseline.lock> <generated.lock>",
            file=sys.stderr,
        )
        return 2

    baseline_path = Path(sys.argv[1])
    generated_path = Path(sys.argv[2])

    baseline = normalize(baseline_path.read_text(encoding="utf-8"))
    generated = normalize(generated_path.read_text(encoding="utf-8"))

    if baseline == generated:
        print(f"OK: {generated_path} matches {baseline_path}")
        return 0

    diff = difflib.unified_diff(
        baseline.splitlines(keepends=True),
        generated.splitlines(keepends=True),
        fromfile=str(baseline_path),
        tofile=str(generated_path),
    )
    sys.stdout.writelines(diff)
    print(
        f"\nERROR: dependency lock drift detected.\n"
        f"  baseline:  {baseline_path}\n"
        f"  generated: {generated_path}\n"
        f"If the change is intentional, regenerate lockfiles with:\n"
        f"  ./scripts/ci/update_idf_locks.sh",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
