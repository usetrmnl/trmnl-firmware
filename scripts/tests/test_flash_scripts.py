import os
from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]


class FlashScriptTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.firmware = self.directory / "test firmware.bin"
        self.firmware.write_bytes(b"test fixture, not real firmware")
        self.args_file = self.directory / "pio-args"
        pio = self.directory / "pio"
        pio.write_text(
            '#!/bin/bash\nprintf "%s\\n" "$@" > "$PIO_ARGS"\n'
            'exit "${PIO_EXIT:-0}"\n'
        )
        pio.chmod(0o755)
        self.env = {
            **os.environ,
            "PATH": str(self.directory) + os.pathsep + os.environ["PATH"],
            "PIO_ARGS": str(self.args_file),
            "NO_STUB": "0",
        }

    def run_flash(self, script, *args):
        return subprocess.run(
            ["bash", str(ROOT / "scripts" / script), *map(str, args)],
            env=self.env,
            capture_output=True,
            text=True,
        )

    def test_success_does_not_promise_automatic_reboot(self):
        for script in ("flash_app.sh", "flash_merged.sh"):
            with self.subTest(script=script):
                result = self.run_flash(script, self.firmware)
                self.assertEqual(0, result.returncode, result.stderr)
                self.assertIn("completed successfully", result.stdout)
                self.assertNotIn("will reboot automatically", result.stdout)
                self.assertIn("reset", result.stdout.lower())
                self.assertIn(str(self.firmware), self.args_file.read_text().splitlines())

    def test_merged_flash_explains_trmnl_x_soft_reset(self):
        result = self.run_flash("flash_merged.sh", self.firmware, "esp32s3")
        self.assertEqual(0, result.returncode, result.stderr)
        self.assertIn("TRMNL X", result.stdout)
        self.assertIn("soft reset", result.stdout.lower())
        self.assertIn("esp32s3", self.args_file.read_text().splitlines())

    def test_no_stub_options_are_preserved(self):
        self.env["NO_STUB"] = "1"
        result = self.run_flash("flash_merged.sh", self.firmware, "esp32s3")
        self.assertEqual(0, result.returncode, result.stderr)
        args = self.args_file.read_text().splitlines()
        self.assertIn("--no-stub", args)
        self.assertIn("--no-compress", args)

    def test_failure_does_not_print_success_or_reset_instructions(self):
        self.env["PIO_EXIT"] = "1"
        for script in ("flash_app.sh", "flash_merged.sh"):
            with self.subTest(script=script):
                result = self.run_flash(script, self.firmware)
                self.assertNotEqual(0, result.returncode)
                self.assertNotIn("completed successfully", result.stdout)
                self.assertNotIn("soft reset", result.stdout.lower())

    def test_missing_firmware_does_not_invoke_pio(self):
        for script in ("flash_app.sh", "flash_merged.sh"):
            with self.subTest(script=script):
                result = self.run_flash(script, self.directory / "missing.bin")
                self.assertNotEqual(0, result.returncode)
                self.assertFalse(self.args_file.exists())


if __name__ == "__main__":
    unittest.main()
