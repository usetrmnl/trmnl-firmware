Import("env")
import subprocess

try:
    git_hash = subprocess.check_output(["git", "rev-parse", "--short", "HEAD"]).decode("ascii").strip()
except Exception as e:
    print(f"WARNING — could not read git hash ({e})")
    git_hash = ""

if git_hash:
    print(f"Defining FW_COMMIT: {git_hash}")
    env.ProcessFlags(f'-DFW_COMMIT=\\"{git_hash}\\"')
