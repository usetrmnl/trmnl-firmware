Import("env")
import os
import subprocess

try:
    git_hash = subprocess.check_output(["git", "rev-parse", "--short", "HEAD"]).decode("ascii").strip()
except Exception as e:
    print(f"WARNING — could not read git hash ({e})")
    git_hash = ""

# A header, not a -D flag: a flag that changes every commit misses the build cache for every file.
header_dir = os.path.join(env.subst("$BUILD_DIR"), "fw_commit")
header = os.path.join(header_dir, "fw_commit.h")
content = f'#define FW_COMMIT "{git_hash}"\n'
os.makedirs(header_dir, exist_ok=True)
if not os.path.exists(header) or open(header).read() != content:
    with open(header, "w") as f:
        f.write(content)

print(f"Defining FW_COMMIT: {git_hash}")
env.ProcessFlags(f"-I {header_dir}")
