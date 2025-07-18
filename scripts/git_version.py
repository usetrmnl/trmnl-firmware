Import("env", "projenv")
import subprocess

try:
    include_hash = env.GetProjectOption("custom_include_git_hash", "false").lower() == "true"
    
    if include_hash:
        git_hash = subprocess.check_output(['git', 'rev-parse', '--short', 'HEAD']).decode('ascii').strip()
        print(f"Using git hash: {git_hash}")
        projenv.Append(CPPDEFINES=[("FW_VERSION_SUFFIX", f'\\"+{git_hash}\\"')])
    else:
        print("Skipping git hash (custom_include_git_hash = false)")
            
except:
    print("No git repository found, using default empty FW_VERSION_SUFFIX")
    pass