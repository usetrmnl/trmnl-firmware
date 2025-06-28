Import("env", "projenv")
import subprocess

try:
    git_hash = subprocess.check_output(['git', 'rev-parse', '--short', 'HEAD']).decode('ascii').strip()

    print("Using git hash", git_hash)
    
    projenv.Append(CPPDEFINES=[("GIT_COMMIT_HASH", f'\\"+{git_hash}\\"')])
            
except:
    print("No git repository found, using default empty GIT_COMMIT_HASH")
    pass