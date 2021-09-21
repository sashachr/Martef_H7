import os
import subprocess

def get_git_current_version() -> str:
    return subprocess.check_output(['git', 'describe', '--abbrev=6', '--dirty', '--always', '--tags']).decode('UTF-8').strip()
def get_git_current_hash() -> str:
    return subprocess.check_output(['git', 'rev-parse', 'HEAD']).decode('UTF-8').strip()

ver = get_git_current_version()
hash = get_git_current_hash()
file = 'Martef\\gitversion.h'
with open(file, 'w') as gv:
    gv.write('uint8_t GitVersion[] = \"' + ver + '\";\n')
    gv.write('uint8_t GitSha[] = \"' + hash + '\";\n')
pass