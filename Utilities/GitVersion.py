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
    gv.write('#define GITVERSION \"' + ver + '\"\n')
    guid = '#define GITSHA { 0x' + hash[0:8]
    guid += ', 0x' + hash[8:12]
    guid += ', 0x' + hash[12:16]
    guid += ', { 0x' + hash[16:18] + ', 0x' + hash[18:20] + ', 0x' + hash[20:22] + ', 0x' + hash[22:24] + ', 0x' + hash[24:26] + ', 0x' + hash[26:28] + ', 0x' + hash[28:30] + ', 0x' + hash[30:32] + ' } }\n'
    gv.write(guid)
pass