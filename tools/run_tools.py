Import("env")

import os
import subprocess

print("Running build scripts...")

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


def run_tool(name: str):
    tool_path = os.path.join(os.getcwd(), 'lib/core/tools', name)
    print(bcolors.WARNING + f"[-] Running {name}" + bcolors.ENDC)
    res = subprocess.run(tool_path)
    if res.returncode != 0:
        print(bcolors.FAIL + f"[X] {name} failed" + bcolors.ENDC)


run_tool("gen_statuscode.py")
