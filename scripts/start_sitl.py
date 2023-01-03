#!/usr/bin/env python
import signal
import sys
import subprocess
import rospkg
import time

PACK_PATH = ""

def signal_handler(sig, frame):
    with open("a.txt", "a") as f:
        f.write(str(sig)+"\n")
    rc = subprocess.call(f"{PACK_PATH}/scripts/shutdownsitl.sh", shell=True)
    print(f"shutdown.sh exited with return code {rc}")
    # exit()

if __name__ == "__main__":
    rospack = rospkg.RosPack()
    # catchable_sigs = set(signal.Signals) - {signal.SIGKILL, signal.SIGSTOP}
    catchable_sigs = [signal.SIGTERM, signal.SIGINT, signal.SIGHUP]
    for sig in catchable_sigs:
        signal.signal(sig, signal_handler)
    PACK_PATH = rospack.get_path("kuav_simulation")

    rc = subprocess.call(f"{PACK_PATH}/scripts/startsitl.sh {' '.join(sys.argv[1:])}", shell=True)
    print(f"startsitl.sh exited with return code {rc}")