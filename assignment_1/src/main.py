#!/usr/bin/env python3

import start
import signal
import sys
import os

def signal_handler(sig, frame):
    os.system('killall -9 roslaunch roscore python')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
start.main()
