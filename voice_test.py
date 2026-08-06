#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Voice test — cycles through all block colors and speaks them.
Run this to verify spd-say is working on the robot.
"""

import subprocess
import sys
import time

# Colors to test (English → Chinese)
COLORS = [
    ("red",    "红色"),
    ("blue",   "蓝色"),
    ("green",  "绿色"),
    ("yellow", "黄色"),
]

LANG = "zh"


def speak(text):
    """Call spd-say to speak text. Returns True on success."""
    print("Speaking: %s" % text)
    try:
        result = subprocess.run(
            ["spd-say", "-l", LANG, "--", text],
            timeout=10,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        if result.returncode == 0:
            print("  OK")
            return True
        else:
            print("  FAIL: spd-say returned %d" % result.returncode)
            print("  stderr:", result.stderr.decode("utf-8", errors="replace"))
            return False
    except subprocess.TimeoutExpired:
        print("  FAIL: spd-say timed out")
        return False
    except FileNotFoundError:
        print("  FAIL: spd-say not found. Install: sudo apt install speech-dispatcher")
        return False
    except Exception as exc:
        print("  FAIL: %s" % exc)
        return False


def main():
    print("=" * 50)
    print("Voice Announcement Test")
    print("=" * 50)

    # Check spd-say availability
    try:
        result = subprocess.run(["which", "spd-say"], capture_output=True, text=True)
        if result.returncode != 0:
            print("ERROR: spd-say not installed!")
            print("Install with: sudo apt install speech-dispatcher")
            sys.exit(1)
        print("spd-say found: %s" % result.stdout.strip())
    except Exception:
        pass

    print()

    while True:
        for eng, cn in COLORS:
            text = "发现%s方块" % cn
            speak(text)
            time.sleep(1.5)

        print("--- Cycle complete, repeating in 3s ---")
        time.sleep(3)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
