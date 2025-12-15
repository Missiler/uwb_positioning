#!/usr/bin/env python3
import sys
from uwb_positioning.reader import main

def run():
    # Strip bare --ros-args
    sys.argv = [a for a in sys.argv if a != '--ros-args']
    main()

if __name__ == "__main__":
    run()