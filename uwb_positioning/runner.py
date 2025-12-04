#!/usr/bin/env python3
def main():
    import sys
    # Strip the bare --ros-args token that launch injects
    sys.argv = [a for a in sys.argv if a != '--ros-args']
    from uwb_positioning import main as real_main
    real_main()