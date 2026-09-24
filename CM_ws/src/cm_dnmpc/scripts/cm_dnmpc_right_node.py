#!/usr/bin/env python3
import sys
from pathlib import Path

SOURCE = str(Path(__file__).resolve().parent)
if SOURCE not in sys.path:
    sys.path.insert(0, SOURCE)

from cm_dnmpc_arm_node import main


if __name__ == "__main__":
    main("right")
