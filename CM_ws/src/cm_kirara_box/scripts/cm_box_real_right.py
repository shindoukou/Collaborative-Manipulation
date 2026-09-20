#!/usr/bin/env python3
import sys
from pathlib import Path

import rospkg

DNMPC_SCRIPTS = Path(rospkg.RosPack().get_path("cm_dnmpc")) / "scripts"
sys.path.insert(0, str(DNMPC_SCRIPTS))

from cm_dnmpc_real_node import main


if __name__ == "__main__":
    main(forced_side="right", forced_object="box")
