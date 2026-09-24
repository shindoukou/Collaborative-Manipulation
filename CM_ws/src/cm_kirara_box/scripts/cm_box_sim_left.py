#!/usr/bin/env python3
import sys
from pathlib import Path

import rospkg

DNMPC_SCRIPTS = Path(rospkg.RosPack().get_path("cm_dnmpc")) / "scripts"
sys.path.insert(0, str(DNMPC_SCRIPTS))

from cm_dnmpc_arm_node import main


if __name__ == "__main__":
    if "--command-enabled" not in sys.argv and "--dry-run" not in sys.argv:
        sys.argv.append("--command-enabled")
    if not any(argument == "--joint-timeout" or
               argument.startswith("--joint-timeout=")
               for argument in sys.argv):
        sys.argv.extend(("--joint-timeout", "120"))
    main(forced_side="left", forced_object="box", forced_runtime="sim")
