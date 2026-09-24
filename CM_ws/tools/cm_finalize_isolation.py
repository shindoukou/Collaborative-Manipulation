#!/usr/bin/env python3
from pathlib import Path


ROOT = Path("/home/mas-2/3rd_party/CM_ws")
SRC = ROOT / "src"
OLD = "/home/mas-2/3rd_party/TCST_ws"
NEW = "/home/mas-2/3rd_party/CM_ws"
OLD_MESH = OLD + "/src/tcst_planning/models/meshes"
NEW_MESH = NEW + "/src/cm_kirara_description/meshes"


def replace_text(path):
    data = path.read_text(encoding="utf-8", errors="surrogateescape")
    updated = data.replace(OLD_MESH, NEW_MESH).replace(OLD, NEW)
    if updated != data:
        path.write_text(updated, encoding="utf-8", errors="surrogateescape")


for path in SRC.rglob("*"):
    if not path.is_file() or "__pycache__" in path.parts:
        continue
    try:
        replace_text(path)
    except (UnicodeDecodeError, ValueError):
        pass

obsolete = (
    SRC / "cm_kirara_ball" / "scripts" / "install_0505_estimate_run_0803.py",
    SRC / "cm_kirara_ball" / "scripts" / "scan_move_start_0803.py",
)
for path in obsolete:
    if path.exists():
        path.unlink()

for cache in SRC.rglob("__pycache__"):
    if cache.is_dir():
        for child in cache.iterdir():
            child.unlink()
        cache.rmdir()
