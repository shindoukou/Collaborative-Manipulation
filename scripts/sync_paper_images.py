r"""Update the README image copies from the manuscript's original figure directory.

Usage:
    py -3 scripts/sync_paper_images.py --source-dir "F:\\BaiduSyncdisk\\0728latex"
    py -3 scripts/sync_paper_images.py --source-dir "F:\\BaiduSyncdisk\\0728latex" --watch

The source directory remains the authority. GitHub displays copies in this repo;
it cannot read files from a visitor's local F: drive.
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import tempfile
import time
from pathlib import Path

from PIL import Image


REPO = Path(__file__).resolve().parents[1]
MANIFEST = REPO / "assets" / "paper" / "manifest.json"


def source_for(source_dir: Path, reference: str) -> Path:
    path = source_dir.joinpath(*reference.split("/"))
    if path.is_file():
        return path
    for extension in (".pdf", ".png", ".PNG", ".jpg", ".jpeg"):
        candidate = Path(str(path) + extension)
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(f"Original image not found: {path}")


def render_pdf(source: Path, target: Path, pdftoppm: str) -> None:
    with tempfile.TemporaryDirectory(prefix="paper-figure-") as directory:
        prefix = Path(directory) / "page"
        subprocess.run(
            [pdftoppm, "-f", "1", "-singlefile", "-cropbox", "-png", "-r", "180", str(source), str(prefix)],
            check=True,
            stdout=subprocess.DEVNULL,
        )
        with Image.open(prefix.with_suffix(".png")) as image:
            image.load()
            if image.mode not in ("RGB", "RGBA"):
                image = image.convert("RGB")
            image.thumbnail((1800, 1800), Image.Resampling.LANCZOS)
            image.save(target, "PNG", optimize=True)


def make_preview(source: Path, target: Path, pdftoppm: str | None) -> None:
    target.parent.mkdir(parents=True, exist_ok=True)
    if source.suffix.lower() == ".pdf":
        if pdftoppm is None:
            raise RuntimeError("pdftoppm is required to display PDF figures in the README")
        render_pdf(source, target, pdftoppm)
        return

    with Image.open(source) as image:
        image.load()
        if target.suffix.lower() == ".jpg":
            if image.mode in ("RGBA", "LA"):
                canvas = Image.new("RGB", image.size, "white")
                canvas.paste(image.convert("RGB"), mask=image.getchannel("A"))
                image = canvas
            else:
                image = image.convert("RGB")
            image.thumbnail((900, 900), Image.Resampling.LANCZOS)
            image.save(target, "JPEG", quality=84, optimize=True, progressive=True)
        else:
            image.save(target)


def synchronize(source_dir: Path, force: bool, response_dir: Path | None = None) -> int:
    entries = json.loads(MANIFEST.read_text(encoding="utf-8"))
    pdftoppm = shutil.which("pdftoppm")
    changed = 0
    response_skipped = False
    for entry in entries:
        root = response_dir if entry.get("source_root") == "response" else source_dir
        if root is None:
            response_skipped = True
            continue
        source = source_for(root, entry["latex_reference"])
        target = REPO.joinpath(*entry["repository_path"].split("/"))
        if not force and target.exists() and source.stat().st_mtime_ns <= target.stat().st_mtime_ns:
            continue
        make_preview(source, target, pdftoppm)
        changed += 1
        print(f"Updated {entry['repository_path']} from {source}")
    if response_skipped:
        print("Response-letter images skipped: pass --response-dir to update them.")
    return changed


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-dir", type=Path, required=True, help="Directory containing the manuscript's original images")
    parser.add_argument("--response-dir", type=Path, help="Response template directory, for figures that differ from the manuscript")
    parser.add_argument("--watch", action="store_true", help="Keep local previews in sync while this command runs")
    parser.add_argument("--force", action="store_true", help="Rebuild every preview, even when the source timestamp is older")
    args = parser.parse_args()
    if not args.source_dir.is_dir():
        parser.error(f"Source directory does not exist: {args.source_dir}")

    if args.response_dir is not None and not args.response_dir.is_dir():
        parser.error(f"Response directory does not exist: {args.response_dir}")

    try:
        while True:
            changed = synchronize(args.source_dir, args.force, args.response_dir)
            print(f"Synchronized {changed} image(s).")
            if not args.watch:
                break
            args.force = False
            time.sleep(2)
    except KeyboardInterrupt:
        print("Stopped watching.")


if __name__ == "__main__":
    main()
