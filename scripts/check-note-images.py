#!/usr/bin/env python3
"""Check the lecture notes for broken image references.

A line that ends in `](images/x.png)` but never opens with `![` is a broken
image: the alt text was cut off, so the page shows no picture and no caption.
The regular image check in `lint_note.py` misses these, because a line without
`![` is not an image to it.

Usage:
    python3 scripts/check-note-images.py
    python3 scripts/check-note-images.py path/to/index.md
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
CONTENT = REPO / "src/content/lecture"

# A line that ends in an image link.
IMAGE_TAIL_RE = re.compile(r"\]\(images/[^)\s]+\)\s*$")


def check_file(path: Path) -> list[str]:
    """Return the problems in one note file."""
    problems: list[str] = []
    text = path.read_text(encoding="utf-8")
    for lineno, line in enumerate(text.splitlines(), start=1):
        stripped = line.strip()
        if IMAGE_TAIL_RE.search(stripped) and "![" not in stripped:
            problems.append(f"{path}:{lineno}: 图片引用缺 `![` 开头：{stripped[:70]}")
    return problems


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("files", nargs="*", help="note files to check; default is every lecture note")
    args = parser.parse_args()

    if args.files:
        files = [Path(name).expanduser().resolve() for name in args.files]
    else:
        files = sorted(CONTENT.rglob("*.md"))

    for path in files:
        if not path.is_file():
            print(f"error: note not found: {path}", file=sys.stderr)
            return 2

    problems: list[str] = []
    for path in files:
        problems.extend(check_file(path))

    for problem in problems:
        print(problem, file=sys.stderr)
    if problems:
        print(f"{len(problems)} broken image reference(s)", file=sys.stderr)
        return 1
    print(f"check-note-images: OK ({len(files)} file(s))")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
