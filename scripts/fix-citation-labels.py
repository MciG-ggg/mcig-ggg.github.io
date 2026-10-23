#!/usr/bin/env python3
"""Give every source citation the full label the contract asks for.

The contract says a citation looks like

    [第3章_回归.pptx · p.14](http://alist/.../第3章_回归.pdf#page=14)

Some notes were written with the short form `[p. 14](url)`. The file name is
in the URL, so the label can be rebuilt from it.

Usage: python3 scripts/fix-citation-labels.py <笔记目录> [...]
"""

from __future__ import annotations

import re
import sys
from pathlib import Path
from urllib.parse import unquote, urlsplit

SHORT_RE = re.compile(r"\[p\.\s*(\d+)\]\(([^)\s]+)\)")


def file_name(url: str) -> str | None:
    """Read the file name out of a source URL."""
    path = urlsplit(url).path
    name = unquote(path.rsplit("/", 1)[-1])
    return name or None


def fix(text: str) -> tuple[str, int]:
    """Rewrite the short citations. Returns the new text and the count."""
    count = 0

    def replace(match: re.Match[str]) -> str:
        nonlocal count
        page, url = match.group(1), match.group(2)
        name = file_name(url)
        if not name:
            return match.group(0)
        count += 1
        return f"[{name} · p.{page}]({url})"

    return SHORT_RE.sub(replace, text), count


def main() -> None:
    if len(sys.argv) < 2:
        sys.exit("usage: python3 scripts/fix-citation-labels.py <笔记目录> [...]")

    total = 0
    for target in sys.argv[1:]:
        root = Path(target)
        pages = [root] if root.is_file() else sorted(root.rglob("index.md"))
        for page in pages:
            text = page.read_text(encoding="utf-8")
            new, count = fix(text)
            if count:
                page.write_text(new, encoding="utf-8")
                total += count
                print(f"{page}: 改了 {count} 处")
    print(f"一共改了 {total} 处")


if __name__ == "__main__":
    main()
