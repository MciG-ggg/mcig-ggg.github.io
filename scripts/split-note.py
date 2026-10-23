#!/usr/bin/env python3
"""Split a long lecture note into one page per chapter.

The long course notes are hard to flip through. This script turns one note
into a landing page plus one page per chapter.

  src/content/lecture/zh-cn/自动控制原理_期末复习讲义/
    index.md                      <- 目录页
    第 0 章 前置知识/index.md      <- 一章一页
    第 0 章 前置知识/images/       <- 该章用到的图

The images move with the chapter that uses them, so the paths stay
`images/xxx.png` and the contract holds.

Usage: python3 scripts/split-note.py <笔记目录>
"""

from __future__ import annotations

import re
import shutil
import sys
from pathlib import Path

FRONTMATTER_RE = re.compile(r"\A---\n(.*?)\n---\n", re.DOTALL)
CHAPTER_RE = re.compile(r"^## (.+)$", re.MULTILINE)
# 图的 alt 文本里可能有方括号，比如 $[x^*-\epsilon]$，所以只找收尾的 ](images/...)
IMAGE_RE = re.compile(r"\]\(\s*(images/[^)\s]+)\s*\)")
CARDLINK_RE = re.compile(r"```cardlink\n.*?```", re.DOTALL)
ILLEGAL = '/\\:*?"<>|'


def read_frontmatter(text: str) -> tuple[dict[str, str], str]:
    """Split the frontmatter fields from the body."""
    match = FRONTMATTER_RE.match(text)
    if not match:
        return {}, text
    fields: dict[str, str] = {}
    for line in match.group(1).splitlines():
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        if ":" not in line:
            continue
        key, value = line.split(":", 1)
        fields[key.strip()] = value.strip()
    return fields, text[match.end():]


def field_list(fields: dict[str, str], key: str) -> list[str]:
    """Read a bracketed list field."""
    value = fields.get(key, "")
    return [item.strip() for item in value.strip("[]").split(",") if item.strip()]


def sanitize(title: str) -> str:
    """Turn a chapter title into a directory name."""
    name = title.strip()
    for char in ILLEGAL:
        name = name.replace(char, "-")
    return name.strip()


def describe(body: str, limit: int = 70) -> str:
    """Write a one line description from the first paragraph of the body."""
    for block in body.split("\n\n"):
        text = block.strip()
        if not text or text.startswith(("#", ">", "|", "-", "!", "`", ":::")):
            continue
        text = re.sub(r"\[([^\]]*)\]\([^)]*\)", r"\1", text)
        text = re.sub(r"[*_`$]", "", text).strip()
        if len(text) > limit:
            text = text[:limit].rstrip() + "…"
        return text
    return ""


def main() -> None:
    if len(sys.argv) != 2:
        sys.exit("usage: python3 scripts/split-note.py <笔记目录>")

    note_dir = Path(sys.argv[1]).resolve()
    index = note_dir / "index.md"
    if not index.is_file():
        sys.exit(f"error: {index} 不存在")

    text = index.read_text(encoding="utf-8")
    fields, body = read_frontmatter(text)

    matches = list(CHAPTER_RE.finditer(body))
    if not matches:
        sys.exit(f"error: {index} 里没有 ## 章节")

    preamble = body[: matches[0].start()].strip()
    cardlink = CARDLINK_RE.search(preamble)
    cardlink_text = cardlink.group(0) if cardlink else ""

    series = fields.get("series", note_dir.name)
    tags = ", ".join(field_list(fields, "tags")) or series
    timestamp = fields.get("timestamp", "2026-01-01T00:00:00+08:00")
    source_url = fields.get("source_url", "")
    draft = fields.get("draft", "false")
    toc = fields.get("toc", "true")
    description = fields.get("description", "")

    chapters: list[tuple[str, str]] = []
    for position, match in enumerate(matches):
        start = match.end()
        end = matches[position + 1].start() if position + 1 < len(matches) else len(body)
        chapters.append((match.group(1).strip(), body[start:end].strip()))

    images_dir = note_dir / "images"
    moved = 0

    for title, chapter_body in chapters:
        directory = note_dir / sanitize(title)
        if directory.exists():
            shutil.rmtree(directory)
        chapter_images = directory / "images"
        chapter_images.mkdir(parents=True)

        for relative in set(IMAGE_RE.findall(chapter_body)):
            source = note_dir / relative
            if source.exists():
                shutil.move(str(source), str(chapter_images / Path(relative).name))
                moved += 1

        chapter_fields = [
            "---",
            f"title: {title}",
            f"timestamp: {timestamp}",
            f"series: {series}",
            f"tags: [{tags}]",
            f"description: {describe(chapter_body) or description}",
            f"source_url: {source_url}",
            f"toc: {toc}",
            f"draft: {draft}",
            "---",
            "",
        ]
        page = "\n".join(chapter_fields)
        if cardlink_text:
            page += cardlink_text + "\n\n"
        page += f"[← 返回《{fields.get('title', note_dir.name)}》目录](../)\n\n"
        page += f"## {title}\n\n{chapter_body}\n"
        (directory / "index.md").write_text(page, encoding="utf-8")

    lines = [
        "---",
        f"title: {fields.get('title', note_dir.name)}",
        f"timestamp: {timestamp}",
        f"series: {series}",
        f"tags: [{tags}]",
        f"description: {description}",
        f"source_url: {source_url}",
        "toc: false",
        f"draft: {draft}",
        "---",
        "",
    ]
    if cardlink_text:
        lines += [cardlink_text, ""]
    lines += [preamble.replace(cardlink_text, "").strip(), ""] if cardlink_text else [preamble, ""]
    lines.append("## 章节")
    lines.append("")
    for title, _ in chapters:
        lines.append(f"- [{title}](<{sanitize(title)}>)")
    lines.append("")
    index.write_text("\n".join(lines), encoding="utf-8")

    # 剩下的图没有被任何一章引用，是旧版本留下的死文件
    dropped = 0
    if images_dir.is_dir():
        for leftover in images_dir.iterdir():
            if leftover.is_file():
                leftover.unlink()
                dropped += 1
        if not any(images_dir.iterdir()):
            images_dir.rmdir()

    print(f"{note_dir.name}: 拆成 1 个目录页 + {len(chapters)} 章，移动 {moved} 张图，删掉 {dropped} 张没被引用的图")


if __name__ == "__main__":
    main()
