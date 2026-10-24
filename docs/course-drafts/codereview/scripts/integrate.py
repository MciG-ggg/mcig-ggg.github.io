#!/usr/bin/env python3
"""Assemble the chapter drafts into one blog page.

The tutorial is a single page. Each chapter draft becomes one `##` section of
that page, and the chapter heading carries an explicit id so the cross-chapter
links point at an anchor instead of another file.
"""

import json
import re
from pathlib import Path

DRAFTS = Path(__file__).resolve().parent.parent
REPO = DRAFTS.parents[2]
META = json.loads((DRAFTS / "agent_outputs" / "page_meta.json").read_text(encoding="utf-8"))
BASE = REPO / "src/content/lecture/zh-cn" / META["directory"]

VIDEO_TITLE = {
    "BV1Dh4y1J7iG": "code review 真的有必要吗，到底在review什么？",
    "BV1RfaX6sESp": "【Code Review】AI时代的CR还有意义么？一起看一个模板解析库",
    "BV1punn6JEWi": "实习/科研/学习中不用AI干不完，用AI写代码又看不懂，我该怎么办？",
}
VIDEO_DESC = {
    "BV1Dh4y1J7iG": "御风大世界讲 code review 在公司流程里的位置、要查什么、以及怎么沟通",
    "BV1RfaX6sESp": "码农高天逐行 review temparse 这个模板解析库，从代码走到设计",
    "BV1punn6JEWi": "杜子源源回答实习生的疑问：AI 时代怎么保住自己的判断力",
}
VIDEO_IMAGE = {
    "BV1Dh4y1J7iG": "http://i1.hdslb.com/bfs/archive/12c0b2d9b6f7329aaa4a9d4a389784b530b05c7f.jpg",
    "BV1RfaX6sESp": "http://i2.hdslb.com/bfs/archive/4e38bd7d15cede74eb1639ee7e6ac7e8adc1c0e7.jpg",
    "BV1punn6JEWi": "http://i1.hdslb.com/bfs/archive/1bb4755a174aa0e23bfdf390f11a33b4c88182f8.jpg",
}

INTRO = [
    "AI 写代码之后，code review 没有变得不重要，只是审的东西换了位置。缩进、变量名格式、重复的样板代码，这些琐碎问题被 AI 加 linter 加 formatter 接管了；design、interface、CI 这些层面的问题浮到水面上，而它们从来不需要 IDE 插件来发现。",
    "这篇教程把三场视频整理在一起读。前两章来自御风大世界，讲 code review 在公司流程里的位置和一份能落地的审查清单。第三到第五章来自码农高天，他当场 review 一个叫 temparse 的模板解析库，从 README 一路读到 `parse` 这个小写 class 的设计理由。第六章是杜子源源回答实习生的问题：AI 写的代码看不懂，问题出在哪。",
]


def cardlink(video: str) -> str:
    return (
        "```cardlink\n"
        f"url: https://www.bilibili.com/video/{video}/\n"
        f'title: "{VIDEO_TITLE[video]}"\n'
        f'description: "{VIDEO_DESC[video]}"\n'
        "host: bilibili.com\n"
        f"image: {VIDEO_IMAGE[video]}\n"
        "```"
    )


def frontmatter() -> str:
    return (
        "---\n"
        f"title: {META['title']}\n"
        f"timestamp: {META['timestamp']}\n"
        f"series: {META['series']}\n"
        f"tags: [{', '.join(META['tags'])}]\n"
        f"description: {META['description']}\n"
        f"source_url: {META['source_url']}\n"
        "toc: true\n"
        f"draft: {str(META.get('draft', True)).lower()}\n"
        "---\n"
    )


def main() -> None:
    chapters = META["chapters"]

    lines = [frontmatter(), ""]

    # The cardlinks follow the order the videos appear in, not the order they
    # were first written down.
    seen: list[str] = []
    for chapter in chapters:
        if chapter["video"] not in seen:
            seen.append(chapter["video"])
    for video in seen:
        lines += [cardlink(video), ""]

    lines += ["\n\n".join(INTRO)]
    lines += [""]
    for chapter in chapters:
        lines.append(f"- [{chapter['title']}](#{chapter['id']}) — {chapter['summary']}")

    for chapter in chapters:
        draft = (DRAFTS / "sections" / f"section_{chapter['n']}.md").read_text(encoding="utf-8")
        draft = draft.strip()
        # The writer starts with the chapter heading; the page repeats it, with
        # the id the cross-chapter links point at.
        draft = re.sub(r"^##\s+.*\n+", "", draft, count=1)
        lines += ["", f"## {chapter['title']} {{#{chapter['id']}}}", "", draft]

    BASE.mkdir(parents=True, exist_ok=True)
    (BASE / "index.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote one page with {len(chapters)} sections")


if __name__ == "__main__":
    main()
