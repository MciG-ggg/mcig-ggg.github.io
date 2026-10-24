#!/usr/bin/env python3
"""Cut the figures this tutorial uses out of the three source videos.

Every figure is checked with `view image` before it lands in the blog.
"""

import subprocess
from pathlib import Path

WORK = Path(__file__).resolve().parent.parent
REPO = WORK.parents[2]
BASE = REPO / "src/content/lecture/zh-cn/CodeReview教程_AI时代下的CodeReview"

# (chapter, filename, video, seconds, crop, caption)
FIGURES = [
    (1, "fig-card-roles.png", "BV1Dh4y1J7iG", 114, None,
     "御风大世界在视频里打出「需要什么人参加？」这一页标题卡"),
    (2, "fig-card-what.png", "BV1Dh4y1J7iG", 175, None,
     "标题卡「到底要review什么？」"),
    (2, "fig-card-mindset.png", "BV1Dh4y1J7iG", 265, None,
     "标题卡「心态很重要！」"),
    (3, "fig-readme-types.png", "BV1RfaX6sESp", 50, "1150:820:0:100",
     "temparse 的 README：Quick Start 与 Supported Types 表"),
    (3, "fig-parser-generics.png", "BV1RfaX6sESp", 130, "1330:790:430:140",
     "Parser 类的源码：类型参数与 cached_property 缓存的正则"),
    (3, "fig-convert-case.png", "BV1RfaX6sESp", 280, "1330:790:430:140",
     "_convert 用 match case 按 origin 分派类型转换"),
    (4, "fig-parse-class.png", "BV1RfaX6sESp", 420, "1330:790:430:140",
     "小写 class parse 的源码与 PEP 718 的注释"),
    (4, "fig-format-conversion.png", "BV1RfaX6sESp", 500, "1330:790:430:140",
     "Conversion 与 FormatConversion 两个装饰器类的实现"),
    (4, "fig-json-case.png", "BV1RfaX6sESp", 280, "1330:300:430:640",
     "match case 的最后一段：list、dict、json 都交给 json.loads"),
    (5, "fig-repo-tree.png", "BV1RfaX6sESp", 880, "1450:700:30:170",
     "temparse 的 GitHub 仓库首页：.github/workflows 只有一个发布流程"),
    (6, "fig-chat-advice.png", "BV1punn6JEWi", 300, "1000:430:0:540",
     "杜子源源给出的建议截图：把工作和知识解耦，以 50 行代码为目标迭代"),
    (6, "fig-three-tiers.png", "BV1punn6JEWi", 480, "1030:340:20:40",
     "视频里给出的三层能力分级：能力、基础、导向"),
]


def main() -> None:
    manifest = []
    for chapter, name, video, second, crop, caption in FIGURES:
        out = BASE / "images" / name
        out.parent.mkdir(parents=True, exist_ok=True)
        cmd = [
            "ffmpeg", "-y", "-hide_banner", "-loglevel", "error",
            "-ss", str(second), "-i", str(WORK / "source" / f"{video}.mp4"),
            "-frames:v", "1",
        ]
        if crop:
            cmd += ["-vf", f"crop={crop}"]
        cmd += [str(out)]
        subprocess.run(cmd, check=True)
        manifest.append(
            {
                "chapter": chapter,
                "id": f"{chapter}-{name[:-4]}",
                "type": "frame",
                "path": f"images/{name}",
                "video": video,
                "interval": f"{second // 60:02d}:{second % 60:02d}",
                "caption": caption,
            }
        )
    (WORK / "agent_outputs" / "figures_manifest.json").write_text(
        __import__("json").dumps({"figures": manifest}, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    print(f"wrote {len(manifest)} figures")


if __name__ == "__main__":
    main()
