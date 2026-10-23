#!/usr/bin/env python3
"""Assemble the Auto-Drive-Intro note from the chapter fragments.

The fragments in docs/course-drafts/adr/ are the source. This script writes
the frontmatter, the card link and the chapter index, then concatenates the
fragments in chapter order into the published note. Edit a fragment and rerun
the script; do not edit the published note by hand.

Usage: python3 scripts/assemble-note.py
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
DRAFTS = REPO / "docs/course-drafts/adr"
NOTE = REPO / "src/content/lecture/zh-cn/Auto-Drive-Intro/index.md"
ALIST = "http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro"

FRONTMATTER = """---
title: Auto-Drive-Intro
timestamp: 2026-10-23T11:00:00+08:00
series: Auto-Drive-Intro
tags: [自动驾驶, 智能驾驶, 环境感知, 视觉感知, 激光雷达, 点云, 路径规划, 地图构建, SLAM, 定位, V2X, Apollo]
description: 中南大学《智能驾驶导论》全六讲的读书笔记：智能驾驶概述、无人车基本框架、视觉感知、雷达感知、路径规划、地图与定位，附 APOLLO 平台上的四次大作业。每处结论都链回课件的对应幻灯片。
source_url: {alist}/《智能驾驶导论》PPT-2020/
toc: true
draft: false
---

```cardlink
url: {alist}/《智能驾驶导论》PPT-2020/
title: "智能驾驶导论 · 全六讲课件"
description: "中南大学人工智能与机器人实验室《智能驾驶导论》的六讲课件"
host: 10.188.31.55:5244
```
"""

INDEX = """
- 第 1 章 智能驾驶概述 — 什么是智能驾驶、SAE 分级、八条优点与四类风险、全球竞争格局与主要玩家
- 第 2 章 无人车基本框架 — 整体框架、硬件与软件架构、定位技术、高精地图、V2X、决策规划与控制
- 第 3 章 视觉感知 — 机器视觉流程、智能驾驶中的视觉感知任务、深度学习网络
- 第 4 章 雷达感知 — 毫米波与激光雷达原理、传感器部署、可行区域检测、目标识别四方案、点云处理
- 第 5 章 路径规划 — 全局与局部规划、PRM/RRT/A*、Frenet 坐标系、轨迹生成与损失函数
- 第 6 章 地图与定位 — 地图表示、定位技术、IMU 与误差、GPS-IMU 融合、SLAM
- 课后实践：APOLLO 平台大作业 — 车辆规划可视化、路径规划、速度规划与数据采集、控制结果分析
"""

# The fragments in chapter order. The homework fragment comes last.
ORDER = ["part1.md", "part2.md", "part3.md", "part4.md", "part5.md", "part6.md", "part7.md"]


def main() -> None:
    missing = [name for name in ORDER if not (DRAFTS / name).exists()]
    if missing:
        sys.exit(f"error: fragment missing: {', '.join(missing)}")

    parts = [(DRAFTS / name).read_text(encoding="utf-8").strip() for name in ORDER]
    body = "\n\n".join(part for part in parts if part)

    NOTE.write_text(FRONTMATTER.format(alist=ALIST) + INDEX + "\n" + body + "\n", encoding="utf-8")
    print(f"wrote {NOTE} ({len(NOTE.read_text(encoding='utf-8').splitlines())} lines)")


if __name__ == "__main__":
    main()
