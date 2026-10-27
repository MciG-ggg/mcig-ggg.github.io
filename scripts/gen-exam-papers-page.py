"""Build the Auto-Drive-Intro mock paper page from the exam blueprints.

The page in `src/content/lecture/zh-cn/Auto-Drive-Intro_模拟卷/index.md` holds
the six exam PDFs and one source table per paper. The source tables come from
the blueprints of the exam, so the page never repeats a mapping that the
blueprint already has.

Usage:

    python3 scripts/gen-exam-papers-page.py

The script reads the blueprints and writes the page. It changes nothing else.
"""
import re
import urllib.parse
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DRAFTS = ROOT / "docs/course-drafts/exam-papers-20261027/build"
PAGE = ROOT / "src/content/lecture/zh-cn/Auto-Drive-Intro_模拟卷/index.md"

ALIST = "http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro"

# The courseware file of every source code, with the folder of the file server
# that holds it.
COURSEWARE = {
    "S01": ("第一章 无人驾驶概述_2026.pdf", "课件"),
    "S02": ("第七章 无人驾驶控制理论.pdf", "课件"),
    "S03": ("第三章 高精度地图.pdf", "课件"),
    "S04": ("第二章 无人驾驶感知基础.pdf", "课件"),
    "S05": ("第二章-行车环境感知-激光雷达和点云.pdf", "课件"),
    "S06": ("第五章 无人驾驶预测系统.pdf", "课件"),
    "S07": ("第五章 无人驾驶预测系统实例.pdf", "课件"),
    "S08": ("第八章 强化深度学习案例.pdf", "课件"),
    "S09": ("第八章 无人驾驶强化学习.pdf", "课件"),
    "S10": ("第四章 无人驾驶定位系统2.pdf", "课件"),
    "S11": ("第四章-汽车智能驾驶定位解决方案.pdf", "课件"),
    "H1": ("作业一：基于百度APOLLO自动驾驶平台的车辆规划可视化与调试.docx", "作业"),
    "H2": ("作业二：基于百度apollo平台的多场景下路径规划.docx", "作业"),
    "H3": ("作业三：基于百度APOLLO平台多场景下的速度规划与数据采集.docx", "作业"),
    "H4": ("作业四：基于百度APOLLO平台的车辆控制结果分析.docx", "作业"),
}

# The file of every paper, with the pages of the blog that hold the file and
# the file name the browser gives to the saved copy.
PAPERS = [
    ("A", "A 卷", "a-paper.pdf", "a-answer.pdf", "智能驾驶导论_A卷_试卷.pdf", "智能驾驶导论_A卷_答案.pdf"),
    ("B", "B 卷", "b-paper.pdf", "b-answer.pdf", "智能驾驶导论_B卷_试卷.pdf", "智能驾驶导论_B卷_答案.pdf"),
    ("C", "C 卷", "c-paper.pdf", "c-answer.pdf", "智能驾驶导论_C卷_试卷.pdf", "智能驾驶导论_C卷_答案.pdf"),
]

# The scope of every paper, written for the reader of the page.
FOCUS = {
    "A": "专题一：ETPP 未知环境遍历、HSGNN 行人轨迹预测、社交空间约束路径规划",
    "B": "专题二：多机器人协同覆盖、HAER-MADDPG 围捕决策、足球机器人协同进攻",
    "C": "课程核心：控制、强化学习、定位、地图、感知与预测",
}


def percent_encode(path: str) -> str:
    """Encode every part of a path for the file server."""
    return "/".join(urllib.parse.quote(part) for part in path.split("/"))


def source_link(token: str) -> str:
    """Turn one source token into a link to the page of the courseware."""
    code, _, pages = token.partition(":")
    if code not in COURSEWARE:
        raise ValueError(f"unknown source code: {token}")
    filename, folder = COURSEWARE[code]
    url = f"{ALIST}/{percent_encode(folder)}/{percent_encode(filename)}"
    first = pages.split(",")[0].split("-")[0]
    return f"[{filename} · p.{pages}]({url}#page={first})"


def read_blueprint(letter: str) -> list:
    """Read the question table of one paper from the blueprint."""
    text = (DRAFTS / "blueprint.md").read_text(encoding="utf-8")
    section = re.search(rf"### {letter}卷\n(.*?)(?=\n### |\n## |\Z)", text, re.S)
    if not section:
        raise ValueError(f"no section for paper {letter}")
    rows = []
    for line in section.group(1).splitlines():
        if not line.startswith("|") or line.startswith("|---") or "题号" in line:
            continue
        cells = [cell.strip() for cell in line.strip("|").split("|")]
        if len(cells) < 7:
            continue
        rows.append(cells)
    if not rows:
        raise ValueError(f"no question rows for paper {letter}")
    return rows


def source_table(rows) -> str:
    """Build the source table of one paper."""
    lines = ["| 题号 | 主题 | 题型 | 分值 | 难度 | 来源 |", "|---:|---|---|---:|---|---|"]
    for number, topic, source, kind, points, difficulty, _ in rows:
        links = "；".join(source_link(token.strip()) for token in source.split(chr(59)))
        lines.append(f"| {number} | {topic} | {kind} | {points} | {difficulty} | {links} |")
    return "\n".join(lines)


def paper_block(letter: str, rows) -> str:
    """Build the section of one paper."""
    name = next(item[1] for item in PAPERS if item[0] == letter)
    paper = next(item[2] for item in PAPERS if item[0] == letter)
    answer = next(item[3] for item in PAPERS if item[0] == letter)
    paper_name = next(item[4] for item in PAPERS if item[0] == letter)
    answer_name = next(item[5] for item in PAPERS if item[0] == letter)
    return f"""## {name}

**侧重：** {FOCUS[letter]}

:::pdf{{src="/pdf/auto-drive-intro/{paper}" title="{name}试卷" download="{paper_name}" height="760"}}
:::

<details>
<summary>{name}答案与评分细则</summary>

:::pdf{{src="/pdf/auto-drive-intro/{answer}" title="{name}答案" download="{answer_name}" height="760"}}
:::

</details>

<details>
<summary>{name}题源对照</summary>

{source_table(rows)}

</details>"""


def build_page(blueprints) -> str:
    """Build the whole page."""
    blocks = [paper_block(letter, blueprints[letter]) for letter, _, _, _, _, _ in PAPERS]
    body = "\n\n".join(blocks)

    return f"""---
title: Auto-Drive-Intro 模拟卷
timestamp: 2026-10-27T15:30:00+08:00
series: Auto-Drive-Intro
tags: [课内, Auto-Drive-Intro]
description: 《智能驾驶导论》期末模拟卷三套，每套 100 分 25 题。试卷与答案各一份 PDF，页面里直接读，也能下载打印。每道题的出处链回课件和作业的对应页。
source_url: http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/课件/
toc: true
draft: false
---

```cardlink
url: http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/课件/
title: "Auto-Drive-Intro · 课程课件与作业"
description: "《智能驾驶导论》的十一份课件与四份 APOLLO 平台作业，模拟卷的取材来源"
host: 10.188.31.55:5244
```

> [!IMPORTANT] 三套卷，每套 100 分 25 题
> 单选 10 题 20 分，填空 10 题 20 分，问答 3 题 30 分，程序 2 题 30 分。建议 120 分钟。

> [!TIP] 先闭卷做，再展开答案
> 答案在每卷末尾的折叠块里。程序题用 Python 3 写，题面给了要用的 NumPy 接口，不用启动 Apollo。

## 三套卷的分工

| 卷 | 取材侧重 | 单选与填空 | 问答 | 程序题 |
|---|---|---|---|---|
| A 卷 | {FOCUS["A"]} | ETPP公式、HSGNN注意力与群组参数计算 | 遍历代价、群组划分、社交空间路径评价 | 稀疏注意力softmax、社交空间路径选择 |
| B 卷 | {FOCUS["B"]} | 覆盖路径数据、HAER-MADDPG与BI-MAPDDPG计算 | 遗传算法负载均衡、经验优先级、足球策略实验 | 覆盖路径分段评价、双通道策略分析 |
| C 卷 | {FOCUS["C"]} | PID、Q-learning、定位与感知计算 | 直方图定位、多模态预测决策、控制策略比较 | 车道掩码拟合、纵向控制误差分析 |

配套复习资料在[《Auto-Drive-Intro 代码题复习》](/lecture/Auto-Drive-Intro_代码题复习)。讲义在[《Auto-Drive-Intro》](/lecture/Auto-Drive-Intro)。

{body}

## 命题边界

- 题目依据十一份课件和四份作业。试卷采用原创样例，不是教师发布的正式考卷。
- 程序题给出必要公式、输入输出和边界条件。程序题不要求安装或启动 Apollo。
- 数值样例、数组、简化代价和验收定义都是原创命题数据。源材料只提供概念和实验背景。
- 题目难度未经真实考生校准。参考程序用合成图像和人工数组自检，不能替代 Apollo 回放、实车实验或算法效果验证。

试卷与答案的 LaTeX 源文件在仓库的 `docs/course-drafts/exam-papers-20261027/`，命题蓝图和核验记录在同目录的 `build/` 下。
"""


def main() -> None:
    blueprints = {letter: read_blueprint(letter) for letter, _, _, _, _, _ in PAPERS}
    PAGE.write_text(build_page(blueprints), encoding="utf-8")
    print(f"wrote {PAGE}")


if __name__ == "__main__":
    main()
