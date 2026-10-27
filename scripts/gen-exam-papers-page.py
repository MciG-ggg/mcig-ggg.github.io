"""Build the Auto-Drive-Intro mock paper page from the exam blueprint.

The page in `src/content/lecture/zh-cn/Auto-Drive-Intro_模拟卷/index.md` holds
the six exam PDFs and one source table per paper. The source tables come from
the blueprint of the exam, so the page never repeats a mapping that the
blueprint already has.

Usage:

    python3 scripts/gen-exam-papers-page.py

The script reads the blueprint and writes the page. It changes nothing else.
"""
import re
import urllib.parse
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
BLUEPRINT = ROOT / "docs/course-drafts/exam-papers-20261027/build/blueprint.md"
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


def read_blueprint() -> dict:
    """Read the question table of every paper."""
    text = BLUEPRINT.read_text(encoding="utf-8")
    papers = {}
    for letter, _, _, _, _, _ in PAPERS:
        section = re.search(rf"### {letter}卷\n(.*?)(?=\n### |\n## |\Z)", text, re.S)
        if not section:
            raise ValueError(f"no section for paper {letter}")
        rows = []
        for line in section.group(1).splitlines():
            if not line.startswith("|") or line.startswith("|---") or "题号" in line:
                continue
            cells = [cell.strip() for cell in line.strip("|").split("|")]
            rows.append(cells)
        papers[letter] = rows
    return papers


def source_table(rows) -> str:
    """Build the source table of one paper."""
    lines = ["| 题号 | 主题 | 题型 | 分值 | 来源 |", "|---:|---|---|---:|---|"]
    for number, topic, source, kind, points, _, _ in rows:
        links = "；".join(source_link(token.strip()) for token in source.split(";"))
        lines.append(f"| {number} | {topic} | {kind} | {points} | {links} |")
    return "\n".join(lines)


def build_page(papers) -> str:
    """Build the whole page."""
    blocks = []
    for letter, name, paper, answer, paper_name, answer_name in PAPERS:
        blocks.append(
            f"""## {name}

:::pdf{{src="/pdf/auto-drive-intro/{paper}" title="{name}试卷" download="{paper_name}" height="760"}}
:::

<details>
<summary>{name}答案与评分细则</summary>

:::pdf{{src="/pdf/auto-drive-intro/{answer}" title="{name}答案" download="{answer_name}" height="760"}}
:::

</details>

<details>
<summary>{name}题源对照</summary>

{source_table(papers[letter])}

</details>"""
        )

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
description: "《智能驾驶导论》的十一份课件与四次 APOLLO 平台作业，模拟卷的取材来源"
host: 10.188.31.55:5244
```

三套卷，每套 100 分 25 题：单选 10 题 20 分，填空 10 题 20 分，问答 3 题 30 分，程序 2 题 30 分。建议 120 分钟。

**答案在每卷末尾的折叠块里。** 闭卷做完再展开对答案。程序题用 Python 3 写，题面给了要用的 NumPy 与 OpenCV 接口，不用启动 Apollo。

| 卷 | 侧重 | 第 24 题 | 第 25 题 |
|---|---|---|---|
| A 卷 | 控制、车道线检测、高精地图、定位、预测、强化学习、Cyber 通道 | 灰度、高斯平滑、Canny、ROI 与 Hough | 路径采样边界与候选代价 |
| B 卷 | 点云、Frenet、多源定位、预测、速度规划、LQR 与 PID、Cyber 回放 | Hough 线段筛选与左右线合并 | 速度差分、阈值与距离一致性 |
| C 卷 | 语义分割、地图验证、SLAM、预测、泊车、MPC、强化学习、控制参数 | 分割掩码拟合与车道中心偏移 | 控制误差、超调与窗口内稳定时刻 |

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
    papers = read_blueprint()
    PAGE.write_text(build_page(papers), encoding="utf-8")
    print(f"wrote {PAGE}")


if __name__ == "__main__":
    main()
