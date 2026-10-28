# 智能驾驶导论三套试卷

打开对应的试卷 PDF 独立作答，再用答案 PDF 评分。每套 100 分，建议 120 分钟。

| 题型 | 题数 | 每题分值 | 小计 |
|---|---:|---:|---:|
| 单项选择 | 10 | 2 | 20 |
| 填空 | 10 | 2 | 20 |
| 问答 | 3 | 10 | 30 |
| 程序 | 2 | 15 | 30 |

| 试卷 | 覆盖范围 | 第 24 题 | 第 25 题 |
|---|---|---|---|
| A | 第 1 至第 8 章，每章至少两题 | 三维点云最近邻距离 | 社会空间轨迹平滑与时间联合代价 |
| B | 第 1 至第 8 章，每章至少两题 | 点云半径离群点过滤 | ETPP 道路图扩展代价选择 |
| C | 第 1 至第 8 章，每章至少两题 | 二维覆盖路径长度 | MPC 预测时域状态跟踪代价 |

题目依据用户提供的 11 份课件和 4 份作业。试卷采用原创样例，不是教师发布的正式考卷。
程序题给出必要公式、输入输出和边界条件。程序题不要求安装或启动 Apollo。
题目难度未经真实考生校准。

## 文件

- [main.tex](main.tex) 是 LaTeX 入口，默认构建 A 卷试卷。
- [A.tex](A.tex)、[B.tex](B.tex)、[C.tex](C.tex) 各自保存一套题目和答案。答案开关控制输出。
- [programs](programs) 保存六份程序答案和自检。答案 PDF 直接读取程序文件。
- [snippets](snippets) 保存题面 API、输入样例和输出样例。
- [命题蓝图](build/blueprint.md) 给出逐题来源；[核验记录](build/核验记录.md) 给出检查结果和限制。

## 构建 PDF

在本目录运行 XeLaTeX。以下示例构建 A 卷试卷。连续运行两次以更新总页数。

~~~sh
xelatex -interaction=nonstopmode -halt-on-error -jobname=A_exam '\def\Paper{A}\input{main.tex}'
xelatex -interaction=nonstopmode -halt-on-error -jobname=A_exam '\def\Paper{A}\input{main.tex}'
~~~

构建 A 卷答案时启用答案开关。B、C 卷只需替换 Paper 和 jobname 中的字母。

~~~sh
xelatex -interaction=nonstopmode -halt-on-error -jobname=A_key '\def\Paper{A}\def\ShowAnswers{1}\input{main.tex}'
xelatex -interaction=nonstopmode -halt-on-error -jobname=A_key '\def\Paper{A}\def\ShowAnswers{1}\input{main.tex}'
~~~

模板使用 TeX Live 中的 Fandol 中文字体。无需项目外的图片。

## 检查程序答案

使用 Python 3，并安装 NumPy 与 OpenCV。以下命令运行本套程序自检。

~~~sh
python3 -m pip install -r requirements.txt
python3 programs/check_programs.py
~~~

自检使用合成图像和人工数组。它检查题面算法与边界约定，不验证真实道路上的检测效果，
也不替代 Apollo 仿真或实车实验。
