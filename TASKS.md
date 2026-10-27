# TASKS

## 已完成

### 阶段 A：blog token + 动作条 + chip 行

- 合并三份重复调色板为一份 `light-dark()`，修「系统深色时用户选浅色无效」的 bug
- Shiki 改用 `defaultColor: "light-dark()"`，代码块跟随主题，删掉两处 `!important` 覆盖
- 新增 `--accent-color`、alpha 阶梯 `--surface-05..25`、radius 阶梯 `--radius-2..24`、全局 `:focus-visible`
- 新增 `ArticleActions.astro`：复制链接 / 复制 Markdown 原文 / 分享
- note 与 jotting 详情页改用 ChatGPT 风格 chip 元信息行
- 表格边框 3px → 1px，引用块 4px → 2px
- 修四个监听器泄漏：TOC 与 Position 的 scroll 监听、medium-zoom 每次换页叠加 document 监听器、Note/Jotting 内层 `<main>` 与 swup `containers: ['main']` 冲突

### `lecture` collection

- 新增 collection、详情页、列表页、导航入口
- `Note` 列表组件加 `base` 属性，两页共用
- `Section` 类型加 `"lecture"`

### 两个 skill 改造

四个 skill 从「产出 LaTeX → PDF」改为「产出 blog 原生 Markdown，落进 `lecture` collection」。

- 共享写作契约 `_shared/blog-note-contract.md`（落盘位置、frontmatter、章节组织、语气、图的要求、检查清单）
- `youtube-render-pdf` 161 行、`bilibili-render-pdf` 202 行、`drive-render-pdf` 485 行、`review-notes-pdf` 299 行
- 删掉全部 `.tex` 模板和 LaTeX 耦合脚本；`lint_handout.py` → `lint_note.py`，`build_handout.py` → `note_links.py`
- `final-review-suite` 同步更新
- 修 `drive-render-pdf/SKILL.md` 里硬编码的 Google API key（改为从 yt-dlp 源码读）
- alist 地址从 `10.188.31.124`（已下线）改成 `10.188.31.55`

### 两篇笔记

- `vLLM小课堂（十八）RL-Kernel 训推一致性`：B 站 AI 字幕（`ai-zh`，未用 Whisper），15 章、17 张图、36 个可点击时间戳
- `计算智能：进化算法与优化方法`：按课件文件编排，12 章（第 0-10、12 章，课件无第 11 章）、111 小节、65 张图、440 处源页链接、0 死链

### 课程讲义按课件编排

讲义原来只覆盖 12 个课件里的 9 个：开篇被并进第 1 章，约束优化那 3 个课件没写，第 9 章「五大算法横向对照」是讲义自己综合的、没有课件撑着。

- 章节号照抄课件文件编号，文件缺号时笔记也缺号
- 一个课件文件一章，不合并、不丢弃
- 不写没有课件撑着的章：横向对照内容并进第 7 章（算法比较方法），机制对照表移到前置区
- 速查章整章删除：公式速查表、跨章符号表、易混对清单
- 提取课件文本用 python-pptx 直读。`extract_materials.py` 走 soffice，而 soffice 是指向不存在路径的坏 shim，所以它对 pptx 返回 0 个 block

### 顺带修的 blog bug

- 新增 `src/utils/remark/directive-text-guard.ts`：`remark-directive` 会把 `[03:10](url)` 里的冒号当指令标记，链接文字丢掉第一个冒号之后的内容
- 修 i18n 的 `home.note` 缩进错误（被解析成 `home.jotting` 的子键）

### 目录（TOC）三个问题

- **页面被顶**：`scrollIntoView` 会滚动所有祖先容器，包括 body。改成只滚目录面板
- **滚到底不跟随**：`rootMargin: "-10% 0px -70% 0px"` 让观察带只有视口 10%~30%，标题滚出这条带就没有可见项。改成取「视口线上方最后一个标题」，并在页面到底时取最后一个
- **目录太长**：`.toc-nav` 没有高度约束，159 项全部铺开。给它加 `flex: 1; min-height: 0`，父容器加 `min-h-0`

### 课程讲义结构

「按知识结构重组」这条规则只适用于视频，不适用于课程材料。课程章节顺序就是教学大纲，复习的人要把「第 7 章」对到笔记和试卷上。

- 共享契约分两种源类型：视频按知识结构重组，课程保持章节顺序 1:1
- `##` 是章节（带课程章节号），`###` 是小节，章节标题不许丢
- 章节超过五个时开头放章节索引
- 笔记里不写公式速查表、符号表、易混对清单、自测题清单、考前冲刺清单
- `计算智能` 那篇重排成 9 章，速查章删掉

### 第 0 章插入后的第 1 章清理

第 0 章「开篇」按课件 26 页插入后，第 1 章仍带着 9 处指向 `Chapter 0_ 开篇.pdf` 的引用。

- 逐处核对 `Chapter 0_ 开篇.json`（26 页）与 `Chapter 1_ Introduction to EAs.json`（13 页），能对上第 1 章课件页的改指第 1 章，只第 0 章才讲的整段删掉
- 删掉第 1 章的「八类复杂优化问题」一节：第 0 章「课程的九个方向」已完整讲这八类，`可行域 (feasible region)` 英文注记补回第 0 章
- `fig_007`（第 0 章第 7 页课程主线图）和 `fig_006`（第 0 章第 12 页约束优化图）随内容移到第 0 章，图片文件与编号不动
- 改完第 1 章只剩第 1 章课件的引用，字符数 14266 → 12824

### 第 10、12 章公式复原与核对

- 解码 MathType 的 WMF 记录流（记录长度按 16 位字计、`META_EXTTEXTOUT` 紧凑负载、`META_MOVETO` 先行后列），用 `Symbol.ttf` 的 cmap 还原希腊字母与运算符，重画后逐条核对课件 media
- 修掉 4 处：DOC 目标函数里多出的平方根、MOMMOP 缩放因子 $\eta$ 未声明为自选符号、WeB 加权和"落在 0 与 1 之间"的说法错、MCOP 约束下标被说成课件原文

### vLLM小课堂十八 RL-Kernel 训推一致性

- 按讲者的 slide 顺序重排：14 章，章节边界对着 slide 标题逐帧核对，不按知识结构重排
- 12 张图全部从 `video.mp4` 按整秒切（1920×1080 原始分辨率），裁掉播放器的摄像头窗口和底部控制条，逐张目视确认内容完整
- 115 处可点击时间戳，格式统一 `[MM:SS](url?t=秒)`，超过一小时用 `H:MM:SS`
- ASR 纠错：`VARM`/`WIM`/`威尔姆`/`魏母` → vime；`our kernel` → RL-Kernel；`训退` → 训推；`V0.1.0` → v0.1.0；`MC tr`/`麦克TRO` → Megatron-LM；`rock com` → ROCm；`KA` → CUDA；`LMEL` → lm-eval；`BENCHUART` → benchmark
- 逐条核对引用的秒数落在字幕行上；只落在画面上的（如 GitHub 面板的 RFC 验收条款）单独标注
- 补 `###` 小节：14 个 `##` 章各切 2 到 9 个小节，共 53 个 `###`；正文一字未改，115 处时间戳、12 处图引用原样保留，文件只增 889 字符（全部来自标题行与其空行）

### vLLM小课堂（十八）RL-Kernel 训推一致性：按 slide 重排章节

- 把原来按视频内容写的 17 个平铺 `##` 章合并成 12 章（按用户给的 13 段 slide 序列，实际落 13 章），章节边界逐帧核对 8 张 contact sheet
- 每章下把旧章标题降成 `###` 小节，131 行正文一字未改，17 处图引用、36 处时间戳原样保留
- 补上章首索引（13 行，一章一行），frontmatter 与 `cardlink` 块保持原样

### 文档：讲义体系改造方案改写为 as-built 记录

- `docs/lecture-skill-redesign.md` 从「计划」改写成「已建成什么」的记录：每节标已完成/部分完成/未开始
- 新增「施工过程中发现的问题」一节：Astro content store 撞车发旧版、TOC 的 `scrollIntoView` 顶页、MathType 公式读不出、`extract_materials.py` 走坏 soffice 返回 0 block、AList 从 /hdd 搬到 /Nextcloud
- 下一件事：交互模块库仍未提交（`pnpm check` 报 4 个类型错），PDF 导出脚本与 `@media print` 未写

### 《智能驾驶导论》讲义（Auto-Drive-Intro）

三份片段（lecture / slides / homework）拼成一篇讲义，落到 `src/content/lecture/zh-cn/Auto-Drive-Intro/index.md`。

- 3 章（第 2 章 无人驾驶感知基础 / 第 2 章 专题：行车环境感知 / 课后实践：APOLLO 平台大作业）、20 个 `###` 小节、73 张图、109 处源页引用、5 个 mermaid、5 张表
- 拼装时改的两处：slides 的 42 处引用尾部多一个 `]`（`...pptx)]`），会让整段 URL 被当成链接目标；lecture 末尾「四种组织方式怎么解决同一个问题」是跨章速查表，删掉，只留正文
- 裸 URL 紧跟中文时 GFM autolink 会吞掉后面的字（`http://...kitti）和` 一起进 href），改成显式 markdown 链接
- `lint_note.py` 0 错误；73 张图 alt 齐全、文件都在 `images/`；149 处 KaTeX 公式与 5 个 mermaid 全部解析通过

### Auto-Drive-Intro 重写

整门《智能驾驶导论》的六讲课件进了 blog，笔记从 3 章变 7 章。

- 素材：`/Volumes/dav/Nextcloud/School/Auto-Drive-Intro/《智能驾驶导论》PPT-2020/`，六讲 217 张幻灯片 199 张内嵌图。rar 原件也在上一级目录
- 笔记：1145 行，7 章 58 小节 93 图 209 处源页引用，lint 0 错误
- 旧的两章（感知基础 PDF、激光雷达专题）分别并入第 3、4 章
- 引用统一是 `[第N讲 XXX.pptx · p.幻灯片号](AList 的 percent-encoded 路径)`

### 代码题复习资料

老师说要考代码题，把 roadline 的传统车道线检测和四次 APOLLO 作业整理成一篇复习笔记。

- `src/content/lecture/zh-cn/Auto-Drive-Intro_代码题复习/`：5 章 19 小节 17 图 11 段代码 6 张表 12 道自测题
- 传统车道线检测：六步流水线每一步的代码、参数和「调大调小会怎样」
- Apollo 工具链：启动命令、四个调试器、Dreamview 四个 Tab
- 规划：三个子模块、四个场景、速度规划参数、S-T 图的读法
- 控制：纵向 PID 的结构、横向 LQR 的四个权重、参数到曲线的因果链
- 自测题的答案折叠在每道题后面

### quiz 模块与三套模拟卷

- 新增第 8 个交互模块 `quiz`：题干和答案之间用一个 `---` 分隔，答案默认隐藏，每题一个「显示答案」按钮，每卷一个「全部显示答案」按钮
- 外层指令要用 4 个冒号、内层 3 个，否则内层会把外层提前关掉
- `src/content/lecture/zh-cn/Auto-Drive-Intro_模拟卷/`：三套卷，每套 100 分 12 题（选择 5、填空 3、程序 2、解答 2），36 道题，22 处源引用

### quiz 模块升级成可作答、可判分

- 选择题的选项列表变成单选框，填空题里的 `{{blank}}` 变成输入框，程序题和解答题给一个文本域
- 每卷一个「提交答案」按钮：客观题自动判分并标出对错，解答题给出参考答案和「我答对了」的勾选框
- 判分规则抽成纯函数 `src/utils/interactive/grade.ts`，`scripts/test-quiz-grade.ts` 覆盖 15 个用例
- 模拟卷的 36 道题都带答案键，选择 15 道、填空 9 道，程序和解答 12 道留给读者自己判
- **改 remark 插件不会让 Astro 的内容存储失效**，本地验证要用 `pnpm build:clean`（清 `node_modules/.astro/data-store.json`）。CI 的 deploy 工作流已经这么做了

### 计算智能 模拟卷

- `src/content/lecture/zh-cn/计算智能_模拟卷/`：三套卷，每套 100 分 40 题（判断 16、填空 14、编程 4、大题 6），78 处课件页码引用
- 取材：课件十二章 + 老师的 wiki 备考备忘录（代码速通、计算智能计算题、计算智能重点）
- 编程题给的是完整实现：DE/rand/1/bin、NSGA-II 非支配排序与拥挤度、GA 一代、PSO
- 大题是概念计算题和算法流程模拟题：NSGA-II 非支配排序与拥挤度、DE 单步模拟、约束处理三法、NSGA-II 机制、公平比较
- quiz 模块加了「判断」题型：自动生成正确/错误两个选项

### 讲义按章拆页

长讲义一页放不下，读者翻不动。拆成目录页加每章一页。

- `scripts/split-note.py` 做拆分：目录页 + 每章一页，图跟着该章走，路径仍是 `images/xxx.png`
- 拆了三篇：计算智能（12 章）、自动控制原理（9 章）、Auto-Drive-Intro（7 章）
- 页面数从 62 涨到 90
- 顺手修了计算智能笔记的 410 处短格式引用（`[p. 2]` 改成 `[文件名 · p.2]`），这篇笔记从来没通过过 lint
- lint 加了 `--index-page`：目录页没有正文，不需要溯源引用

### AI 时代下的 Code Review 教程

三场 B 站视频合成一篇教程，落在 `src/content/lecture/zh-cn/CodeReview教程_AI时代下的CodeReview/`。**一页**，六章是页内的六个 `##` 小节。

- 源：御风大世界《code review 真的有必要吗》（2023-05-07）、码农高天《【Code Review】AI时代的CR还有意义么？一起看一个模板解析库》（2026-10-03）、杜子源源《实习/科研/学习中不用AI干不完》（2026-10-19）
- 案例库是 temparse（https://github.com/jiang-zhexin/temparse ，Python 3.14 的 t-string 模板解析库），仓库源码抓下来当写作依据
- 六章：流程位置 / 审查清单与沟通心态 / 读代码 / 设计层分歧 / 工程化收尾 / 判断力
- 12 张图全部从视频抽帧，逐张目视检查后裁剪；两个标题卡（需要什么人参加、到底要review什么、的心态很重要）和三段能力分级叠图是唯一值得留的文字画面
- 65 处时间戳溯源，全部回查字幕原文校准秒数
- 摘要在 `docs/course-drafts/codereview/`：outline、glossary、6 份 chapter brief、clean.srt、切片、writer 草稿、consistency 报告、两个脚本
- 审校 agent 报了 26 个问题（8 必须改、11 建议改），自己逐条改完：ASR 错字、切片越界、跨章重复代码块、术语漂移
- 顺带发现 `astro check` 的 Duplicate id 警告是旧目录名残留导致的缓存问题，`pnpm build:clean` 后消失
- 顺带把讲义页面的打印样式补上：`src/styles/print.css` 去掉导航、目录、复制按钮，保留标题与元信息；暗色主题打印时把 `color-scheme` 钉到 light，否则白底浅色字打出来看不见
- 原来拆成目录页加六个章节页，后来合成一页：章标题带 `{#chN}` 显式 id，跨章链接改成页内锚点 `#chN`，12 张图并到一个 `images/`

### skill 同步

- 共享契约 §1 写了拆页结构和目录页写法，§2 加了「tags 只放课内和课程名」，§10 检查清单加了拆页和标签两项
- `review-notes-pdf/SKILL.md` 的产出结构、拓扑、脚本表同步
- `multi-agent-workflow.md` 的拓扑加了 split 这一步

### Auto-Drive-Intro 换成 2026 版课件

主参考从 2020 版六讲换成 2026 版课件（`/Users/mcig/Downloads/智能驾驶导论课件.rar`，11 份 PDF，486 页）。2020 版降级成辅助材料，在正文里被引用，不单独占章。

- 章节结构照抄 2026 版课件的章号：第 1、2、3、4、5、7、8 章。课件没有第六章，笔记也不补
- 几个课件文件共用一个章号（第二章有感知基础与点云两份，第四章、第五章、第八章各两份），一章写两个块，块标题写文件自己的名字，块下的标题降一级
- 旧笔记的第 2 章 无人车基本框架、第 3 章 视觉感知、第 4 章 雷达感知、第 6 章 地图与定位被 2026 版章节取代，这四章的原文件移到 `review-notes-20261024/old_pages/`
- 引用 430 处，全部校验过链接前缀与页码：前缀不符 0 处，页码越界 0 处
- 每章 lint 0 错误；跨章比对 7 章、11 份课件文件，问题 0 处

### Auto-Drive-Intro 只讲到第五章

课程只讲到第五章，后面的四页删掉。

- 删掉 `第 7 章 控制理论`、`第 8 章 强化学习`、`附录 A 路径规划`、`课后实践：APOLLO 平台大作业`
- 目录页的摘要、章节列表、PDF 列表三处各删四条；frontmatter 的 `description` 与 cardlink 描述改成只提前五章
- 四个旧 URL 加进 `astro.config.ts` 的 `redirects`，指向目录页 `/lecture/Auto-Drive-Intro`
- `第 5 章 路径规划` 的旧重定向原本指向 `附录 A 路径规划`，改指目录页
- APOLLO 四次作业的内容在 `Auto-Drive-Intro_代码题复习` 里讲过一遍，控制理论的内容在模拟卷第三卷里，所以删掉不丢考点
- AList `讲义/` 目录里那四份 PDF 没有删，只是目录页不再链过去

### 讲义图片的 alt 文本被截断

重建 Auto-Drive-Intro 时，32 处图片引用的 `![alt` 开头整段丢失，只剩 `](images/xxx.png)`。Markdown 不认这种行，页面上 32 张图一张都不渲染，alt 也没了。

- 图本身是对的，课件整页截图，内容与所在小节对得上。缺的只是 alt
- 逐张目视看图重写 alt，40 到 120 个中文字符，说清图里的关键文字与结构
- `lint_note.py` 查不出这个坏法：缺 `![` 的行对它来说不是图片。新增 `scripts/check-note-images.py` 专门查，挂在 lint-staged 的 `src/content/lecture/**/*.md` 上

### 概率霍夫变换的原理

`Auto-Drive-Intro_代码题复习` 的第四步只列了参数。补一节讲原理。

- 点线对偶：笛卡尔系的点对应霍夫空间的一条线，变量与参数互换，方程从 $y=mx+b$ 变成 $b=-xm+y$
- 竖直线斜率无穷大，改极坐标 $d=x\cos\theta+y\sin\theta$，一个点对应一条曲线
- 累加器投票：曲线交点越多，说明那条线上真有那么多像素，票数过阈值才定一条线
- 概率版与标准版的差别在投票之后：标准版在整张累加器上找局部极大值，概率版从命中点沿候选方向两侧行走，走过的像素从掩码清掉，所以更快
- 长度判定是「或」不是「且」：`abs(x2-x1) >= minLineLength` 或 `abs(y2-y1) >= minLineLength`，源码在 `hough.cpp` 的 `HoughLinesProbabilistic`

### PDF 导出集成进 skill

讲义页面的 PDF 用无头 Chrome 从 blog 构建产物导出，不做二次排版。

- `review-notes-pdf/scripts/export_pdf.py`：在 `127.0.0.1` 起静态服务，让 Chrome 逐页 `--print-to-pdf`。构建产物的资源是绝对路径，`file://` 打开会丢样式，所以必须走 HTTP
- 打印效果由 blog 的 `src/styles/print.css` 决定：藏掉导航、目录栏、复制按钮，暗色主题下把 `color-scheme` 钉到 light
- PDF 发布到 AList 的 `讲义/` 目录，目录页的「PDF 版」一节链过去，路径和文件名都百分号编码
- 目录页自己的 PDF 改名 `Auto-Drive-Intro.pdf`

### PDF 嵌入模块与三套试卷

blog 能把 PDF 嵌在页面里，试卷不再只挂在文件服务器上。

- 新增 `pdf` 交互模块：`src/utils/remark/modules/pdf.ts` 造 `figure.iw-widget.iw-pdf`，里面是 `iframe.iw-pdf-frame` 加两个链接（新标签页打开、下载 PDF）。样式在 `src/styles/modules/pdf.css`，没有客户端 JS
- `:::pdf{src="..." title="..." download="..." height="760"}`。文件放在 `public/pdf/` 下
- `src/styles/markdown.css` 给 `details` 加了折叠块样式。`<details>` 里放 `:::pdf` 和表格都能正常渲染，但 `</details>` 后必须空一行，否则下一个 `##` 标题被吞进折叠块
- 三套试卷的 PDF 放进 `public/pdf/auto-drive-intro/`：A/B/C 卷各一份试卷一份答案
- `Auto-Drive-Intro_模拟卷` 那一页重写成 PDF 版：三卷各一个 `##` 小节，试卷直接嵌，答案与题源对照各放一个折叠块
- 题源对照表由 `scripts/gen-exam-papers-page.py` 从命题蓝图生成。蓝图在 `docs/course-drafts/exam-papers-20261027/build/blueprint.md`，每道题的出处链回课件或作业的对应页
- 试卷的 LaTeX 源文件、程序答案和自检脚本在 `docs/course-drafts/exam-papers-20261027/`
- 旧的 quiz 版三套卷子（可作答、可判分）从这一页移走了，还在 git 历史里（`4f2d41d`）

## 下一步

1. 打开 https://mcig-ggg.github.io/lecture/计算智能_进化算法与优化方法 从目录页进一章看看翻起来是否顺手。
2. `Auto-Drive-Intro_代码题复习` 和两套模拟卷没有拆，模拟卷本来就该整卷做，代码题复习 533 行一页还行。要拆说一声。
3. 交互模块还没在真实文章里用过。挑一篇有数据表的笔记试一次 `:::sortable` 和 `:::chart`。
4. 《Code Review 教程》已从 draft 改成正式发布。改动它的正文用 `docs/course-drafts/codereview/scripts/integrate.py` 重新生成，不要手改已发布的那页。
5. `Auto-Drive-Intro` 的 PDF 已发布到 AList `讲义/` 目录，浏览器打开 `http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/讲义/` 看得到列表。删章后 AList 上还留着第 7、8 章、附录 A、课后实践四份旧 PDF，目录页不再链过去，文件本身没有删。
6. AList 上的 PDF 已重导并上传。目录页与五章各一份，每章 4 到 8 张图都进 PDF 了。旧 PDF 一个图都没有：那些页面当时图片引用缺 `![` 开头，打印出来是空白。
7. AList 上 `第 7 章 控制理论.pdf`、`第 8 章 强化学习.pdf`、`附录 A 路径规划.pdf`、`课后实践：APOLLO 平台大作业.pdf` 四份保留着，目录页不再链过去，文件本身没删。
8. 三套试卷的 PDF 已进仓库的 `public/pdf/auto-drive-intro/`，部署后 `/lecture/Auto-Drive-Intro_模拟卷` 上能直接读。AList 上还没有这三套卷子的文件，要同步的话用 `review-notes-pdf/scripts/publish_alist.py`。
- 2026-10-27 本轮完成：修复 B19 题干超宽和 A21 评分拆分，并重编六份 PDF；三套新卷、命题蓝图、题源页面和站点已构建。来源、难度、程序自检及 PDF 副本已核验。下一步：部署后打开模拟卷页面，检查 PDF 嵌入和下载。
