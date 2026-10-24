第 6 章 判断力：把工作和知识解耦（V1 00:00-09:50）

- 提问：实习做 KV 量化，只能用 agent 替代工作否则会落后，公司也鼓励用，其他实习生都在用；用起来几乎没收获，写好 prompt 让 agent 自己优化算子或框架 [00:00]
- 算子刚开始的优化还能看懂，迭代几轮性能提升后代码晦涩难懂，只能让 AI 解释它优化了什么；目标是看懂优化过程，但自己写不出这么高效的算子，每天只能从大层面了解知识 [00:20]
- 这个问题正常吗。正常，而且不只是实习生，中学大学、科研、已工作的朋友都会有这种感受：AI 在逐渐替代脑力劳动 [01:35]
- 两拨人：已工作的倾向全员做 agent，只做产出验收和极端情况检查；在学习的倾向弄懂每一行代码，把它学清楚，这是核心竞争力 [02:00]
- 两个想法都对，问题的本质在于你要用 AI 来干什么；他既在实习，公司要的是收益和产出，既要产出又要学到东西是比较困难的 [02:23]
- 人在失败中提升：高中数学周考月考期末考，是为了检查出能力边界；圆锥曲线不会做不会怪立体几何做错，因为知道那是知识面不够，下次提升重点就明确了 [03:13]
- 现在的问题：问 AI 时它没有告诉你这个知识属于哪个范围，范围是人裁定的；一个问题可能同时涉及数据库、环境配置、数据结构、AI 知识，作为学习者就 loss 掉了，完全不知道该学什么 [04:01]
- 第一步是确定问题的边界，别把问题摊大到吸收不了的程度 [04:30]
- 建议：把工作和知识解耦——工作中核心目的是达到公司要求、没有严重性能 bug [04:53]
- 学习要明确核心目标和优化方案，以 50~100 行为单位不断迭代自己的知识；假设现在写不出更高效的算子，就在已有基础上做 50 行的重构迭代 [05:00]
- 分工：概念性问题交给 AI（它冒出来一堆没听过的名词时，搜索让它帮你解决），判断性工作留给自己；自己负责判断，AI 负责 if-else 这类逻辑 [05:41]
- 历史类比：工业革命时的卢德运动，一批纺织工人反抗纺织机；上世纪的新卢德运动里会计等岗位被计算机替代；现在这次替代的是脑力 [06:29]
- AI 的边界是判断能力，分三种人 [07:17]
  - 第一种：能力强，只是不想写代码或觉得工作量太大 [07:30]
  - 第二种：能力不那么强，不知道怎么做，希望 AI 帮忙做；基础较好但面对未知知识时不知道哪些要学、哪些要舍弃 [08:05]
  - 第三种：连自己要做什么都不知道，AI 说什么做什么；主要该提升概念性知识和判断力，判断力的前提是大量阅读 [08:30]
- 结论：不可能抛弃 AI，也不能全让 AI 替我们思考；我去做核心的判断和选择，让 AI 补充相关背景知识 [08:57]

## 术语表（写作时按此纠正 ASR 错字）

| ASR 听到的 | 正确写法 | 说明 |
|---|---|---|
| time parts / temp ars | temparse | template + parse，Python 3.14 的 t-string 模板解析库 |
| 拍抗台湾 / PON台湾 / 拍on台湾 | PyCon Taiwan | 码农高天提到的会议，V2 结尾的宣传段不进正文 |
| REJETS | regex | |
| JASON / J3点漏的S | json / json.loads | |
| GENER | generic | |
| tape check | type check | |
| pass（小写 class） | parse | temparse 里那个小写 class 就是 parse |
| 3.14版本新引入的这个string template | Python 3.14 的 t-string | PEP 750 |
| INFA（视频 1 里的公司名） | 不写公司名，只说「实习的公司做 KV 量化」 | 字幕识别不可靠 |
| APPRETF | approve | |
| cpu这样级别的人 | CTO 这个级别的人 | V3 |
| try cat | try-catch | V3 |
| 探空 | 判空 | V3 |

专有名词标准写法：`Parser`、`Conversion`、`FormatConversion`、`parse`、`Conversion[T]`、`Parser[*T]`、type variable、type checker、t-string、raw string、interpolations、format spec、regular expression、match case、run time、coverage、CI、GitHub workflow、wheel、KV 量化、agent、prompt、卢德运动。

代码标识符：作者仓库是 https://github.com/jiang-zhexin/temparse ，源码在 `src/temparse/__init__.py`；`parse` 的 `__new__` 注释写着 "Used to replace this function until PEP 718 is accepted."，讲泛型时按这句原话引用，不要替它下「PEP 718 是否被接受」的判断。

## 章节产物

每章一个 section 文件：`sections/section_1.md` … `sections/section_6.md`。
第 6 章对应教程的最后一层，从个人学习落到判断力；前五章是 review 本身。

## 图（figure agent 产出）

每章的图放在 `figures/chapter_N/`，manifest 在 `agent_outputs/figures_manifest.json`。
图只保留三类：V2 的代码截图与 README 截图、V3 的标题卡、V1 的三段能力分级文字叠图。
其余 talking-head 帧丢弃。

## 落盘

```
src/content/lecture/zh-cn/AI时代CodeReview教程/
├── index.md                     # 目录页
├── 第 1 章 Code Review 在流程中的位置/index.md + images/
├── 第 2 章 审查清单与沟通心态/index.md + images/
├── 第 3 章 案例：读一个模板解析库的代码/index.md + images/
├── 第 4 章 设计层的分歧：装饰器、interface 与一致性/index.md + images/
├── 第 5 章 工程化收尾：覆盖率、CI，以及 AI 之后剩下的问题/index.md + images/
└── 第 6 章 判断力：把工作和知识解耦/index.md + images/
```


---

## 写作指令（writer agent 必读）

产出文件：`docs/course-drafts/codereview/sections/section_6.md`

正文从 `## 第 6 章 判断力：把工作和知识解耦` 开始，写完整章节，不是摘要。写到最后一个知识点就停，不写章末小结、不写总结、不写过渡到下一章的话。

材料：
- 本章要点：`docs/course-drafts/codereview/agent_outputs/briefs/chapter_6.md`（上面这一节）
- 本章字幕切片（带时间戳）：`docs/course-drafts/codereview/slices/chapter_6.txt`
- 术语表：`docs/course-drafts/codereview/agent_outputs/glossary.md`（专有名词与英文写法一律照它来）
补充材料：
- 讲师的原话：「我去做核心的判断核心的选择，然后让AI来给我补充相关的背景知识就可以了」[08:57]。
- 视频里给出的三层能力分级用「能力 / 基础 / 导向」三组词描述：能力强但工作量大、结果导向；能力一般、基础好、知识导向；能力不行、基础不行、概念导向。截图见 figures。

本章可用的图（放在本章目录的 `images/` 下，正文里单独成段引用，alt 文本写图注，路径写 `images/xxx.png`）：
- `images/fig-chat-advice.png` — 杜子源源给出的建议截图：把工作和知识解耦，以 50 行代码为目标迭代（视频 05:00）
- `images/fig-three-tiers.png` — 视频里给出的三层能力分级：能力、基础、导向（视频 08:00）

硬性要求：
1. 溯源链接只用时间戳一种形状，链接文本写成 `[MM:SS]`，URL 用 `https://www.bilibili.com/video/BV1punn6JEWi/?t=<秒数>`，秒数 = 分×60+秒。至少 3 处，放在知识点后面，不要单独成行。
2. 正文里不要出现这些词（本博客的 lint 会报错）：本节将介绍、接下来、值得注意的是、本章小结、要点回顾、总结、综上所述、总而言之、显然、易证、众所周知、不难看出、大家可以自己验证。
3. 不写 `##` 以外的一级标题，不写「小结 / 总结」类标题。
4. 讲师原话用引用块保留，每一行都以 `>` 开头。引用块里可以出现上面禁用的词。
5. 代码用围栏代码块并标语言。代码前一句说明这段代码在做什么，代码后不要重复叙述代码里已经能看得出的东西。
6. 章节开头直接进入问题，不写「大家好」「本节将介绍」「接下来我们来看」这类开场白。
7. 中文里夹 English 术语（`code review`、`interface`、`t-string`、`type check`、`CI`、`coverage`）。句子短，陈述句优先。不写「不是……而是……」这种转折。
8. 不留 `[cite]`、`TODO`、待补之类的占位符。不保留原始逐字稿。
9. 公式极少用；确需公式时用 `$...$`，且公式前先用一句中文说明它在表达什么。
