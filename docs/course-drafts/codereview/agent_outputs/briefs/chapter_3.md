第 3 章 案例：读一个模板解析库的代码（V2 00:00-04:50）

- 项目名吐槽：叫 temparse，读起来像 temp ars，「临时的屁股」冲击力太大 [00:00]
- 从 README 开始：项目核心是从一个字符串里按模板提取数据；模板里大括号的内容决定怎么从原始字符串变成目标数据 [00:20]
- 类型语义：str 保持原样，int 把字符串变成整数；泛型提示返回值类型，对 type checker 友好 [01:00]
- 代码结构：只有一个文件，全部放在 `__init__` 里；小项目这么做也不是不行，但更推荐分文件；市面上确实有项目这么干 [01:40]
- 核心类 `Parser`：用 type variable 表示输出类型，经典 generic 写法，拆包时 type checker 能检查类型对不对 [01:50]
- 借力 t-string：Python 3.14 引入的 t-string 会先把自己的部分 parse 成 raw string 和 interpolations，一部分 parser 的活交给 t-string 干 [02:20]
- 正则：整个字符串用 regular expression 做 match，提前 compile 完还把 property cache 住；值得学——同一个正则要 match 很多次时不要每次重新 match，先 compile 再 match [02:40]
- parse 部分：把每个 match 到的字符串和对应的 interpolation 成对传给 convert [03:15]
- convert 的两个参数：已匹配上的字符串、原来的模板；命名建议不要用 i，i 常被当 index 或 integer 用，改用 s 更好读 [03:30]
- `i.value` 是大括号里的东西求值之后的值：这里是 str class、int class [04:00]
- 用 match case 分派：origin 是 int 就转成 integer，float、complex 同理；整个代码不到 170 行 [04:20]


---

## 写作指令（writer agent 必读）

产出文件：`docs/course-drafts/codereview/sections/section_3.md`

正文从 `## 第 3 章 读一个模板解析库的代码` 开始，写完整章节，不是摘要。写到最后一个知识点就停，不写章末小结、不写总结、不写过渡到下一章的话。

材料：
- 本章要点：`docs/course-drafts/codereview/agent_outputs/briefs/chapter_3.md`（上面这一节）
- 本章字幕切片（带时间戳）：`docs/course-drafts/codereview/slices/chapter_3.txt`
- 术语表：`docs/course-drafts/codereview/agent_outputs/glossary.md`（专有名词与英文写法一律照它来）
补充材料：仓库源码在 source/temparse___init__.py，README 在 source/temparse_README.md，需要引用代码时照抄这两份文件的真实内容，不要编造行号或变量名。
- `Parser.__init__` 把 template 拆成 raw string 与 `(.+?)` 捕获组，`_match_re` 用 `functools.cached_property` 缓存编译好的正则。

本章可用的图（放在本章目录的 `images/` 下，正文里单独成段引用，alt 文本写图注，路径写 `images/xxx.png`）：
- `images/fig-readme-types.png` — temparse 的 README：Quick Start 与 Supported Types 表（视频 00:50）
- `images/fig-parser-generics.png` — Parser 类的源码：类型参数与 cached_property 缓存的正则（视频 02:10）
- `images/fig-convert-case.png` — _convert 用 match case 按 origin 分派类型转换（视频 04:40）

硬性要求：
1. 溯源链接只用时间戳一种形状，链接文本写成 `[MM:SS]`，URL 用 `https://www.bilibili.com/video/BV1RfaX6sESp/?t=<秒数>`，秒数 = 分×60+秒。至少 3 处，放在知识点后面，不要单独成行。
2. 正文里不要出现这些词（本博客的 lint 会报错）：本节将介绍、接下来、值得注意的是、本章小结、要点回顾、总结、综上所述、总而言之、显然、易证、众所周知、不难看出、大家可以自己验证。
3. 不写 `##` 以外的一级标题，不写「小结 / 总结」类标题。
4. 讲师原话用引用块保留，每一行都以 `>` 开头。引用块里可以出现上面禁用的词。
5. 代码用围栏代码块并标语言。代码前一句说明这段代码在做什么，代码后不要重复叙述代码里已经能看得出的东西。
6. 章节开头直接进入问题，不写「大家好」「本节将介绍」「接下来我们来看」这类开场白。
7. 中文里夹 English 术语（`code review`、`interface`、`t-string`、`type check`、`CI`、`coverage`）。句子短，陈述句优先。不写「不是……而是……」这种转折。
8. 不留 `[cite]`、`TODO`、待补之类的占位符。不保留原始逐字稿。
9. 公式极少用；确需公式时用 `$...$`，且公式前先用一句中文说明它在表达什么。
