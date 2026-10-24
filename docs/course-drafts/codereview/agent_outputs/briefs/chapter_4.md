第 4 章 设计层的分歧：装饰器、interface 与一致性（V2 04:53-13:39）

- 引子：AI 加持下很难像以前的 review 那样找细节小问题，AI 加 linter 加 formatter 把蠢问题都消掉了；但 CR 依然有意义，问题从项目角度看 [04:53]
- `parse` 为什么是小写 class：作者故意的，想让用户概念里它是 function；因为当时的 Python 版本没法给函数直接加泛型，这个提案是 Guido 本人提的、还没被接受；为了保住 typing 做的一个 hack [05:40]
- 不一致：`Conversion` 到了装饰器那块却又变成大写开头的 class [06:30]
- decorator 怎么工作：只定义了 `__init__` 和 `__call__`，把圈出来的部分改写一下就明白了 [06:45]
- `FormatConversion`：专门对付带 format 的情况 [07:19]
- 例子：`percent` 把百分号字符串转成 float；除了 `==` 用得不好（浮点世界尽量不写等号）[07:20]
- 追问：这个 decorator 带来了什么？转换完全由大括号里的东西决定，大括号里是 percent 就全局找 percent 函数，把 30% 喂进去 [07:45]
- 作者给 Conversion 找的两个作用：显式规定「percent 是一种转换方式」；规定函数的 argument 数量 [08:06]
- 反驳一：不需要显式规定是 conversion，用户给什么函数就用什么函数转换 [08:30]
- 反驳二：type check 角度没有额外信息，原函数本来就带类型；argument 数量、有没有 format 运行时都能知道 [08:54]
- 结论：conversion 与 format conversion 在设计上压根不该存在，用户直接写一个 conversion 函数传进去即可 [09:20]
- 另一个设计问题：最基础的例子里 string 出现两次、int 也出现两次——要把数转成整数这么常见的事竟然要写两次 int [09:42]
- 泛型那部分可能是必须的（给 type checker 用），但大括号里也必须写吗？既然已经在模板上写了要返回什么值，能不能有默认方式 [09:42]
- interface 设计原则：没有歧义的情况下让用户输入的东西越少越好；同样的数据不要让用户输两次 [10:00]
- 直觉冲突：f-string、t-string 里大括号内容一般表示「要转换什么变量」，作者的设计里大括号里的 percent 表示的是「转换这个 30% 的方式」，方向反了；不熟悉这个库的人第一反应一定是「30% 对应 percent」[10:30]
- 作者的替代方案：t-string 本身提供了 format spec，冒号后面会自动 pass 出来，冒号后面代表格式转换，可以定义「这样就代表我用这个函数转换」[11:00]
- consistency 问题：interface 给用户之后一致性要好，用户顺着你的逻辑应该能明白 [11:30]
- 反例点名 JSON：把 json 扔到大括号里、用 json.loads 把字符串变 dict 或 list；但 json 是 module，没法把字符串 feed 给 module，等于自己开辟了一个新语义 [12:06]
- 修法：用 json.loads 就够了，不用专门给它规定语义 [12:54]
- 还有 list、dict 与 json 语义一模一样，怪；如果支持 list 和 dict，应该做一些 run time 的 type check [13:00]
- 顺带：既然这么在意 type，parse 也应该做 type check [13:40]


---

## 写作指令（writer agent 必读）

产出文件：`docs/course-drafts/codereview/sections/section_4.md`

正文从 `## 第 4 章 设计层的分歧：装饰器、interface 与一致性` 开始，写完整章节，不是摘要。写到最后一个知识点就停，不写章末小结、不写总结、不写过渡到下一章的话。

材料：
- 本章要点：`docs/course-drafts/codereview/agent_outputs/briefs/chapter_4.md`（上面这一节）
- 本章字幕切片（带时间戳）：`docs/course-drafts/codereview/slices/chapter_4.txt`
- 术语表：`docs/course-drafts/codereview/agent_outputs/glossary.md`（专有名词与英文写法一律照它来）
补充材料：仓库源码在 source/temparse___init__.py，README 在 source/temparse_README.md。
- `parse.__new__` 的 docstring 原话："Used to replace this function until PEP 718 is accepted."，引用这句英文原话时不要改动，也不要替它判断 PEP 718 的当前状态。
- `_convert` 里 `case _ if (origin is list or origin is dict or origin is json or is_typeddict(origin)): return json.loads(s)` 是 JSON 一致性问题的代码依据。

本章可用的图（放在本章目录的 `images/` 下，正文里单独成段引用，alt 文本写图注，路径写 `images/xxx.png`）：
- `images/fig-parse-class.png` — 小写 class parse 的源码与 PEP 718 的注释（视频 07:00）
- `images/fig-format-conversion.png` — Conversion 与 FormatConversion 两个装饰器类的实现（视频 08:20）
- `images/fig-json-case.png` — match case 的最后一段：list、dict、json 都交给 json.loads（视频 04:40）

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
