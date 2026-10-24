# 一致性审校报告：AI 时代 Code Review 教程

审校对象：`sections/section_1.md` … `sections/section_6.md`（工作目录 `docs/course-drafts/codereview/`）
参照：`agent_outputs/glossary.md`、`agent_outputs/briefs/chapter_1.md` … `chapter_6.md`、`agent_outputs/outline.md`、`agent_outputs/figures_manifest.json`、`slices/chapter_*.txt`、`source/*.clean.srt`、`source/temparse___init__.py`、`source/temparse_README.md`、`source/temparse_publish.yml`
只读审校，未改动任何章节文件。

问题总数 26：必须改 8、建议改 11、可不改 7。另有 2 条备注，不计入总数。

---

## 1. ASR 错字泄漏进正文

sections/section_1.md:5 — 「上线以后的账单是代码质量不行」里的「账单」是改写残留，原话是「我们发上线以后呢」，语义被改坏 — 改成「发布上线以后，代码质量不行、运行性能很差」【必须改】

sections/section_3.md:9 — 谐音梗写成「tempars」，既不是库名 temparse，也不符本章 brief 与 outline 的「temp ars」，读者会当成第四种拼法 — 改成「像 temp ars」，保留讲师原话的口语梗【必须改】

sections/section_4.md:158 — 「冒号后面的内容会被自动 pass 出来」用 pass 作动词；本章第 7 行刚讲清 `parse` 是小写 class，正文再出现 pass 会被读成 class 名 — 改成「冒号后面的内容会被自动带出来」【必须改】

术语表其余条目逐条核对无残留：体测/题测→提测（section_1.md:3、21、23、26 全对）、探空→判空（section_2.md:23）、try cat→try-catch（section_2.md:35）、cpu这样级别→CTO（section_2.md:63）、APPRETF→approve（section_6.md:58）、REJETS→regex、JASON/J3点漏的S→json/json.loads（section_4.md:184、197、199）、小写 class 未被写成 pass（section_3.md:102、section_4.md:9）、拍抗台湾/PON台湾/拍on台湾 按 glossary 要求不进正文 — 无

---

## 2. 术语不一致与符号漂移

sections/section_1.md:38 — 同一标识符两章写法不同：本章写「user control 下面某一个方法」（两词、无反引号），section_2.md:13 写 `usercontrol` — 改成「`usercontrol` 下面某一个方法」【建议改】

sections/section_4.md:5 — 本章上一行（line 3）用「code review」，全书其余各处也是「code review」，此处改成缩写「CR」 — 改成「code review 剩下的价值只能从项目层面找」【建议改】

sections/section_2.md:35 — 本章内混用两种写法：同一行里「三方 interface」与「接口报错」并存，line 47 又写「interface 路径变更」；第 4、5 章统一用 interface — 把「接口报错」改成「interface 报错」【建议改】

sections/section_3.md:47 — 术语口径与 glossary 及本章 brief 不一致：glossary 把 GENER 定为 generic 并收了 `TypeVar` / type variable，brief 第 3 章写「用 type variable 表示输出类型，经典 generic 写法」；本章改成「PEP 695 的类型参数语法」，另 section_3.md:31、115 与 section_4.md:9、21 用中文「泛型」 — 统一为「type variable（泛型）」与 generic，正文首次出现给英文【建议改】

sections/section_4.md:24 — 同一 class 两章展示不一致：section_3.md:105 的代码块带 `@final`，本章代码块缺 `@final`（真实源码 `source/temparse___init__.py:55` 有） — 补上 `@final`【建议改】

sections/section_2.md:3 — 人物指称与全书不一致：本章用「讲师」四处（3、59、63、69 行），其余章一律写「视频里」「作者」 — 统一成「视频里」，或全书统一一个称法【建议改】

---

## 3. 重复定义（同一概念在两章各写一遍）

sections/section_2.md:5 — 第 5 到 13 行与第 1 章「谁参加」（section_1.md:28-38）重复：开发者先说、QA 在场的理由、一次 review 顺带定下的改动范围，两章引的还是同一句原话（01:40、02:00） — 删掉本章这组重复引文，改成一句承接并指向第 1 章【建议改】

sections/section_3.md:100 — 第 100 到 115 行与 section_4.md:23-30 重复：同一段 `parse` class 代码，同一句 docstring「Used to replace this function until PEP 718 is accepted.」 — 代码块与 docstring 引用留在第 4 章，第 3 章只留一句「小写 class 的理由见第 4 章」【建议改】

sections/section_3.md:156 — 第 156 到 166 行与 section_4.md:80-89 重复：`_convert` 的 match case 代码块 — 第 4 章只保留与 JSON 一致性相关的片段（json.loads 那段），完整分派留第 3 章【建议改】

---

## 4. 图引用完整性

各章正文引用的 `images/xxx.png` 文件名与本章 brief 的可用图清单逐项一致（ch1 一张、ch2 两张、ch3 三张、ch4 三张、ch5 一张、ch6 两张），无跨章引用，无「清单列出但正文没用」的情况，alt 文本与 `figures_manifest.json` 的 caption 对得上 — 无

---

## 5. 溯源完整性

### 5.1 时间戳越出本章切片

sections/section_2.md:7 — `[01:40]` = 100 秒，落在第 1 章切片（0-143 秒），不在本章切片（146-381 秒）；引用内容与第 1 章重复 — 删掉本章这处引文；根因在 brief 第 2 章把 01:40 的原话划进了本章，同步改 brief 或切片起点【必须改】

sections/section_2.md:11 — `[02:00]` = 120 秒，同样落在第 1 章切片，不在本章 146-381 秒内 — 删掉本章这处引文【必须改】

sections/section_2.md:71 — `[06:25]` = 385 秒，超出本章切片上界 381 秒；这句「总的来说就是为人谦虚一点然后多听取别人的建议」在切片 `slices/chapter_2.txt` 里没有 — 把第 2 章切片扩到 390 秒（06:30），或换成切片内的收尾句【必须改】

sections/section_4.md:205 — `[13:40]` = 820 秒，超出本章切片上界 819 秒；被引内容「既然这么在意 type，parse 也应该做 type check」实际落在 13:42-13:55，属第 5 章切片起点，与 section_5.md:7 同源 — 本章切片延到 13:55、第 5 章起点后移，或把这段挪到第 5 章【必须改】

### 5.2 内容无出处

sections/section_2.md:49 — 本章素材是 V3（BV1Dh4y1J7iG）的 02:26-06:21，这段里「linter 和 CI 已经能兜住」「coverage 能量化」在 V3 字幕里不存在，出自 V2 的 04:53 与 14:03 前后 — 整段删除，这个观点归第 4、5 章【必须改】

sections/section_5.md:47 — 「依赖里已经在用 ruff——仓库首页那条提交信息就写着 `build(deps): bump ruff >=0.16.0`」在本章 brief 与 `slices/chapter_5.txt` 里都没有出处（切片只说 workflow 只有 publish） — 用仓库首页截图核对后再保留，否则删掉这处细节，只留「没有测试、没有 lint、没有 format」【建议改】

sections/section_6.md:41 — 第 41 到 48 行的 `git diff --stat HEAD~1`、`git diff HEAD~1 | wc -l` 是编者补充的操作，视频里只给了「50 行这个单位」，正文没标明是补充 — 加一句说明「视频只给了 50 行这个单位，下面是把它落到操作上的做法」【建议改】

### 5.3 形状与秒数一致性

链接文本 MM:SS 与 URL 里 `?t=` 的秒数逐条比对全部一致；全部引用统一为 `[MM:SS](https://www.bilibili.com/video/BV…/?t=N)` 一种形状，无第二种形状，无裸链接（section_5.md:35 的 pypi 链接在 YAML 代码块内，属代码不是引用） — 无

---

## 可不改

sections/section_1.md:26 — `[01:15]` 指的内容实际在 01:37-01:53（改 QA 提的 bug、兼容极端与边界问题、确认功能点都实现）；01:15 是 brief 第 1 章给的切片块起点，未越界 — 可不改

sections/section_6.md:68 — `[06:29]` 指的「这次取代的是脑力」实际在 07:11；06:29 是 brief 第 6 章给的块起点，未越界 — 可不改

sections/section_4.md:174 — `[12:06]` 指的「大括号里是一个 module」实际在 12:39；12:06 是 brief 第 4 章给的块起点，未越界 — 可不改

sections/section_6.md:58 — 分工写「人做 yes-or-no / if-else 判断，approve 交给 AI」，与字幕 06:10-06:20 一致；是 brief 第 6 章那句「AI 负责 if-else」写反了，正文不用改 — 可不改

sections/section_4.md:163 — 代码片段只给了 `(result,) = parse[str](t"{between:2,5}", "abcdefg")`，没有 `between` 的定义；片段本身照抄 README，可不改 — 可不改

sections/section_3.md:150 — 引用块按讲师原话写大写 I、S，正文 line 148 用小写 `i`、`s`（与源码 `def _convert(s: str, i: Interpolation[Any])` 一致）；引语照录原话属正常 — 可不改

sections/section_4.md:32 — 「视频里提到 PEP 718 由 Guido 本人提出」把 docstring 里的 PEP 718 与视频里那句「有一个 pp 是归 do 本人提出的」合并成一句；glossary 只要求不替 PEP 718 判现状，此处只复述视频的「当时尚未被接受」，未越线 — 可不改

---

## 备注

agent_outputs/figures_manifest.json — `figures/` 目录与 PNG 尚未落盘，本次只核对文件名与 brief 清单，无法核对图片内容与 alt 文案；落盘后再核一次 alt 与画面是否对得上

agent_outputs/figures_manifest.json — `4-fig-json-case` 的抓取帧在 04:40（属第 3 章切片末段），图放在第 4 章与 chapter_4 brief 一致，不算跨章
