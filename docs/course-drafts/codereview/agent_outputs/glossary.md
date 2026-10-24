# 术语表：AI 时代下的 Code Review 教程

写作时按这张表纠正字幕的 ASR 错字。术语在全书保持同一写法。

## 项目与库

| ASR 听到的 | 正确写法 | 说明 |
|---|---|---|
| time parts / temp ars | temparse | template + parse。基于 Python 3.14 的 t-string 做模板解析的库。仓库 https://github.com/jiang-zhexin/temparse ，源码在 `src/temparse/__init__.py` |
| 拍抗台湾 / PON台湾 / 拍on台湾 | PyCon Taiwan | 码农高天在视频结尾提的会议。这段是宣传，不进正文 |

## API 与代码标识符

| 写法 | 说明 |
|---|---|
| `parse` | 库里那个小写开头的 class。ASR 把 class 名听成「pass」，正文一律写 `parse` |
| `Parser` | 编译好的模板解析器 |
| `Conversion` | 自定义转换的装饰器类 |
| `FormatConversion` | 带 format spec 的转换装饰器类 |
| `parse[*T]` / `Parser[*T]` | PEP 695 的类型参数语法 |
| `__new__` | `parse` 里真正干活的入口 |
| `__init__` / `__call__` | `Conversion` 的两个方法，装饰器就是靠它们实现的 |
| `TypeVar` / type variable | 类型变量 |
| type checker | 类型检查器 |

`parse` 的 `__new__` 注释原文是 "Used to replace this function until PEP 718 is accepted."。引用这句原话即可，不要替它判断 PEP 718 现在是什么状态。

## Python 与语言

| ASR 听到的 | 正确写法 |
|---|---|
| REJETS | regex |
| JASON | json |
| J3点漏的S | json.loads |
| GENER | generic |
| tape check | type check |
| t string / teeth string | t-string（Python 3.14 的模板字符串，PEP 750） |
| raw string / interpolations / format spec | 保持英文原样 |

## 中文词

| ASR 听到的 | 正确写法 |
|---|---|
| 探空 | 判空 |
| try cat | try-catch |
| cpu这样级别的人 | CTO 这个级别的人 |
| 8服务给撑爆了 | 把服务给撑爆了 |
| APPRETF | approve |

视频 1 里的公司名（ASR 听成「INFA」）识别不可靠，正文只写「实习的公司做 KV 量化」，不写公司名。

## 其他专有名词

`KV 量化`、`agent`、`prompt`、`卢德运动`、`新卢德运动`、`coverage`、`CI`、`GitHub workflow`、`wheel`、`match case`、`run time`、`format spec`、`regular expression`、`type check`。
