## 第 3 章 读一个模板解析库的代码

Code review 的第一份材料是 README。它说明这个库要解决什么问题，代码只是那个问题的实现。

这一章 review 的对象叫 temparse，名字来自 template + parse。项目规模很小，全部代码放在一个文件里。

> 今天我们要 review 的项目呢是一个很小的项目啊叫 temparse。

> 这个名字怎么读怎么觉得像 temp ars，就是临时的屁股，这个词儿冲击力还是太大了。

名字之外，这个库值得看的理由是它小。代码短到能整段读完，review 的成本低，适合拿来练手。

### README 说了什么

项目的核心是：从一个字符串里按模板提取数据。模板里写明想提取的内容；大括号里的内容决定原始字符串怎么变成目标数据。[00:37](https://www.bilibili.com/video/BV1RfaX6sESp/?t=37)

README 给的例子：

```python
from temparse import parse

city, year = parse[str, int](
    t"I live in {str}, since {int}", "I live in Tokyo, since 2010"
)
assert city == "Tokyo"
assert year == 2010
```

模板里的 `{str}` 让那段文字原样保留，`{int}` 把那段文字转成整数。所以 year 拿到的是 `2010` 这个 int，模板里写 `{str}` 时拿到的仍是字符串。

类型参数 `parse[str, int]` 提示返回值的类型，对 type check 友好。[01:19](https://www.bilibili.com/video/BV1RfaX6sESp/?t=79)

![temparse 的 README：Quick Start 与 Supported Types 表](images/fig-readme-types.png)

Supported Types 表列了支持的类型：`str`、`int`、`float`、`complex`、`list`、`dict`、`json`、`datetime.datetime`、`datetime.date`、`datetime.time`。表下面有一条 IMPORTANT：list 和 dict 只是 json 的别名。

### 代码结构：一个文件

全部代码放在 `__init__.py` 里。

> 他的代码本身 review 起来非常轻松啊，因为只有一个文件，他把所有的东西都放到了 INIT 里。在这个项目非常小的情况下这样做也不是不行，但是我还是更推荐大家把它们分别放在不同的文件里啊，不过市面上有一些项目也确实是这么干的。

项目小的时候单文件不算错。规模上去之后，`Conversion`、`FormatConversion` 这些装饰器类和 `parse` 放在同一个文件里，导入路径会变长。[01:40](https://www.bilibili.com/video/BV1RfaX6sESp/?t=100)

### 核心类 Parser

`Parser` 用一个 type variable 表示输出类型，类型参数由调用方给出（PEP 695 的类型参数语法）：

```python
@final
class Parser[*T]:
```

docstring 第一句：

> A compiled template parser that can be reused to parse multiple strings.

写成 `Parser[str, int, float]` 之后，拆包时 type checker 能检查每个位置的类型对不对。[02:15](https://www.bilibili.com/video/BV1RfaX6sESp/?t=135)

`__init__` 把 template 拆成 raw string 与 `(.+?)` 捕获组，拼成一个正则表达式：

```python
    def __init__(self, template: Template, flags: re.RegexFlag = re.NOFLAG) -> None:
        if len(template.interpolations) == 0:
            raise ValueError("template must contain at least one interpolation")

        self._template = template
        self._flags = flags
        self._expression = "".join(
            re.escape(item) if isinstance(item, str) else r"(.+?)" for item in template
        )
```

模板里的字面量部分走 `re.escape`，占位部分换成非贪婪捕获组 `(.+?)`。这个字符串就是后面 match 用的模式。

### 借力 t-string

Python 3.14 的 t-string 会先把自己的内容 parse 成 raw string 和 interpolations，一部分 parser 的活交给 t-string 干。[02:21](https://www.bilibili.com/video/BV1RfaX6sESp/?t=141)

> 比如说例子里的这个 tstring，它可以直接从 Python 端拿到哪些是这种 raw string，哪些是你需要替换的这些模板，包括这个大括号里面。

所以这个库里没有一个手写的 tokenizer。切分模板这件事由 t-string 完成，剩下的活是：把 interpolation 变成正则，用正则去 match 字符串，再把 match 到的文本变成目标类型。

### 正则提前 compile

整个字符串用 regular expression 做 match。编译好的正则用 `functools.cached_property` 缓存住：

```python
    @functools.cached_property
    def _match_re(self) -> re.Pattern[str]:
        return re.compile(self._expression, self._flags)
```

这一点值得学：同一个正则要 match 很多次时，不要每次都重新 match，先 compile 再拿编译结果去 match，能省掉不少时间。[03:01](https://www.bilibili.com/video/BV1RfaX6sESp/?t=181)

`cached_property` 用在这里有两个好处：正则只在第一次用到时编译；每个 `Parser` 实例最多编译一次。

![Parser 类的源码：类型参数与 cached_property 缓存的正则](images/fig-parser-generics.png)

### parse 这个入口

对外暴露的 `parse` 是一个小写开头的 class，调用起来像函数：

```python
@final
class parse[*T]:
    def __new__(cls, t: Template, s: str, flags: re.RegexFlag = re.NOFLAG) -> tuple[*T]:
        return Parser[*T](t, flags).parse(s)
```

小写命名让 `parse(...)` 读起来仍然像函数调用，同时保留 `parse[str, int]` 这种类型参数下标的能力。

### parse 把 match 结果交给 convert

`parse` 方法用 `fullmatch` 匹配整个字符串，匹配不上就抛 `ValueError`；匹配成功时，把每一组捕获到的文本和对应的 interpolation 成对交给 `_convert`：

```python
    def parse(self, s: str) -> tuple[*T]:
        m = self._match_re.fullmatch(s)
        if m is None:
            raise ValueError(f"invalid format: {s!r}")

        return cast(
            tuple[*T],
            tuple(
                _convert(text, interpolation)
                for text, interpolation in zip(
                    m.groups(), self._template.interpolations
                )
            ),
        )
```

`zip` 把捕获组和 interpolation 对齐。这里有个隐含前提：每个 interpolation 在 `__init__` 里恰好生成一个捕获组，所以两边的数量天然一致。改动 `__init__` 的拼接逻辑时，这份前提会被打破，`zip` 会静默截断到较短的一边。这个耦合关系在代码里没有写出来。

### _convert 的两个参数

`_convert` 收两个参数：已经 match 上的字符串，和原来的模板。

```python
def _convert(s: str, i: Interpolation[Any]) -> Any:
```

字符串这个参数用 `s` 表示，这是常见写法。第二个参数作者起名 `i`，这里有一条命名建议：这个参数最好不要叫 `i`，`i` 太常被当 index 用，有时候又被当成 integer，看到 `i.` 后面跟什么都要在脑子里翻译一下。

> 这个 argument 最好不要叫 I，用 S 来表示 string，这个事很常见，但是 I 这个字母啊它太常被用来做 index 了，有些时候呢还被当成 integer 来用，所以我每一次后面看那个 I 点什么的时候我脑子都要反应半天，这里为了可读性建议改一下。

`i.value` 是大括号里的东西求值之后的值。在这个例子里，`{str}` 求值得到 `str` 这个 class，`{int}` 求值得到 `int` 这个 class。[04:17](https://www.bilibili.com/video/BV1RfaX6sESp/?t=257)

### match case 分派类型

`_convert` 先算出 origin，再按 origin 分派：

```python
    origin = get_origin(i.value) or i.value
    match origin:
        case Conversion():
            return i.value(s)

        case FormatConversion():
            return i.value(s, i.format_spec)
```

`Conversion` 和 `FormatConversion` 是两个装饰器类：前者包一个 `s -> T` 的函数，后者包一个 `(s, spec) -> T` 的函数，把用户自己写的转换接进模板。

类型转换本身是几个 case：

```python
        case _ if origin is int:
            return int(s, base=int(i.format_spec) if i.format_spec else 0)

        case _ if origin is float:
            return float(s)

        case _ if origin is complex:
            return complex(s)

        case _ if origin is str:
            return s
```

origin 是 int 就转成整数，float、complex 同理。int 这一支还读了 format spec 当 base，所以 `{int:16}` 能解析十六进制。末尾还有一个 `case _` 兜底，遇到不认识的对象抛 `TypeError`。

![_convert 用 match case 按 origin 分派类型转换](images/fig-convert-case.png)

用 match case 写在这里比一串 if 判断好读：每个分支的条件写在分支头上，一眼能看出顺序，也能看出 `Conversion()` 这种类模式匹配走在前面。

整个文件不到 170 行，一个挺有趣的小功能就做完了。[04:44](https://www.bilibili.com/video/BV1RfaX6sESp/?t=284)
