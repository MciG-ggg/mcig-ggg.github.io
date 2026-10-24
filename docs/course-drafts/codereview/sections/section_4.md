## 第 4 章 设计层的分歧：装饰器、interface 与一致性

AI 加上 linter 和 formatter 之后，code review 里那类细节小问题基本消失了。缩进对不齐、空行少一个、命名风格跑偏，工具都会先拦一道。[04:53](https://www.bilibili.com/video/BV1RfaX6sESp/?t=293)

code review 剩下的价值只能从项目层面找。这一章拿 temparse 这个库当例子，看三个设计分歧：装饰器为什么存在，类型参数为什么要写两遍，同一套语义在不同位置有没有保持一致。

### `parse` 为什么是小写 class

temparse 里那个转换入口叫 `parse`，第一眼看上去违和：它带方括号类型参数，写法上和 class 一样，但名字是小写。

作者是故意的。目的只有一个：让使用者的概念里它是一个 function。[05:33](https://www.bilibili.com/video/BV1RfaX6sESp/?t=333)

```python
from temparse import parse

city, year = parse[str, int](
    t"I live in {str}, since {int}", "I live in Tokyo, since 2010"
)
```

读法就是调用一个函数。可 Python 当时的版本没法给函数直接加类型参数，于是作者把它做成 class，用 `__new__` 顶替函数入口（源码见[第 3 章 读一个模板解析库的代码](#ch3)）。

`__new__` 的 docstring 把这个取舍写清楚了：等到 PEP 718 被接受，这个 class 就由同名 function 替回去。视频里提到 PEP 718 由 Guido 本人提出，当时尚未被接受。

![小写 class parse 的源码与 PEP 718 的注释](images/fig-parse-class.png)

> 他为了保这个 typing 去做一些看起来有点奇怪的这个 hack

这个 hack 可以接受。评审时该看的是「作者想清楚了取舍，并且留了退路」。留了退路，说明作者知道自己在绕路，绕路也有终点。

### `Conversion` 却是大写 class

紧跟着的分歧在同一份文件的另一头。`Conversion` 和 `FormatConversion` 都是大写开头的 class。[06:30](https://www.bilibili.com/video/BV1RfaX6sESp/?t=390)

> 这个 conversion 它是个 decorator 对吧，为什么到了 decorate 这块又变成了一个正常的大写开头的 class 了呢

同一个文件里，`parse` 小写是为了伪装成 function，`Conversion` 大写是老老实实承认自己是个 class。命名在这里同时承担了两件事：告诉读者这是 class，以及告诉读者它该被 `@` 装饰。作者在 `parse` 上选了伪装，在 `Conversion` 上选了诚实。评审至少应该指出这个不一致，逼作者说明两处取舍的理由。

### 装饰器的实现

这两个装饰器类和 Python 的 `functools` 装饰器写法没什么关系。整个类只定义了两个方法：`__init__` 存住被装饰的函数，`__call__` 负责调用。[06:45](https://www.bilibili.com/video/BV1RfaX6sESp/?t=405)

```python
@final
class Conversion[T]:
    def __init__(self, func: Callable[[str], T]) -> None:
        self._convert = func
        functools.update_wrapper(self, func)

    def __call__(self, s: str) -> T:
        return self._convert(s)


@final
class FormatConversion[T]:
    def __init__(self, func: Callable[[str, str], T]) -> None:
        self._convert = func
        functools.update_wrapper(self, func)

    def __call__(self, s: str, format: str) -> T:
        return self._convert(s, format)
```

`__init__` 存下来的对象本身是个 callable，把它当函数传给模板，模板在 run time 会用字符串去调用它。`FormatConversion` 多一个参数，用来接 format spec。

![Conversion 与 FormatConversion 两个装饰器类的实现](images/fig-format-conversion.png)

模板侧的分派写在 `_convert` 里，用 match case 逐个匹配：`Conversion` 命中第一个分支，`FormatConversion` 命中第二个分支，完整的分派表在[第 3 章 读一个模板解析库的代码](#ch3)。

README 里的自定义转换例子把这条路径讲得最清楚。

```python
from temparse import Conversion, parse


@Conversion
def percent(s: str) -> float:
    return float(s.rstrip("%")) / 100


(result,) = parse[float](t"x = {percent}", "x = 30%")
```

README 最后一行写的是 `assert result == 0.3`。这是浮点等号断言，0.3 在二进制里没有精确表示，断言的成立依赖具体的舍入结果。稳妥的写法是 `abs(result - 0.3) < 1e-9`。这类细节 linter 抓不到，属于 code review 该管的范畴。

### 装饰器给 interface 带来了什么

退一步从 interface 的角度看这套装饰器。[07:45](https://www.bilibili.com/video/BV1RfaX6sESp/?t=465)

转换行为完全由大括号里的东西决定。大括号里写 `percent`，库就在全局找一个叫 `percent` 的函数，把 `30%` 这个字符串喂进去，拿到 float。`Conversion` 在这条链路上干了什么？

视频里给了两个候选作用：显式规定 `percent` 是一种 conversion 方式，好让实现能 match 到这个 case；以及规定函数的 argument 数量，有没有 format spec。

两个都站不住。

第一个作用多余。用户给了什么函数，库就用什么函数转换，`Conversion` 里没有任何检查，只是把原函数原样存住，等会儿再扔出来。

第二个作用在 type check 层面也拿不到额外信息：被装饰的函数本来就带类型标注。argument 数量、有没有 format spec，run time 全都知道。

> 这个 conversion 跟 format conversion 在设计的时候压根就不应该存在，他们俩没有给这个项目带来足够多的价值

更省事的 interface 是让用户直接写一个 conversion 函数传进去，不需要 `Conversion` 这个壳。

### 同一份数据写了两次

基础例子里有两处重复。[09:42](https://www.bilibili.com/video/BV1RfaX6sESp/?t=582)

```python
city, year = parse[str, int](
    t"I live in {str}, since {int}", "I live in Tokyo, since 2010"
)
```

`str` 出现两次，`int` 出现两次。模板里写了一遍 `{str}`、`{int}`，类型参数列表里又写了一遍 `str`、`int`。把字符串转成整数这种最常见的动作，用户要写两次 `int`。

类型参数那部分有明确用途，type checker 靠它推断返回值类型。模板里那一遍是不是必须，作者可以给出理由。

视频里给出的替代思路是：模板里已经声明了要返回什么值，转换方式能不能给一个默认。

interface 设计原则有两条：

- 没有歧义的情况下，让用户输入的东西越少越好。
- 同样的数据，不要让用户输两次。

按这两条量，`{int}` 这个写法在有歧义的时候需要消解，消解的办法就是让用户多写一次类型参数。这个代价能不能降，取决于库有没有给出带默认的方案。

### 大括号里的方向反了

自定义转换这块的直觉冲突更麻烦。

在 f-string 和 t-string 里，大括号里的内容一般表示「要转换哪个变量」。`f"{name}"` 里的 `name` 是数据。作者的设计里，大括号里的 `percent` 表示的是「转换这个 `30%` 的方式」。[10:30](https://www.bilibili.com/video/BV1RfaX6sESp/?t=630)

方向是反的。不熟悉这个库的人第一反应一定是「30% 对应 percent」。

> 我刚开始在读这个 readme 的时候大括号里面是什么东西我其实一直没搞明白

作者在视频里给出的替代方案是利用 t-string 自带的 format spec：冒号后面的内容会被自动带出来，而冒号后面在 Python 的惯例里就代表格式转换。定义「冒号后面出现某个名字就表示用这个函数转换」，语义方向和 f-string 的直觉对齐了。

`FormatConversion` 的例子已经在用这个位置。

```python
(result,) = parse[str](t"{between:2,5}", "abcdefg")
```

冒号后面是 `2,5`，代表转换时用的 format spec。把函数名放进同一个位置，interface 就能统一到「冒号后面是转换指令」这一条规则上。

### JSON 破坏了一致性

设计分歧里最硬的一条在 `json` 上。

作者给 `json` 定的语义是：把 JSON 字符串变成 dict 或 list。于是 `{json}` 出现在模板里。

前面所有设计里，大括号里放的都是 class 或者 function，使用者的心智模型是「我把字符串喂给它」。到了 `json` 这里，大括号里是一个 module。[12:39](https://www.bilibili.com/video/BV1RfaX6sESp/?t=759)

> 你是没有办法把一个 string feed 到这个 module 里的，那这个设计就相当于你自己重新又开辟了一个语义，这个语义是哪儿都没有的，你自己发明的

`json` 是一个 module，没有 `__call__`，字符串没法直接喂给它。库为了让它工作，专门在 match case 里加了一段特例。

```python
case _ if (
    origin is list or origin is dict or origin is json or is_typeddict(origin)
):
    return json.loads(s)
```

![match case 的最后一段：list、dict、json 都交给 json.loads](images/fig-json-case.png)

这段代码和它前面的 case 长得完全不一样，是整段分派逻辑里自己编出来的特例。README 里也承认了这一点。

> list and dict are just aliases for json

`list`、`dict`、`json` 三个名字对应完全一样的语义。用户写 `{list}` 和写 `{json}` 得到相同结果，还多记两条别名。

修法有两种。

一种是把 `Conversion` 和 `FormatConversion` 这层壳去掉。用户直接写 `json.loads`，库把它当普通 callable 传进去就能用，`json` 不需要特殊语义。

> 如果你是用 json.loads 都会好非常非常多，你甚至不用专门给它规定一个语义

另一种是如果确实要支持 `list` 和 `dict`，那就在 run time 做 type check，确认解析出来的确实是 list 或 dict，而不是把 dict 装在 list 的位置上。

> 这里我觉得如果你支持 list 和 dict 的话，应该做一些 run time 的 type check

最后一条推论也顺下来了：既然这个库这么在意 type check，`parse` 自己接收的字符串和类型参数也该有 run time 校验。现在的实现里，类型不匹配时抛出的错误来自底层库，报错信息指向使用者填错的地方，而不是指向模板声明和实际值不匹配这件事。[13:45](https://www.bilibili.com/video/BV1RfaX6sESp/?t=825)
