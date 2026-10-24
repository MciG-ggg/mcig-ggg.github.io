---
title: AI 时代下的 Code Review 教程
timestamp: 2026-10-24T10:30:00+08:00
series: CodeReview教程
tags: [课内, CodeReview教程]
description: 把 code review 拆成三个问题来答：流程里它在什么位置、清单上查什么、AI 时代哪些问题消失哪些问题浮出。案例是 temparse 这个模板解析库，结论落在判断力上。
source_url: https://www.bilibili.com/video/BV1RfaX6sESp/
toc: false
draft: false
---


```cardlink
url: https://www.bilibili.com/video/BV1RfaX6sESp/
title: "【Code Review】AI时代的CR还有意义么？一起看一个模板解析库"
description: "码农高天逐行 review temparse 这个模板解析库，从代码走到设计"
host: bilibili.com
image: http://i2.hdslb.com/bfs/archive/4e38bd7d15cede74eb1639ee7e6ac7e8adc1c0e7.jpg
```

```cardlink
url: https://www.bilibili.com/video/BV1Dh4y1J7iG/
title: "code review 真的有必要吗，到底在review什么？"
description: "御风大世界讲 code review 在公司流程里的位置、要查什么、以及怎么沟通"
host: bilibili.com
image: http://i1.hdslb.com/bfs/archive/12c0b2d9b6f7329aaa4a9d4a389784b530b05c7f.jpg
```

```cardlink
url: https://www.bilibili.com/video/BV1punn6JEWi/
title: "实习/科研/学习中不用AI干不完，用AI写代码又看不懂，我该怎么办？"
description: "杜子源源回答实习生的疑问：AI 时代怎么保住自己的判断力"
host: bilibili.com
image: http://i1.hdslb.com/bfs/archive/1bb4755a174aa0e23bfdf390f11a33b4c88182f8.jpg
```

- 第 1 章 Code Review 在流程中的位置 — code review 在开发流程里的位置：什么时候做、谁参与、QA 为什么要在场
- 第 2 章 审查清单与沟通心态 — 一份能落地的审查清单：需求、可读性、规范、对现有逻辑的影响、文档，外加 review 时的沟通分寸
- 第 3 章 读一个模板解析库的代码 — 以 temparse 为例读一遍代码：t-string 模板、泛型签名、预编译正则、match case 分派转换
- 第 4 章 设计层的分歧：装饰器、interface 与一致性 — 三个设计分歧：小写 class parse 的 PEP 718 理由、装饰器该不该存在、JSON 分支破坏 interface 一致性
- 第 5 章 工程化收尾：覆盖率、CI，以及 AI 之后剩下的问题 — 测试没有 coverage、workflow 只有一个发布流程；AI 消掉琐碎问题后 design、interface、CI 成为新战场
- 第 6 章 判断力：把工作和知识解耦 — 实习生用 AI 写算子却看不懂：把工作与知识解耦，以 50 行代码为目标迭代，按能力、基础、导向分三层

AI 写代码之后，code review 没有变得不重要，只是审的东西换了位置。缩进、变量名格式、重复的样板代码，这些琐碎问题被 AI 加 linter 加 formatter 接管了；design、interface、CI 这些层面的问题浮到水面上，而它们从来不需要 IDE 插件来发现。

这篇教程把三场视频拆成六章读。前两章来自御风大世界，讲 code review 在公司流程里的位置和一份能落地的审查清单。第三到第五章来自码农高天，他当场 review 一个叫 temparse 的模板解析库，从 README 一路读到 `parse` 这个小写 class 的设计理由，第六章是杜子源源回答实习生的问题：AI 写的代码看不懂，问题出在哪。

## 章节

- [第 1 章 Code Review 在流程中的位置](<第 1 章 Code Review 在流程中的位置>)
- [第 2 章 审查清单与沟通心态](<第 2 章 审查清单与沟通心态>)
- [第 3 章 读一个模板解析库的代码](<第 3 章 读一个模板解析库的代码>)
- [第 4 章 设计层的分歧：装饰器、interface 与一致性](<第 4 章 设计层的分歧：装饰器、interface 与一致性>)
- [第 5 章 工程化收尾：覆盖率、CI，以及 AI 之后剩下的问题](<第 5 章 工程化收尾：覆盖率、CI，以及 AI 之后剩下的问题>)
- [第 6 章 判断力：把工作和知识解耦](<第 6 章 判断力：把工作和知识解耦>)
