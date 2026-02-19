---
title: OpenVLA 训练源码学习
date: 2026-02-19
timestamp: 2026-02-19T17:46:51+08:00
slug: openvla
category: note
tags:
  - AI
  - EmbodiedAI
  - VLA
  - Paper
---

# OpenVLA 训练源码学习

OpenVLA 基于 Prismatic-vlm [GitHub - TRI-ML/prismatic-vlms: A flexible and efficient codebase for training visually-conditioned language models (VLMs)](https://github.com/TRI-ML/prismatic-vlms) 这个 VLM 项目

这篇重点是 python 语法和代码逻辑，可以先看下面的博客内容

```cardlink
url: https://www.laumy.tech/2334.html/python%E8%A1%A5%E4%B9%A0/
title: "python补习 - laumy的学习笔记"
description: "装饰器 函数装饰器 什么是装饰器 装饰器是python的一种高级语法，本质上是函数包装器，可以在不修改函数代码 […]"
host: www.laumy.tech
favicon: https://www.laumy.tech/wp-content/uploads/2024/11/cropped-IMG_6192-scaled-1-32x32.jpg
```


## Pretrain- 训练 Prismatic-vlm