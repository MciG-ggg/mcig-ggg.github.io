---
title: 生成式模型简介
date: 2026-03-12
timestamp: 2026-03-12T17:10:17+08:00
slug: 生成式模型简介
category: note
tags:
  - AI
  - ML
  - Generative-Model
  - Course
  - MIT6S184
---

一些基本理解

> [!note]
> 1. 要生成的对象是一个向量$z \in \mathbb{R^d}$
> 2. 生成一个对象 $z$ 被建模为从数据分布 $z ~ p_{data}$ 中采样
> 3. 一个 Dataset 由有限数量的样本 $z_{1}, ..., z_{N} \sim p_{data }$ 组成
> 4.  条件生成, 即根据一些已有的数据来生成对象， 从 $z \sim p_{data}(\cdot| y)$ 中采样

[Flow and Diffusion Model](/note/flow-and-diffusion-model)
[如何构建训练目标](/note/如何构建训练目标)
[如何训练生成式模型](/note/如何训练生成式模型)
[如何构建图像生成器](/note/如何构建图像生成器)