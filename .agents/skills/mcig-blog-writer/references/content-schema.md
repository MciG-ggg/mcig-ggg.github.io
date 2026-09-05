# Content Collection Schema

定义在 `src/content.config.ts`。落盘文章前**必须**核对字段。

---

## 1. note（主要博客文章）

**路径**：`src/content/note/` 或 `src/content/note/zh-cn/`

**schema**：

```ts
{
  title: string,           // 必填，文章标题
  timestamp: Date,         // 必填，发布日期（带时区，YYYY-MM-DD HH:MM:SS+08:00）
  series?: string,         // 可选，系列名（如 "MiniMind-O" / "总结"）
  tags?: string[],         // 可选，主题标签
  description?: string,    // 可选，文章描述/excerpt
  sensitive?: boolean,     // 默认 false，敏感内容
  toc?: boolean,           // 默认 true，是否显示目录
  top?: number,            // 默认 0，置顶优先级（越大越靠前）
  draft?: boolean          // 默认 false，草稿状态（不公开）
}
```

**loader**：`glob({ pattern: ["**/*.md", "!**/_*.md", "!**/_*/*.md"], base: "./src/content/note" })`
- 加载所有 `.md` 文件
- 排除下划线开头（`_*` 和 `_*/*.md`），用于私有笔记和图片预览

**典型 frontmatter 示例**：

```yaml
---
title: MiniMind-O CUDA Graph 优化实录
timestamp: 2026-09-05 14:30:00+08:00
series: MiniMind-O
tags:
  - cuda
  - inference
  - optimization
description: 把 MiniMind-O 的端到端推理从 320ms 进一步压到 X ms，用 CUDA Graph + ncu 取证。
toc: true
top: 0
draft: false
---
```

---

## 2. jotting（短记 / 流水）

**路径**：`src/content/jotting/` 或 `src/content/jotting/zh-cn/`

**schema**：

```ts
{
  title: string,           // 必填
  timestamp: Date,         // 必填
  tags?: string[],         // 可选
  description?: string,    // 可选
  sensitive?: boolean,     // 默认 false
  top?: number,            // 默认 0
  draft?: boolean          // 默认 false
}
```

注意：jotting **没有** `series` 和 `toc` 字段。

**典型 frontmatter 示例**：

```yaml
---
title: 3 月月总结
timestamp: 2025-03-31 22:15:00+08:00
tags:
  - conclusion
description: 进组第一个月，科研 reward 很稀疏的过程。
draft: false
---
```

---

## 3. preface（前言 / 站点公告）

**路径**：`src/content/preface/`

**schema**：

```ts
{
  timestamp: Date  // 必填，只有这一个字段
}
```

**典型 frontmatter 示例**：

```yaml
---
timestamp: 2025-01-01 00:00:00+08:00
---
```

**用途**：站点级别的前言、公告、版本说明，不是常规博客文章。

---

## 4. information（静态信息页）

**路径**：`src/content/information/`

**loader**：`glob({ pattern: "**/*.{md,mdx,yaml}", base: "./src/content/information" })`

**schema**：无（自由格式）

**支持文件类型**：`.md` / `.mdx` / `.yaml`

**典型 frontmatter**：无强制字段，按页面需要自填。

**用途**：关于页、友链页、政策页等静态信息。

---

## 落盘决策流程

1. **判定类型** → 查 `templates.md` 的「类型 → 集合映射速查」
2. **判定路径**：
   - 短文 → 直接 `src/content/<集合>/zh-cn/<标题>.md`
   - 长文 / 多文件 → `src/content/<集合>/zh-cn/<标题>/index.md` + 子文件
   - 静态页 → `src/content/information/...`
3. **填 frontmatter**：
   - 必填字段一个不能少
   - 默认值字段（toc/sensitive/top/draft）不写就用默认
   - `timestamp` 用 ISO 8601 带时区（建议 `+08:00`）
4. **私有/草稿**：
   - 完全私有 → 文件名前加 `_`（被 loader 排除）
   - 公开但不想被列表显示 → `draft: true`
   - 敏感内容 → `sensitive: true`
5. **本地预览**：`pnpm dev` 看 Astro 是否报错

---

## 时间戳格式

ISO 8601 带时区：

```yaml
timestamp: 2025-06-22 13:28:56+08:00
```

或用 Zod 接受的 date 格式：`timestamp: 2025-06-22`（Astro 会自动补 midnight UTC，**但**建议带时间避免时区混淆）。

---

## 默认值速查

| 字段 | 默认值 | 说明 |
|---|---|---|
| `sensitive` | `false` | 敏感内容标记 |
| `toc` | `true` | 是否显示右侧目录（仅 note）|
| `top` | `0` | 置顶优先级，0 = 不置顶 |
| `draft` | `false` | 草稿状态，默认不是草稿 |

⚠️ **`draft` 默认是 `false`**（见 `src/content.config.ts` 定义）——只要不显式设 `draft: true`，新文章落盘即公开。
