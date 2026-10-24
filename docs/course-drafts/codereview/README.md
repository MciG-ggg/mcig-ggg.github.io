# Code Review 教程的草稿与中间产物

这篇教程由三个 B 站视频合成，来源与章节切分见 `agent_outputs/outline.md`。成品是**一页**：`src/content/lecture/zh-cn/CodeReview教程_AI时代下的CodeReview/index.md`，六章是这页里的六个 `##` 小节。

| 目录 | 内容 |
|---|---|
| `agent_outputs/outline.md` | 六章的要点、章节到时间区间的映射 |
| `agent_outputs/glossary.md` | 术语表。字幕是 ASR 出来的，专有名词全靠这张表纠正 |
| `agent_outputs/briefs/` | 每章交给 writer agent 的写作指令 |
| `agent_outputs/consistency_report.md` | 审校 agent 报的问题清单 |
| `agent_outputs/page_meta.json` | 成品页的 frontmatter、章节标题与锚点 id、draft 开关都在这里 |
| `agent_outputs/figures_manifest.json` | 每张图的来源视频、时间点、图注 |
| `sections/` | writer agent 写的章节正文，一章一份 |
| `slices/` | 每章的字幕切片，带时间戳，是溯源链接的校准依据 |
| `source/` | 字幕原件、精修版，以及 temparse 仓库的源码 |
| `scripts/` | 抽图与集成的脚本 |

## 改已发布的正文

不要手改 `src/content/lecture/zh-cn/CodeReview教程_AI时代下的CodeReview/index.md`。改 `sections/section_N.md`，然后跑：

```bash
python3 docs/course-drafts/codereview/scripts/integrate.py
```

脚本读 `agent_outputs/page_meta.json`，把六章草稿拼成成品页。标题、章节锚点 id、发布时间、draft 开关都改那个 JSON。跨章链接写 `#chN`，`chN` 是 `page_meta.json` 里那章的 `id`。

## 重新抽图

抽图脚本从 `source/*.mp4` 取帧，图片写进成品的 `images/`。视频不入库，需要时用 `yt-dlp --cookies-from-browser chrome` 重新下载到 `source/`。

## temparse 源码

`source/temparse_*.py` 与 `source/temparse_README.md` 来自 https://github.com/jiang-zhexin/temparse ，MIT License，许可证原文在 `source/temparse_LICENSE`。
