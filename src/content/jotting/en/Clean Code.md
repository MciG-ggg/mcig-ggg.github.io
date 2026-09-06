---
title: Clean Code
timestamp: 2026-02-22T16:33:16+08:00
tags:
  - Tools
draft: false
---

# Clean Code

I have been reading *Clean Code* and taking a few notes.

I also had a few thoughts while looking through projects:

- *Clean Code* says that the logic inside a function should stay at a single level of abstraction. I find this difficult to follow, or maybe it does not need to be followed so strictly. For example, when performing lower-level operations inside a function, adding a useful comment can be necessary. Without one, the function can be hard to understand for someone who did not write it. Sometimes having too many small functions makes the code harder to read instead (?).
