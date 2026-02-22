---
title: ���的zellij使用
date: 2026-02-22
timestamp: 2026-02-22T14:14:29+08:00
slug: zellij
category: jotting
tags:
  - Tools
---

## 1. 会话管理（持久化你的工作）

Zellij 的核心能力在于即便 SSH 断开，你的代码和进程依然在运行。

- **启动/创建命名会话：** `zellij -s <名称>`（如 `zellij -s dev`）。
    
- **查看当前所有会话：** `zellij ls`。
    
- **智能恢复（最常用）：** `zellij a -c`。
    
    - _逻辑：_ 有旧会话就恢复，没有就新建。建议把这个命令写成别名（alias）。
        
- **退出但不关闭（Detach）：** `Ctrl + o` 然后按 `d`。
    

## 2. 界面控制（Tab 与 Pane）

Zellij 把终端分成了“标签页”和“窗格”。

- **窗格 (Pane) 操作 (`Ctrl + p`)：**
    
    - `n`：新建窗格。
        
    - `d`：向下分割 / `r`：向右分割。
        
    - `f`：切换悬浮/嵌入状态（**Floating**，非常适合临时跑个命令）。
        
    - `z`：全屏切换（Toggle Maximize）。
        
- **标签页 (Tab) 操作 (`Ctrl + t`)：**
    
    - `n`：新建标签。
        
    - `r`：重命名当前标签（区分不同项目）。
        
- **调整大小 (`Ctrl + n`)：** 进入后用方向键调整当前窗格尺寸。
    

## 3. 布局系统 (Layouts)

通过 `.kdl` 文件实现一键启动复杂的开发环境，不再需要手动分屏。

- **定义布局：** 在 `~/.config/zellij/layouts/` 下创建文件。
    
- **核心语法：**
    
    代码段
    
    ```
    layout {
        pane size=1 borderless=true { plugin location="zellij:tab-bar"; }
        pane split_direction="vertical" {
            pane focus=true // 这里运行你的编辑器
            pane size="25%" // 这里放调试终端
        }
        pane size=1 borderless=true { plugin location="zellij:status-bar"; }
    }
    ```
    
- **加载布局：** `zellij --layout <文件名>`。
    

## 4. 现代化的进阶功能

- **紧凑 UI 模式：** 如果觉得上下状态栏太占位，使用 `zellij --layout compact`。
    
- **多节点协作：** 两个人 SSH 到同一台机器，`attach` 到同一个会话，可以实现真正的“远程结对编程”，且两人可以各自看不同的 Tab。
    
- **快捷键解耦：** 既然你选择了在编辑器内管理文件，建议在 `config.kdl` 中配置 `Alt + hjkl` 直接切换窗格，跳过 `Ctrl + p` 这一步前缀键，体验会非常接近 i3 或 Sway。
    
## 5. 删除会话
#### 一键删除所有已退出的会话：
```shell
zellij ls | grep "EXITED" | awk '{print $1}' | sed 's/\x1b\[[0-9;]*m//g' | xargs -I {} zellij delete-session {}
```

#### 删除所有会话（包括运行中的）：

```shell
zellij ls | grep -v "Running" | awk '{print $1}' | sed 's/\x1b\[[0-9;]*m//g'| xargs -I {} zellij delete-session {}
```