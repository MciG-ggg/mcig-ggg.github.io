---
title: How I Use Zellij
timestamp: 2026-02-22T17:25:35+08:00
tags:
  - Tools
draft: false
---

## 1. Session Management

Zellij's core feature is that your code and processes keep running even when your SSH connection drops.

- **Start or create a named session:** `zellij -s <name>` (for example, `zellij -s dev`).

- **List all current sessions:** `zellij ls`.

- **Smart attach, my most-used command:** `zellij a -c`.

  - _Logic:_ attach to an existing session when one exists; otherwise create a new one. It is worth making this command an alias.

- **Detach without closing the session:** press `Ctrl + o`, then `d`.

## 2. Interface Control: Tabs and Panes

Zellij divides the terminal into tabs and panes.

- **Pane operations (`Ctrl + p`):**

  - `n`: create a new pane.
  - `d`: split downward; `r`: split to the right.
  - `f`: toggle floating or embedded mode. This is useful for running a temporary command.
  - `z`: toggle maximized mode.

- **Tab operations (`Ctrl + t`):**

  - `n`: create a new tab.
  - `r`: rename the current tab, which helps distinguish projects.

- **Resize panes (`Ctrl + n`):** enter resize mode and use the arrow keys to adjust the current pane.

## 3. Layouts

Layouts use `.kdl` files to start a complex development environment with one command, so you do not have to split panes manually.

- **Define a layout:** create a file under `~/.config/zellij/layouts/`.

- **Core syntax:**

  ```kdl
  layout {
      pane size=1 borderless=true { plugin location="zellij:tab-bar"; }
      pane split_direction="vertical" {
          pane focus=true // editor
          pane size="25%" // debugging terminal
      }
      pane size=1 borderless=true { plugin location="zellij:status-bar"; }
  }
  ```

- **Load a layout:** `zellij --layout <layout-name>`.

## 4. More Advanced Features

- **Compact UI mode:** if the top and bottom status bars take up too much space, use `zellij --layout compact`.

- **Collaboration across hosts:** two people can SSH into the same machine and attach to the same session. They can even view different tabs, which makes this a practical form of remote pair programming.

- **Decoupled keybindings:** since I choose to manage files inside the editor, I recommend configuring `Alt + hjkl` in `config.kdl` to switch panes directly. This skips the `Ctrl + p` prefix and feels much closer to i3 or Sway.

## 5. Deleting Sessions

#### Delete all exited sessions

```shell
zellij ls | grep "EXITED" | awk '{print $1}' | sed 's/\\x1b\\[[0-9;]*m//g' | xargs -I {} zellij delete-session {}
```

#### Delete all sessions, including running sessions

```shell
zellij ls | grep -v "Running" | awk '{print $1}' | sed 's/\\x1b\\[[0-9;]*m//g' | xargs -I {} zellij delete-session {}
```
