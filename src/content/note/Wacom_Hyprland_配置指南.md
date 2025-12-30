---
title: 
timestamp: 2025-11-12 19:56:15+08:00
---

# Hyprland 下 Wacom 数位板映射到外接显示器配置指南

## 概述

本指南详细介绍如何在 Hyprland 窗口管理器下配置 Wacom 数位板，使其只映射到指定的外接显示器，而不是整个多显示器桌面空间。

## 适用场景

- **系统环境**: Ubuntu 24.04 LTS + Hyprland
- **硬件需求**: Wacom 数位板 + 外接显示器
- **目标**: 数位板输入只在外接显示器上响应，提供精确的绘画体验

## 配置步骤

### 1. 安装 Wacom 驱动

```bash
# 更新包索引
sudo apt update

# 安装 Wacom 驱动（注意：wacom-tools 已弃用）
sudo apt install xserver-xorg-input-wacom

# 验证安装
dpkg -l | grep wacom
```

**预期输出**:
```
ii  libwacom-common                                  2.10.0-2                                  all          Wacom model feature query library (common files)
ii  libwacom-dev:amd64                               2.10.0-2                                  amd64        Wacom model feature query library (development files)
ii  libwacom9:amd64                                  2.10.0-2                                  amd64        Wacom model feature query library
ii  xserver-xorg-input-wacom                         1:1.2.0-1ubuntu2                          amd64        X.Org X server -- Wacom input driver
```

### 2. 识别设备信息

#### 2.1 识别 Wacom 设备名称

```bash
# 方法一：使用 Hyprland 命令
hyprctl devices | grep -i wacom

# 方法二：如果需要更详细信息
libinput list-devices | grep -i wacom
```

**示例输出**:
```
wacom-one-by-wacom-s-pen
```

#### 2.2 识别显示器信息

```bash
# 查看所有显示器配置
hyprctl monitors
```

**示例输出**:
```
Monitor eDP-1 (ID 0):  # 内置笔记本显示器
	2520x1680@120.00000 at 2433x1373
	make: Thermotrex Corporation
	model: TL142GVXP12-0

Monitor DP-2 (ID 1):  # 外接显示器
	1920x1080@60.00000 at 2433x293
	make: HP Inc.
	model: HP P24v G4
```

### 3. 备份配置文件

```bash
# 备份 Hyprland 主配置文件
cp ~/.config/hypr/hyprland.conf ~/.config/hypr/hyprland.conf.backup

# 备份用户设置文件
cp ~/.config/hypr/UserConfigs/UserSettings.conf ~/.config/hypr/UserConfigs/UserSettings.conf.backup
```

### 4. 配置 Wacom 映射

#### 4.1 找到 Tablet 配置块

编辑用户设置文件：
```bash
nano ~/.config/hypr/UserConfigs/UserSettings.conf
```

找到 `tablet` 配置块（通常在 `input` 部分）：

```ini
# 原始配置
tablet {
    transform = 0
    left_handed = 0
}
```

#### 4.2 添加输出映射

修改为：

```ini
# 更新后的配置
tablet {
    transform = 0
    left_handed = 0
    output = DP-2  # 将 Wacom 映射到外接显示器
}
```

**配置参数说明**:
- `transform`: 屏幕旋转角度（0=0°, 1=90°, 2=180°, 3=270°）
- `left_handed`: 左手模式（0=右手, 1=左手）
- `output`: 目标显示器名称（根据你的实际显示器名称调整）

### 5. 重新加载配置

```bash
# 重新加载 Hyprland 配置
hyprctl reload

# 或者使用快捷键
# Mod + Shift + R
```

### 6. 验证配置

#### 6.1 检查设备状态

```bash
# 检查 Wacom 设备是否正常识别
hyprctl devices | grep -A 5 -i wacom
```

#### 6.2 实际测试

1. **打开绘图应用**（Krita、GIMP、Inkscape 等）
2. **移动数位板笔** - 光标应该只出现在指定的外接显示器上
3. **测试绘画功能** - 确认压感和绘画功能正常
4. **测试数位板按钮** - 验证快捷键功能

## 常见显示器名称

根据连接方式，显示器名称可能为：

| 接口类型 | 典型名称 |
|---------|---------|
| HDMI | `HDMI-A-1`, `HDMI-A-2` |
| DisplayPort | `DP-1`, `DP-2`, `DP-3` |
| USB-C/雷电 | `USB-C-1`, `DP-3` |
| 内置显示器 | `eDP-1`, `LVDS-1` |

## 故障排除

### 问题1：配置不生效

**解决方案**：
```bash
# 完全重启 Hyprland
mod + shift + r

# 或重启系统
sudo reboot
```

### 问题2：找不到设备名称

**解决方案**：
```bash
# 查看所有输入设备
hyprctl devices

# 确保数位板已连接并被识别
lsusb | grep -i wacom
```

### 问题3：映射范围不正确

**解决方案**：
```bash
# 检查显示器分辨率和位置
hyprctl monitors

# 可能需要调整显示器布局
# 在 ~/.config/hypr/monitors.conf 中配置
```

### 问题4：压感不工作

**解决方案**：
```bash
# 检查是否安装了完整的驱动
sudo apt install libwacom9

# 在绘图应用中检查压感设置
# 确保选择了正确的输入设备
```

## 高级配置

### 左手模式配置

```ini
tablet {
    transform = 0
    left_handed = 1  # 启用左手模式
    output = DP-2
}
```

### 旋转配置

```ini
tablet {
    transform = 1  # 顺时针旋转90度
    left_handed = 0
    output = DP-2
}
```

### 多显示器特定映射

如果有多个外接显示器，可以指定具体映射：

```ini
tablet {
    transform = 0
    left_handed = 0
    output = DP-2  # 第二个显示器
    # output = HDMI-A-1  # HDMI 显示器
}
```

## 配置文件位置

- **主配置文件**: `~/.config/hypr/hyprland.conf`
- **用户设置**: `~/.config/hypr/UserConfigs/UserSettings.conf`
- **显示器配置**: `~/.config/hypr/monitors.conf`
- **工作区配置**: `~/.config/hypr/workspaces.conf`

## 备份和恢复

### 创建备份脚本

```bash
#!/bin/bash
# backup_wacom_config.sh

BACKUP_DIR="$HOME/.config/hypr/backups"
DATE=$(date +"%Y%m%d_%H%M%S")

mkdir -p $BACKUP_DIR

# 备份配置文件
cp ~/.config/hypr/UserConfigs/UserSettings.conf "$BACKUP_DIR/UserSettings_$DATE.conf"
cp ~/.config/hypr/hyprland.conf "$BACKUP_DIR/hyprland_$DATE.conf"

echo "配置已备份到: $BACKUP_DIR"
```

### 恢复配置

```bash
# 恢复最新备份
cp ~/.config/hypr/backups/UserSettings_*.conf ~/.config/hypr/UserConfigs/UserSettings.conf
hyprctl reload
```

## 参考资源

- [Hyprland 官方文档](https://wiki.hyprland.org/)
- [Wacom Linux 支持](https://github.com/linuxwacom)
- [Ubuntu Wacom 配置指南](https://help.ubuntu.com/community/Wacom)

---

**提示**: 配置完成后，建议创建系统还原点，以便在出现问题时快速恢复。

**注意**: 本指南基于 Ubuntu 24.04 LTS 和 Hyprland，其他发行版可能需要调整包管理命令。
