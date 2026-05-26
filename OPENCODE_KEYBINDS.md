# OpenCode 常用快捷键

> 默认前缀键: `Ctrl+X`，带 `<leader>` 的快捷键需先按前缀键再按对应键。

## 通用

| 快捷键 | 功能 |
|--------|------|
| `Ctrl+C` | 清除输入框 / 中断 |
| `Ctrl+D` | 退出 |
| `Ctrl+P` | 命令面板 |
| `Ctrl+X` `Q` | 退出 |
| `Ctrl+X` `H` | 帮助 |
| `Ctrl+Alt+K` | 显示所有快捷键 |

## 会话管理

| 快捷键 | 功能 |
|--------|------|
| `Ctrl+X` `N` | 新建会话 |
| `Ctrl+X` `L` | 会话列表 |
| `Ctrl+R` | 重命名会话 |
| `Ctrl+D` | 删除会话 |
| `Ctrl+X` `G` | 会话时间线 |
| `Ctrl+X` `U` | 撤销上条消息 |
| `Ctrl+X` `R` | 重做 |
| `Ctrl+X` `C` | 压缩会话 |
| `Ctrl+X` `X` | 导出会话 |
| `Ctrl+X` `E` | 外部编辑器编辑 |
| `Escape` | 中断当前操作 |

## 模型与 Agent

| 快捷键 | 功能 |
|--------|------|
| `Ctrl+A` | 模型列表 |
| `Ctrl+F` | 收藏/取消收藏模型 |
| `Ctrl+X` `M` | 模型选择 |
| `F2` | 切换最近使用模型 |
| `Shift+F2` | 反向切换模型 |
| `Ctrl+T` | 切换模型变体 |
| `Tab` | 切换 Agent |
| `Shift+Tab` | 反向切换 Agent |
| `Ctrl+X` `A` | Agent 列表 |

## 输入编辑

| 快捷键 | 功能 |
|--------|------|
| `Enter` | 发送消息 |
| `Shift+Enter` / `Ctrl+Enter` / `Alt+Enter` | 换行 |
| `Ctrl+C` | 清除输入 |
| `Ctrl+V` | 粘贴 |
| `Ctrl+A` | 光标移到行首 |
| `Ctrl+E` | 光标移到行尾 |
| `Ctrl+B` / `←` | 光标左移 |
| `Ctrl+F` / `→` | 光标右移 |
| `Alt+B` / `Alt+←` | 光标左移一个单词 |
| `Alt+F` / `Alt+→` | 光标右移一个单词 |
| `Ctrl+K` | 删除到行尾 |
| `Ctrl+U` | 删除到行首 |
| `Ctrl+W` | 删除前一个单词 |
| `Alt+D` | 删除后一个单词 |
| `Ctrl+Shift+D` | 删除整行 |
| `Ctrl+D` / `Delete` | 删除光标处字符 |
| `Backspace` | 删除前一个字符 |
| `Ctrl+←` | 光标左移一个单词 |
| `Ctrl+→` | 光标右移一个单词 |
| `Home` | 光标移到缓冲区开头 |
| `End` | 光标移到缓冲区末尾 |
| `Super+A` | 全选（终端中可能不生效，建议自定义为 `Ctrl+A`） |

## 选择文本

| 快捷键 | 功能 |
|--------|------|
| `Shift+←` | 向左选择 |
| `Shift+→` | 向右选择 |
| `Shift+↑` | 向上选择 |
| `Shift+↓` | 向下选择 |
| `Shift+Home` | 选择到缓冲区开头 |
| `Shift+End` | 选择到缓冲区末尾 |
| `Alt+Shift+←` | 向左选择一个单词 |
| `Alt+Shift+→` | 向右选择一个单词 |
| `Ctrl+Shift+A` | 选择到行首 |
| `Ctrl+Shift+E` | 选择到行尾 |

## 撤销/重做

| 快捷键 | 功能 |
|--------|------|
| `Ctrl+-` / `Super+Z` | 撤销输入 |
| `Ctrl+.` / `Super+Shift+Z` | 重做输入 |

## 消息与历史

| 快捷键 | 功能 |
|--------|------|
| `↑` / `↓` | 浏览历史消息 |
| `Ctrl+X` `Y` | 复制消息 |
| `Ctrl+X` `H` | 切换隐藏内容显示 |
| `PageUp` | 向上翻页 |
| `PageDown` | 向下翻页 |
| `Ctrl+G` / `Home` | 跳到顶部 |
| `End` | 跳到底部 |

## 侧边栏与视图

| 快捷键 | 功能 |
|--------|------|
| `Ctrl+X` `B` | 切换侧边栏 |
| `Ctrl+X` `S` | 状态视图 |
| `Ctrl+X` `T` | 主题列表 |
| `Ctrl+X` `E` | 打开编辑器 |

## 导航（子会话）

| 快捷键 | 功能 |
|--------|------|
| `Ctrl+X` `↓` | 进入子会话 |
| `→` | 下一个子会话 |
| `←` | 上一个子会话 |
| `↑` | 返回父会话 |

## 系统

| 快捷键 | 功能 |
|--------|------|
| `Ctrl+Z` | 挂起终端 |

---

## 自定义快捷键

在项目根目录创建 `tui.json`：

```json
{
  "$schema": "https://opencode.ai/tui.json",
  "keybinds": {
    "input_select_all": "ctrl+a",
    "input_clear": "ctrl+l",
    "app_exit": "ctrl+q"
  }
}
```

重启 opencode 后生效。

### 常用自定义建议

| 默认值 | 建议改为 | 原因 |
|--------|----------|------|
| `Super+A` (全选) | `Ctrl+A` | 终端中 Super 键通常不生效 |
| `Ctrl+C` (清除输入) | `Ctrl+L` | 避免与中断混淆 |
| `Ctrl+X` (前缀键) | `Ctrl+Space` | 更易按 |

参考文档: <https://opencode.ai/docs/keybinds/>
