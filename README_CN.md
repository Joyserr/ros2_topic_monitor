# ROS2 Topic Monitor

## 中文

一个高性能的 ROS2 话题监控工具，具有类似 htop 的交互式 ncurses TUI 界面。

### ✨ 特性

- **高性能监控**: 基于 C++17 + rclcpp，CPU 占用极低（< 5%）
- **ncurses TUI 界面**: 类似 htop 的交互式终端界面
- **自动话题发现**: 使用 ROS graph API 自动发现所有话题
- **懒加载机制**: 主界面仅统计元数据，进入详情页才解析消息内容
- **多线程架构**: 使用 MultiThreadedExecutor 实现高并发处理
- **实时统计**: FPS、延迟、带宽等关键指标实时显示
- **线程安全**: 使用 shared_mutex、atomic 等保证数据一致性

### 📋 依赖要求

- ROS2 Humble
- ncurses 库
- C++17 兼容编译器

### 🔧 编译安装

```bash
cd ~/colcon_ws
colcon build --packages-select ros2_topic_monitor
source install/setup.bash
```

### 🚀 使用方法

启动监控工具：

```bash
ros2 run ros2_topic_monitor monitor
```

#### 主界面操作

- **↑/↓**: 选择话题
- **Enter**: 进入话题详情页
- **Q**: 退出程序
- **R**: 手动刷新话题列表

#### 详情页操作

- **Q**: 返回主界面
- **D**: 切换消息内容解析模式
- **↑/↓**: 滚动消息内容

### 📊 显示信息

#### 主界面

显示所有话题的实时统计信息：

- 话题名称
- 消息类型
- FPS（每秒消息数）
- 延迟（毫秒）
- 带宽（字节/秒）

#### 详情页

显示选中话题的详细信息：

- 实时指标统计
- FPS 历史曲线（ASCII sparkline）
- 最新消息内容（hex dump）

### 🔍 故障排查

#### ncurses 相关问题

如果界面显示异常：

```bash
export TERM=xterm-256color
```

#### 编译错误

确保已安装 ncurses 开发库：

```bash
sudo apt-get install libncurses5-dev libncursesw5-dev
```

#### 话题不显示

- 检查 ROS2 话题是否正在发布：`ros2 topic list`
- 确认 ROS_DOMAIN_ID 设置正确

### 📝 许可证

MIT License