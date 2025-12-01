# ROS2 Topic Monitor

[English](README.md) | [中文](#中文)

---

## 中文

一个高性能的 ROS2 话题监控工具，具有交互式 ncurses TUI 界面，支持结构化消息解析和实时指标可视化。

### ✨ 特性

- **高性能**: 使用 C++17 + rclcpp 构建，CPU 占用极低（< 5%）
- **交互式 TUI**: 类似 htop 的终端用户界面，基于 ncurses
- **自动话题发现**: 使用 ROS2 图 API 自动发现所有话题
- **懒加载**: 主视图仅跟踪元数据；消息解析在详情视图中进行
- **多线程**: 使用 MultiThreadedExecutor 实现高并发
- **实时指标**: 实时显示 FPS、延迟、带宽和消息计数
- **线程安全**: 使用 shared_mutex 和原子操作保证数据一致性
- **QoS 显示**: 显示每个话题的 QoS 配置（可靠性、持久性、历史策略）
- **结构化消息解析**: 动态解析并以人类可读格式显示消息内容
- **数组支持**: 完全支持定长和动态数组，逐元素显示
- **搜索与过滤**: 实时话题搜索，不区分大小写匹配
- **分页显示**: 大量话题时自动分页，支持 Page Up/Down 导航
- **FPS 历史图表**: 可视化趋势图，显示消息频率随时间变化

### 📋 系统要求

- ROS2 Humble
- ncurses 库
- C++17 兼容编译器
- 标准 ROS2 消息包（std_msgs、sensor_msgs、geometry_msgs 等）

### 🔧 编译和安装

```bash
cd ~/colcon_ws
colcon build --packages-select ros2_topic_monitor
source install/setup.bash
```

### 🚀 使用方法

启动监控器：

```bash
ros2 run ros2_topic_monitor monitor
```

#### 主视图控制键

- **↑/↓**: 逐行浏览话题
- **PgUp/PgDn**: 翻页浏览（当话题超出屏幕时）
- **/**: 进入搜索模式
- **Enter**: 进入选中话题的详情视图
- **Q**: 退出应用程序
- **R**: 手动刷新话题列表

#### 搜索模式控制键

- **输入**: 输入搜索关键词（不区分大小写，匹配话题名称任意位置）
- **Backspace**: 删除最后一个字符
- **Enter**: 退出搜索模式并查看过滤结果
- **ESC**: 清除搜索并返回完整话题列表

#### 详情视图控制键

- **Q**: 返回主视图
- **D**: 切换消息内容解析（启用/禁用结构化显示）
- **A**: 切换数组展开（显示所有元素 vs. 前 10 个）
- **↑/↓**: 滚动消息内容
- **PgUp/PgDn**: 快速滚动（每次 10 行）

### 📊 显示信息

#### 主视图

显示所有话题的实时统计信息：

| 列 | 说明 |
|-----|------|
| Topic | 话题名称（过长时截断） |
| Type | 消息类型（显示 package/msg/Type 格式） |
| FPS | 每秒消息数（频率） |
| Delay(ms) | 平均消息延迟 |
| BW(B/s) | 每秒带宽（字节） |

**底部信息：**

- 左侧：控制键提示
- 右侧：分页信息（如 "Page 1/3 (25 topics)"）

#### 详情视图

显示选中话题的综合信息：

1. **实时指标**（第 2-6 行）：
   - FPS: 消息频率（Hz）
   - Delay: 平均延迟（毫秒）
   - Bandwidth: 数据速率（字节/秒）
   - Count: 接收的消息总数

2. **QoS 配置**（第 8-11 行）：
   - Reliability: BEST_EFFORT 或 RELIABLE
   - Durability: VOLATILE 或 TRANSIENT_LOCAL  
   - History: KEEP_LAST（含深度）或 KEEP_ALL
   - *关键字段用 [*] 标记*

3. **FPS 历史图表**（第 13-15 行）：
   - 可视化柱状图显示消息频率趋势
   - 每列代表一个时间点
   - 高度表示 FPS 值（越高 = 消息越多）
   - 图表下方显示范围

4. **消息内容**（第 16 行起）：
   - **结构化格式**（解析启用时）：
     - 分层显示所有消息字段
     - 嵌套消息使用适当缩进（每层 2 个空格）
     - 数组内联显示（≤5元素）或逐元素显示（>5元素）
     - 数组展开切换：显示前 10 个或所有元素
     - 关键字段用 [*] 高亮标记（精确匹配或后缀模式）
     - 字符串字段使用引号显示
     - 数值精度：浮点数 6 位小数，内联数组 3 位小数
     - 动态库加载类型支持
     - 完整 CDR 反序列化和反射 API
   
   - **原始格式**（解析失败时回退）：
     - 十六进制转储原始消息数据
     - 限制为前 1KB 以保证可读性
     - 无法找到类型库时自动回退

### 🎯 支持的消息类型

本工具使用 ROS2 的反射 API 动态解析消息，支持：

#### ✅ 所有 ROS2 标准包

- **std_msgs**: String、Int32、Float64、Bool、Header 等
- **sensor_msgs**: Image、PointCloud2、LaserScan、Imu、NavSatFix 等
- **geometry_msgs**: PoseStamped、PointStamped、Twist、Transform 等
- **nav_msgs**: Odometry、Path、OccupancyGrid 等
- **tf2_msgs**: TFMessage
- **任何其他标准 ROS2 消息包**

#### ✅ 自定义消息

- 任何包中的用户自定义消息类型
- 通过动态库加载自动加载
- 新消息类型无需重新编译

#### 📝 支持的消息字段类型

- **基本类型**: bool、byte、int8/16/32/64、uint8/16/32/64、float32、float64
- **字符串**: std::string，带适当引号
- **数组**: 
  - 定长数组：直接内存访问
  - 动态数组：std::vector 提取和元素迭代
  - 小型数组（≤5元素）内联显示
  - 大型数组（>5元素）逐元素显示
  - 可配置展开限制（前 10 个或所有）
- **嵌套消息**: 递归解析并 2 空格缩进
- **关键字段高亮**（智能匹配）：
  - 精确匹配：`header`、`stamp`、`timestamp`、`frame_id`、`seq`、`time`、`id`、`name`
  - 后缀模式：`*_id`、`*_name`、`*_time`、`*_stamp`
  - 无误报（如 `width` 不会匹配 `id`）

### 🔍 故障排除

#### 消息显示原始数据而非结构化格式

如果看到十六进制转储而非结构化字段：

1. **检查消息库是否已安装**：

   ```bash
   ros2 pkg list | grep <package_name>
   ```

2. **验证库路径**（库应在标准 ROS2 安装路径中）：
   - `/opt/ros/humble/lib/lib<package>__rosidl_typesupport_*.so`
   - 工具会自动尝试多种库变体

3. **启用详情模式**：在详情视图中按 'D' 切换解析

4. **检查库加载**：
   - 工具尝试 `rosidl_typesupport_cpp`、`rosidl_typesupport_c` 和 `rosidl_typesupport_introspection_cpp`
   - 使用 `dlopen`/`dlsym` 进行动态加载
   - 所有尝试失败后回退到原始十六进制转储

#### ncurses 显示问题

如果界面显示不正确：

```bash
export TERM=xterm-256color
```

#### 编译错误

确保所有依赖项已安装：

```bash
sudo apt-get install libncurses5-dev libncursesw5-dev
```

#### 话题不显示

- 检查 ROS2 话题是否正在发布：`ros2 topic list`
- 验证 ROS_DOMAIN_ID 设置正确
- 按 'R' 手动刷新话题列表

#### 搜索不工作

- 确保在主视图中（非详情视图）
- 按 '/' 激活搜索模式
- 搜索不区分大小写并匹配部分字符串
- 使用 Backspace 删除字符
- 按 Enter 确认搜索，ESC 取消

#### 数组显示问题

- 在详情视图中按 'A' 切换显示前 10 个或所有元素
- 小型数组（≤5元素）自动内联显示
- 大型数组逐元素显示并带索引号

#### 字段对齐问题

- 关键字段使用 `[*]` 前缀标记
- 智能匹配防止误报（如 `width` vs `id`）
- 所有字段每层使用一致的 2 空格缩进

### 📝 许可证

MIT License