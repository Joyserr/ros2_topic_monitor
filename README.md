# ROS2 Topic Monitor

[English](#english) | [中文](README_CN.md)

---

## English

A high-performance ROS2 topic monitoring tool with an interactive ncurses TUI interface, featuring structured message parsing and real-time metrics visualization.

### ✨ Features

- **High Performance**: Built with C++17 + rclcpp, extremely low CPU usage (< 5%)
- **Interactive TUI**: htop-like terminal user interface powered by ncurses
- **Auto Topic Discovery**: Automatically discovers all topics using ROS2 graph API
- **Lazy Loading**: Main view only tracks metadata; message parsing happens in detail view
- **Multi-threaded**: Uses MultiThreadedExecutor for high concurrency
- **Real-time Metrics**: Live display of FPS, latency, bandwidth and message count
- **Thread-safe**: Guaranteed data consistency with shared_mutex and atomic operations
- **QoS Display**: Shows QoS configuration (Reliability, Durability, History) for each topic
- **Structured Message Parsing**: Dynamically parses and displays message content in human-readable format
- **Array Support**: Fully supports fixed-size and dynamic arrays with element-by-element display
- **Search & Filter**: Real-time topic search with case-insensitive matching
- **Pagination**: Automatic pagination for large topic lists with Page Up/Down navigation
- **FPS History Chart**: Visual trend chart showing message frequency over time

### 📋 Requirements

- ROS2 Humble
- ncurses library
- C++17 compatible compiler
- Standard ROS2 message packages (std_msgs, sensor_msgs, geometry_msgs, etc.)

### 🔧 Build and Installation

```bash
cd ~/colcon_ws
colcon build --packages-select ros2_topic_monitor
source install/setup.bash
```

### 🚀 Usage

Launch the monitor:

```bash
ros2 run ros2_topic_monitor monitor
```

#### Main View Controls

- **↑/↓**: Navigate through topics (line by line)
- **PgUp/PgDn**: Navigate through pages (when topics exceed screen size)
- **/**: Enter search mode
- **Enter**: Enter topic detail view for selected topic
- **Q**: Quit application
- **R**: Manually refresh topic list

#### Search Mode Controls

- **Type**: Enter search keywords (case-insensitive, matches anywhere in topic name)
- **Backspace**: Delete last character
- **Enter**: Exit search mode and view filtered results
- **ESC**: Clear search and return to full topic list

#### Detail View Controls

- **Q**: Return to main view
- **D**: Toggle message content parsing (enable/disable structured display)
- **A**: Toggle array expansion (show all elements vs. first 10)
- **↑/↓**: Scroll message content
- **PgUp/PgDn**: Fast scroll (10 lines at a time)

### 📊 Display Information

#### Main View

Shows real-time statistics for all topics:

| Column | Description |
|--------|-------------|
| Topic | Topic name (truncated if too long) |
| Type | Message type (shows package/msg/Type) |
| FPS | Messages per second (frequency) |
| Delay(ms) | Average message delay |
| BW(B/s) | Bandwidth in bytes per second |

**Footer Information:**
- Left: Control key hints
- Right: Pagination info (e.g., "Page 1/3 (25 topics)")

#### Detail View

Shows comprehensive information for selected topic:

1. **Real-time Metrics** (rows 2-6):
   - FPS: Message frequency in Hz
   - Delay: Average latency in milliseconds
   - Bandwidth: Data rate in bytes/second
   - Count: Total messages received

2. **QoS Profile** (rows 8-11):
   - Reliability: BEST_EFFORT or RELIABLE
   - Durability: VOLATILE or TRANSIENT_LOCAL
   - History: KEEP_LAST (with depth) or KEEP_ALL
   - *Key fields marked with [*]*

3. **FPS History Chart** (rows 13-15):
   - Visual bar chart showing message frequency trend
   - Each column represents a time point
   - Height indicates FPS value (higher = more messages)
   - Range displayed below chart

4. **Message Content** (row 16+):
   - **Structured Format** (when parsing enabled):
     - Hierarchical display of all message fields
     - Nested messages with proper indentation (2 spaces per level)
     - Arrays displayed inline (≤5 elements) or per-element (>5 elements)
     - Array expansion toggle: show first 10 or all elements
     - Key fields highlighted with [*] marker (exact match or suffix patterns)
     - String fields shown in quotes
     - Numeric precision: 6 decimals for float/double, 3 for inline arrays
     - Dynamic library loading for type support
     - Full CDR deserialization with introspection API
   
   - **Raw Format** (fallback when parsing fails):
     - Hexadecimal dump of raw message data
     - Limited to first 1KB for readability
     - Auto-fallback when type library not found

### 🎯 Supported Message Types

The tool uses ROS2's introspection API to dynamically parse messages, supporting:

#### ✅ All ROS2 Standard Packages
- **std_msgs**: String, Int32, Float64, Bool, Header, etc.
- **sensor_msgs**: Image, PointCloud2, LaserScan, Imu, NavSatFix, etc.
- **geometry_msgs**: PoseStamped, PointStamped, Twist, Transform, etc.
- **nav_msgs**: Odometry, Path, OccupancyGrid, etc.
- **tf2_msgs**: TFMessage
- **Any other standard ROS2 message package**

#### ✅ Custom Messages
- User-defined message types in any package
- Automatically loaded via dynamic library loading
- No recompilation needed for new message types

#### 📝 Message Field Types Supported
- **Primitives**: bool, byte, int8/16/32/64, uint8/16/32/64, float32, float64
- **Strings**: std::string with proper quoting
- **Arrays**: 
  - Fixed-size arrays: Direct memory access
  - Dynamic arrays: std::vector extraction and element iteration
  - Inline display for small arrays (≤5 elements)
  - Per-element display for large arrays (>5 elements)
  - Configurable expansion limit (first 10 or all)
- **Nested Messages**: Recursive parsing with 2-space indentation
- **Key Field Highlighting** (smart matching):
  - Exact match: `header`, `stamp`, `timestamp`, `frame_id`, `seq`, `time`, `id`, `name`
  - Suffix patterns: `*_id`, `*_name`, `*_time`, `*_stamp`
  - No false positives (e.g., `width` won't match `id`)

### 🔍 Troubleshooting

#### Messages Show Raw Data Instead of Structured Format

If you see hex dump instead of structured fields:

1. **Check message library is installed**:
   ```bash
   ros2 pkg list | grep <package_name>
   ```

2. **Verify library path** (libraries should be in standard ROS2 install paths):
   - `/opt/ros/humble/lib/lib<package>__rosidl_typesupport_*.so`
   - The tool tries multiple library variants automatically

3. **Enable detail mode**: Press 'D' in detail view to toggle parsing

4. **Check library loading**:
   - Tool attempts `rosidl_typesupport_cpp`, `rosidl_typesupport_c`, and `rosidl_typesupport_introspection_cpp`
   - Uses `dlopen`/`dlsym` for dynamic loading
   - Falls back to raw hex dump if all attempts fail

#### ncurses Display Issues

If the interface displays incorrectly:

```bash
export TERM=xterm-256color
```

#### Build Errors

Ensure all dependencies are installed:

```bash
sudo apt-get install libncurses5-dev libncursesw5-dev
sudo apt-get install ros-humble-geometry-msgs  # If not already installed
```

#### Topics Not Showing

- Check if ROS2 topics are being published: `ros2 topic list`
- Verify ROS_DOMAIN_ID is set correctly
- Press 'R' to manually refresh the topic list

#### Search Not Working

- Ensure you're in main view (not detail view)
- Press '/' to activate search mode
- Search is case-insensitive and matches partial strings
- Use Backspace to delete characters
- Press Enter to confirm search, ESC to cancel

#### Array Display Issues

- Press 'A' in detail view to toggle between showing first 10 or all elements
- Small arrays (≤5 elements) display inline automatically
- Large arrays show per-element with index numbers

#### Field Alignment Issues

- Key fields are marked with `[*]` prefix
- Smart matching prevents false positives (e.g., `width` vs `id`)
- All fields use consistent 2-space indentation per nesting level

### 📝 License

MIT License