# ROS2 Topic Monitor

## English

A high-performance ROS2 topic monitoring tool with an interactive ncurses TUI interface, similar to htop.

### ✨ Features

- **High Performance**: Built with C++17 + rclcpp, extremely low CPU usage (< 5%)
- **Interactive TUI**: htop-like terminal user interface powered by ncurses
- **Auto Topic Discovery**: Automatically discovers all topics using ROS graph API
- **Lazy Loading**: Main view only tracks metadata; message parsing happens in detail view
- **Multi-threaded**: Uses MultiThreadedExecutor for high concurrency
- **Real-time Metrics**: Live display of FPS, latency, bandwidth and more
- **Thread-safe**: Guaranteed data consistency with shared_mutex and atomic operations

### 📋 Requirements

- ROS2 Humble
- ncurses library
- C++17 compatible compiler

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

- **↑/↓**: Navigate through topics
- **Enter**: Enter topic detail view
- **Q**: Quit application
- **R**: Manually refresh topic list

#### Detail View Controls

- **Q**: Return to main view
- **D**: Toggle message content parsing
- **↑/↓**: Scroll message content

### 📊 Display Information

#### Main View

Shows real-time statistics for all topics:

- Topic name
- Message type
- FPS (messages per second)
- Delay (milliseconds)
- Bandwidth (bytes/second)

#### Detail View

Shows detailed information for selected topic:

- Real-time metric statistics
- FPS history chart (ASCII sparkline)
- Latest message content (hex dump)

### 🔍 Troubleshooting

#### ncurses Display Issues

If the interface displays incorrectly:

```bash
export TERM=xterm-256color
```

#### Build Errors

Ensure ncurses development libraries are installed:

```bash
sudo apt-get install libncurses5-dev libncursesw5-dev
```

#### Topics Not Showing

- Check if ROS2 topics are being published: `ros2 topic list`
- Verify ROS_DOMAIN_ID is set correctly

### 📝 License

MIT License