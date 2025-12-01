#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>
#include "ros2_topic_monitor/topic_discovery.hpp"
#include "ros2_topic_monitor/lazy_subscriber.hpp"
#include "ros2_topic_monitor/ui_main.hpp"
#include "ros2_topic_monitor/ui_detail.hpp"

using namespace ros2_topic_monitor;

std::atomic<bool> g_running{true};

void signalHandler(int signum)
{
  (void)signum;
  g_running.store(false);
}

class TopicMonitorNode : public rclcpp::Node
{
public:
  TopicMonitorNode()
  : Node("topic_monitor")
  {
  }

  void init()
  {
    discovery_ = std::make_shared<TopicDiscovery>(shared_from_this());
    
    refresh_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() {
        refresh_timer_->cancel();
        refresh_timer_ = create_wall_timer(
          std::chrono::milliseconds(800),  // Fast updates - was 2000ms
          std::bind(&TopicMonitorNode::refreshTopics, this));
        
        // Do first refresh
        refreshTopics();
      });
  }

  std::shared_ptr<TopicDiscovery> getDiscovery()
  {
    return discovery_;
  }

  std::unordered_map<std::string, std::shared_ptr<LazySubscriber>> & getSubscribers()
  {
    return subscribers_;
  }

private:
  void refreshTopics()
  {
    discovery_->refreshTopics();
    
    auto topics = discovery_->getTopicList();
    
    // Create subscribers for new topics
    for (const auto & topic_name : topics) {
      if (subscribers_.find(topic_name) == subscribers_.end()) {
        auto topic_info = discovery_->getTopicInfo(topic_name);
        
        try {
          auto subscriber = std::make_shared<LazySubscriber>(
            shared_from_this(),
            topic_name,
            topic_info.type);
          
          subscribers_[topic_name] = subscriber;
        } catch (const std::exception & e) {
          RCLCPP_WARN(get_logger(), "Failed to create subscriber for %s: %s",
            topic_name.c_str(), e.what());
        }
      }
    }
    
    // Remove subscribers for topics that no longer exist
    for (auto it = subscribers_.begin(); it != subscribers_.end(); ) {
      if (!discovery_->hasTopic(it->first)) {
        it = subscribers_.erase(it);
      } else {
        ++it;
      }
    }
  }

  std::shared_ptr<TopicDiscovery> discovery_;
  std::unordered_map<std::string, std::shared_ptr<LazySubscriber>> subscribers_;
  rclcpp::TimerBase::SharedPtr refresh_timer_;
};

int main(int argc, char ** argv)
{
  // Setup signal handler
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);
  
  // Initialize ncurses FIRST to prevent ROS logs from appearing on screen
  UIMain ui_main;
  ui_main.init();
  
  // Show loading screen immediately
  ui_main.showLoadingScreen("Starting ROS2 Topic Monitor...");
  
  // Initialize ROS in background
  std::atomic<bool> ros_ready{false};
  std::shared_ptr<TopicMonitorNode> node;
  
  std::thread init_thread([&]() {
    // Initialize ROS (this may take time)
    rclcpp::init(argc, argv);
    
    // Suppress ROS logging to console to prevent interference with ncurses
    auto ret = rcutils_logging_set_logger_level(
      "ros2_topic_monitor", RCUTILS_LOG_SEVERITY_WARN);
    (void)ret;
    
    // Create node
    node = std::make_shared<TopicMonitorNode>();
    node->init();  // Initialize after shared_ptr is created
    
    ros_ready.store(true);
  });
  
  // Wait for ROS initialization with animated loading (max 3 seconds)
  auto start_time = std::chrono::steady_clock::now();
  int dots = 0;
  while (!ros_ready.load()) {
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - start_time).count();
    
    // Show animated dots
    std::string msg = "Starting ROS2 Topic Monitor";
    for (int i = 0; i < (dots % 4); ++i) {
      msg += ".";
    }
    ui_main.showLoadingScreen(msg);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    dots++;
    
    // Timeout after 5 seconds
    if (elapsed > 5000) {
      ui_main.showLoadingScreen("Initialization timeout!");
      std::this_thread::sleep_for(std::chrono::seconds(1));
      ui_main.cleanup();
      if (init_thread.joinable()) {
        init_thread.join();
      }
      rclcpp::shutdown();
      return 1;
    }
  }
  
  init_thread.join();
  
  // Quick transition message
  ui_main.showLoadingScreen("Ready!");
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  
  // Create executor - CPU optimized with fewer threads
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(),
    2  // Reduced from 4 to 2 threads to lower context switching overhead
  );
  executor.add_node(node);
  
  // Start UI thread with pre-initialized ui_main
  std::thread ui_thread_handle([&node, &ui_main]() {
    UIDetail ui_detail;
    
    int selected_index = 0;
    bool in_detail_view = false;
    std::string selected_topic;
    
    // UX Priority: Fast response for user input - reduced intervals for better responsiveness
    const int IDLE_REFRESH_MS = 100;      // Fast refresh - was 500ms (too slow!)
    const int ACTIVE_REFRESH_MS = 33;     // ~30fps for smooth experience - was 150ms
    auto last_input_time = std::chrono::steady_clock::now();
    
    // Immediately show empty main view to eliminate perceived delay
    std::vector<std::string> empty_topics;
    std::unordered_map<std::string, std::shared_ptr<LazySubscriber>> empty_subs;
    ui_main.render(empty_topics, empty_subs, nullptr, 0);
    
    while (g_running.load()) {
      // Calculate refresh interval based on recent user activity
      auto now = std::chrono::steady_clock::now();
      auto time_since_input = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_input_time).count();
      int refresh_ms = (time_since_input < 2000) ? ACTIVE_REFRESH_MS : IDLE_REFRESH_MS;
      
      if (!in_detail_view) {
        // Main view (same as before, but using passed ui_main)
        auto all_topics = node->getDiscovery()->getTopicList();
        auto & subscribers = node->getSubscribers();
        auto discovery = node->getDiscovery();
        
        auto topics = ui_main.getFilteredTopics(all_topics);
        
        if (!topics.empty()) {
          selected_index = std::max(0, std::min(selected_index, 
            static_cast<int>(topics.size()) - 1));
        }
        
        int max_rows, max_cols;
        getmaxyx(stdscr, max_rows, max_cols);
        int max_visible = max_rows - 6;
        int page_offset = ui_main.getPageOffset();
        
        if (selected_index < page_offset) {
          ui_main.setPageOffset(selected_index);
        } else if (selected_index >= page_offset + max_visible) {
          ui_main.setPageOffset(selected_index - max_visible + 1);
        }
        
        ui_main.render(topics, subscribers, discovery, selected_index);
        
        int ch = ui_main.getInput();
        
        // Track user input for dynamic refresh rate
        if (ch != ERR) {
          last_input_time = std::chrono::steady_clock::now();
        }
        
        if (ui_main.isSearchMode()) {
          switch (ch) {
            case 27:  // ESC
              ui_main.exitSearchMode();
              selected_index = 0;
              break;
            case 10:  // Enter
            case KEY_ENTER:
              if (!topics.empty() && selected_index < static_cast<int>(topics.size())) {
                selected_topic = topics[selected_index];
                ui_main.exitSearchMode();
                in_detail_view = true;
                
                auto it = subscribers.find(selected_topic);
                if (it != subscribers.end()) {
                  it->second->enableDetailMode();
                }
              } else {
                ui_main.exitSearchMode();
              }
              break;
            case KEY_BACKSPACE:
            case 127:
            case 8:
              if (!ui_main.getSearchQuery().empty()) {
                std::string query = ui_main.getSearchQuery();
                query.pop_back();
                ui_main.setSearchQuery(query);
                selected_index = 0;
              }
              break;
            default:
              if (ch >= 32 && ch <= 126) {
                std::string query = ui_main.getSearchQuery() + static_cast<char>(ch);
                ui_main.setSearchQuery(query);
                selected_index = 0;
              }
              break;
          }
        } else {
          switch (ch) {
            case KEY_UP:
              selected_index = std::max(0, selected_index - 1);
              break;
            case KEY_DOWN:
              selected_index = std::min(static_cast<int>(topics.size()) - 1, 
                selected_index + 1);
              break;
            case KEY_PPAGE:
              selected_index = std::max(0, selected_index - max_visible);
              ui_main.setPageOffset(std::max(0, page_offset - max_visible));
              break;
            case KEY_NPAGE:
              selected_index = std::min(static_cast<int>(topics.size()) - 1, 
                selected_index + max_visible);
              if (static_cast<int>(topics.size()) > max_visible) {
                int max_page_offset = static_cast<int>(topics.size()) - max_visible;
                ui_main.setPageOffset(std::min(max_page_offset, page_offset + max_visible));
              }
              break;
            case KEY_HOME:
              selected_index = 0;
              ui_main.setPageOffset(0);
              break;
            case KEY_END:
              if (!topics.empty()) {
                selected_index = static_cast<int>(topics.size()) - 1;
                // Adjust page offset to show last item
                if (static_cast<int>(topics.size()) > max_visible) {
                  ui_main.setPageOffset(static_cast<int>(topics.size()) - max_visible);
                }
              }
              break;
            case '/':
              ui_main.enterSearchMode();
              selected_index = 0;
              break;
            case 10:
            case KEY_ENTER:
              if (!topics.empty() && selected_index < static_cast<int>(topics.size())) {
                selected_topic = topics[selected_index];
                in_detail_view = true;
                
                auto it = subscribers.find(selected_topic);
                if (it != subscribers.end()) {
                  it->second->enableDetailMode();
                }
              }
              break;
            case 'q':
            case 'Q':
              g_running.store(false);
              break;
            case 'r':
            case 'R':
              node->getDiscovery()->refreshTopics();
              break;
          }
        }
      } else {
        // Detail view
        auto & subscribers = node->getSubscribers();
        auto discovery = node->getDiscovery();
        auto it = subscribers.find(selected_topic);
        
        if (it != subscribers.end()) {
          ui_detail.render(selected_topic, it->second, discovery);
        }
        
        int ch = ui_detail.getInput();
        
        // Track user input for dynamic refresh rate
        if (ch != ERR) {
          last_input_time = std::chrono::steady_clock::now();
        }
        
        switch (ch) {
          case 'q':
          case 'Q':
            in_detail_view = false;
            
            if (it != subscribers.end()) {
              it->second->disableDetailMode();
            }
            ui_detail.resetScroll();
            break;
          case 'd':
          case 'D':
            if (it != subscribers.end()) {
              if (it->second->isDetailModeEnabled()) {
                it->second->disableDetailMode();
              } else {
                it->second->enableDetailMode();
              }
            }
            break;
          case 'e':
          case 'E':
            if (it != subscribers.end()) {
              it->second->toggleArrayExpansion();
            }
            break;
          case KEY_UP:
            // Navigate/select line if selection active, otherwise scroll
            if (ui_detail.getSelectedLine() >= 0) {
              ui_detail.selectPrevLine();
            } else {
              ui_detail.scrollUp();
            }
            break;
          case KEY_DOWN:
            // Navigate/select line if selection active, otherwise scroll
            if (ui_detail.getSelectedLine() >= 0) {
              ui_detail.selectNextLine();
            } else {
              ui_detail.scrollDown();
            }
            break;
          case KEY_PPAGE:
            for (int i = 0; i < 10; i++) {
              ui_detail.scrollUp();
            }
            break;
          case KEY_NPAGE:
            for (int i = 0; i < 10; i++) {
              ui_detail.scrollDown();
            }
            break;
        }
      }
      
      std::this_thread::sleep_for(std::chrono::milliseconds(refresh_ms));
    }
    
    ui_main.cleanup();
  });
  
  // Spin executor - balanced for responsiveness and CPU
  while (g_running.load() && rclcpp::ok()) {
    executor.spin_some(std::chrono::milliseconds(50));
  }
  
  // Cleanup
  g_running.store(false);
  
  if (ui_thread_handle.joinable()) {
    ui_thread_handle.join();
  }
  
  rclcpp::shutdown();
  
  return 0;
}
