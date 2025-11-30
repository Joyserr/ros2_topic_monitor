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
    
    // Refresh topics periodically
    refresh_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&TopicMonitorNode::refreshTopics, this));
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

void uiThread(std::shared_ptr<TopicMonitorNode> node)
{
  UIMain ui_main;
  UIDetail ui_detail;
  
  ui_main.init();
  
  int selected_index = 0;
  bool in_detail_view = false;
  std::string selected_topic;
  
  while (g_running.load()) {
    if (!in_detail_view) {
      // Main view
      auto all_topics = node->getDiscovery()->getTopicList();
      auto & subscribers = node->getSubscribers();
      auto discovery = node->getDiscovery();
      
      // Get filtered topics
      auto topics = ui_main.getFilteredTopics(all_topics);
      
      // Clamp selected index
      if (!topics.empty()) {
        selected_index = std::max(0, std::min(selected_index, 
          static_cast<int>(topics.size()) - 1));
      }
      
      // Ensure page_offset is valid
      int max_rows, max_cols;
      getmaxyx(stdscr, max_rows, max_cols);
      int max_visible = max_rows - 6;
      int page_offset = ui_main.getPageOffset();
      
      // Auto-scroll to keep selected item visible
      if (selected_index < page_offset) {
        ui_main.setPageOffset(selected_index);
      } else if (selected_index >= page_offset + max_visible) {
        ui_main.setPageOffset(selected_index - max_visible + 1);
      }
      
      ui_main.render(topics, subscribers, discovery, selected_index);
      
      int ch = ui_main.getInput();
      
      if (ui_main.isSearchMode()) {
        // Handle search mode input
        switch (ch) {
          case 27:  // ESC
            ui_main.exitSearchMode();
            selected_index = 0;
            break;
          case 10:  // Enter - Enter detail view for selected topic
          case KEY_ENTER:
            if (!topics.empty() && selected_index < static_cast<int>(topics.size())) {
              selected_topic = topics[selected_index];
              ui_main.exitSearchMode();  // Exit search mode
              in_detail_view = true;  // Enter detail view
              
              // Enable detail mode for selected topic
              auto it = subscribers.find(selected_topic);
              if (it != subscribers.end()) {
                it->second->enableDetailMode();
              }
            } else {
              // If no topic selected, just exit search mode
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
              selected_index = 0;  // Reset selection
            }
            break;
          default:
            if (ch >= 32 && ch <= 126) {  // Printable characters
              std::string query = ui_main.getSearchQuery() + static_cast<char>(ch);
              ui_main.setSearchQuery(query);
              selected_index = 0;  // Reset selection
            }
            break;
        }
      } else {
        // Handle normal navigation
        switch (ch) {
          case KEY_UP:
            selected_index = std::max(0, selected_index - 1);
            break;
          case KEY_DOWN:
            selected_index = std::min(static_cast<int>(topics.size()) - 1, 
              selected_index + 1);
            break;
          case KEY_PPAGE:  // Page Up
            selected_index = std::max(0, selected_index - max_visible);
            ui_main.setPageOffset(std::max(0, page_offset - max_visible));
            break;
          case KEY_NPAGE:  // Page Down
            selected_index = std::min(static_cast<int>(topics.size()) - 1, 
              selected_index + max_visible);
            ui_main.setPageOffset(std::min(static_cast<int>(topics.size()) - max_visible, 
              page_offset + max_visible));
            break;
          case '/':  // Enter search mode
            ui_main.enterSearchMode();
            selected_index = 0;
            break;
          case 10:  // Enter
          case KEY_ENTER:
            if (!topics.empty() && selected_index < static_cast<int>(topics.size())) {
              selected_topic = topics[selected_index];
              in_detail_view = true;
              
              // Enable detail mode for selected topic
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
      
      switch (ch) {
        case 'q':
        case 'Q':
          // Go back to main view
          in_detail_view = false;
          
          // Disable detail mode
          if (it != subscribers.end()) {
            it->second->disableDetailMode();
          }
          break;
        case 'd':
        case 'D':
          // Toggle detail mode
          if (it != subscribers.end()) {
            if (it->second->isDetailModeEnabled()) {
              it->second->disableDetailMode();
            } else {
              it->second->enableDetailMode();
            }
          }
          break;
      }
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  
  ui_main.cleanup();
}

int main(int argc, char ** argv)
{
  // Setup signal handler
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);
  
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<TopicMonitorNode>();
  node->init();  // Initialize after shared_ptr is created
  
  // Create executor with multiple threads
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(),
    4  // 4 threads
  );
  executor.add_node(node);
  
  // Start UI thread
  std::thread ui_thread_handle(uiThread, node);
  
  // Spin executor
  while (g_running.load() && rclcpp::ok()) {
    executor.spin_some(std::chrono::milliseconds(100));
  }
  
  // Cleanup
  g_running.store(false);
  
  if (ui_thread_handle.joinable()) {
    ui_thread_handle.join();
  }
  
  rclcpp::shutdown();
  
  return 0;
}
