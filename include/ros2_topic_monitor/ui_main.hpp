#ifndef ROS2_TOPIC_MONITOR__UI_MAIN_HPP_
#define ROS2_TOPIC_MONITOR__UI_MAIN_HPP_

#include <ncurses.h>
#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
#include "ros2_topic_monitor/topic_discovery.hpp"
#include "ros2_topic_monitor/lazy_subscriber.hpp"

namespace ros2_topic_monitor
{

class UIMain
{
public:
  UIMain();
  ~UIMain();

  // Initialize ncurses
  void init();

  // Cleanup ncurses
  void cleanup();

  // Render main screen
  void render(
    const std::vector<std::string> & topics,
    const std::unordered_map<std::string, std::shared_ptr<LazySubscriber>> & subscribers,
    std::shared_ptr<TopicDiscovery> discovery,
    int selected_index);

  // Get user input (non-blocking)
  int getInput();
  
  // Handle search mode
  bool isSearchMode() const { return search_mode_; }
  void enterSearchMode();
  void exitSearchMode();
  const std::string & getSearchQuery() const { return search_query_; }
  void setSearchQuery(const std::string & query) { search_query_ = query; }
  
  // Get filtered topics based on search
  std::vector<std::string> getFilteredTopics(const std::vector<std::string> & topics) const;
  
  // Page navigation
  int getPageOffset() const { return page_offset_; }
  void setPageOffset(int offset) { page_offset_ = offset; }

private:
  void drawHeader();
  void drawTopicList(
    const std::vector<std::string> & topics,
    const std::unordered_map<std::string, std::shared_ptr<LazySubscriber>> & subscribers,
    std::shared_ptr<TopicDiscovery> discovery,
    int selected_index);
  
  std::string formatBytes(double bytes) const;
  std::string formatDelay(double delay_ms) const;
  
  void drawSearchBar();
  
  int max_rows_;
  int max_cols_;
  
  // Search state
  bool search_mode_;
  std::string search_query_;
  
  // Pagination state
  int page_offset_;  // Starting index for current page
};

}  // namespace ros2_topic_monitor

#endif  // ROS2_TOPIC_MONITOR__UI_MAIN_HPP_
