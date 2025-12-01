#ifndef ROS2_TOPIC_MONITOR__UI_DETAIL_HPP_
#define ROS2_TOPIC_MONITOR__UI_DETAIL_HPP_

#include <ncurses.h>
#include <string>
#include <memory>
#include <vector>
#include "ros2_topic_monitor/lazy_subscriber.hpp"
#include "ros2_topic_monitor/topic_discovery.hpp"

namespace ros2_topic_monitor
{

class UIDetail
{
public:
  UIDetail();
  ~UIDetail();

  // Render detail screen for a topic
  void render(
    const std::string & topic_name,
    std::shared_ptr<LazySubscriber> subscriber,
    std::shared_ptr<TopicDiscovery> discovery);

  // Get user input (non-blocking)
  int getInput();
  
  // Scroll control
  void scrollUp();
  void scrollDown();
  void resetScroll();
  
  // Line selection control for message content
  void selectPrevLine();
  void selectNextLine();
  int getSelectedLine() const { return selected_line_; }

private:
  void drawHeader(const std::string & topic_name);
  void drawMetrics(std::shared_ptr<MetricsManager> metrics);
  void drawQoS(const TopicInfo & topic_info);
  void drawMessageContent(const std::string & content);
  
  int max_rows_;
  int max_cols_;
  int scroll_offset_;
  int total_content_lines_;  // Total lines in current message content
  int max_visible_lines_;    // Maximum visible lines in content area
  int selected_line_;        // Currently selected line in message content (-1 = no selection)
};

}  // namespace ros2_topic_monitor

#endif  // ROS2_TOPIC_MONITOR__UI_DETAIL_HPP_
