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

private:
  void drawHeader(const std::string & topic_name);
  void drawMetrics(std::shared_ptr<MetricsManager> metrics);
  void drawQoS(const TopicInfo & topic_info);
  void drawMessageContent(const std::string & content);
  void drawSparkline(const std::vector<double> & data, int row, int col, int width);
  
  int max_rows_;
  int max_cols_;
  int scroll_offset_;
  
  std::vector<double> fps_history_;
};

}  // namespace ros2_topic_monitor

#endif  // ROS2_TOPIC_MONITOR__UI_DETAIL_HPP_
