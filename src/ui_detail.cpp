#include "ros2_topic_monitor/ui_detail.hpp"
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>

namespace ros2_topic_monitor
{

UIDetail::UIDetail()
: max_rows_(0), max_cols_(0), scroll_offset_(0)
{
  fps_history_.reserve(100);
}

UIDetail::~UIDetail()
{
}

void UIDetail::render(
  const std::string & topic_name,
  std::shared_ptr<LazySubscriber> subscriber,
  std::shared_ptr<TopicDiscovery> discovery)
{
  getmaxyx(stdscr, max_rows_, max_cols_);
  
  clear();
  
  drawHeader(topic_name);
  
  if (subscriber) {
    auto metrics = subscriber->getMetrics();
    drawMetrics(metrics);
    
    // Draw QoS information
    if (discovery) {
      auto topic_info = discovery->getTopicInfo(topic_name);
      drawQoS(topic_info);
    }
    
    // Update FPS history for sparkline
    double fps = metrics->getFPS();
    fps_history_.push_back(fps);
    if (fps_history_.size() > 50) {
      fps_history_.erase(fps_history_.begin());
    }
    
    // Draw FPS sparkline
    if (!fps_history_.empty()) {
      attron(A_BOLD);
      mvprintw(13, 2, "FPS History:");
      attroff(A_BOLD);
      drawSparkline(fps_history_, 14, 4, 50);
    }
    
    // Draw message content
    if (subscriber->isDetailModeEnabled()) {
      std::string content = subscriber->getLastMessageContent();
      drawMessageContent(content);
    } else {
      mvprintw(27, 2, "Detail mode not enabled. Press 'D' to enable message parsing.");
    }
  }
  
  // Draw footer
  attron(COLOR_PAIR(5));
  move(max_rows_ - 1, 0);
  for (int i = 0; i < max_cols_; i++) {
    addch(' ');
  }
  mvprintw(max_rows_ - 1, 2, "[Q] Back to Main  [↑/↓] Scroll  [D] Toggle Detail Mode");
  attroff(COLOR_PAIR(5));
  
  refresh();
}

int UIDetail::getInput()
{
  return getch();
}

void UIDetail::drawHeader(const std::string & topic_name)
{
  attron(COLOR_PAIR(5) | A_BOLD);
  move(0, 0);
  for (int i = 0; i < max_cols_; i++) {
    addch(' ');
  }
  
  std::string title = "ROS2 Topic Monitor - Detail View: " + topic_name;
  if (title.length() > static_cast<size_t>(max_cols_ - 4)) {
    title = title.substr(0, max_cols_ - 7) + "...";
  }
  mvprintw(0, 2, "%s", title.c_str());
  attroff(COLOR_PAIR(5) | A_BOLD);
  
  mvhline(1, 0, ACS_HLINE, max_cols_);
}

void UIDetail::drawMetrics(std::shared_ptr<MetricsManager> metrics)
{
  if (!metrics) {
    return;
  }
  
  double fps = metrics->getFPS();
  double delay = metrics->getDelay();
  double bw = metrics->getBandwidth();
  uint64_t count = metrics->getMessageCount();
  
  attron(A_BOLD);
  mvprintw(2, 2, "Metrics:");
  attroff(A_BOLD);
  
  mvprintw(3, 4, "FPS:        %.2f Hz", fps);
  mvprintw(4, 4, "Delay:      %.2f ms", delay);
  mvprintw(5, 4, "Bandwidth:  %.2f B/s", bw);
  mvprintw(6, 4, "Count:      %lu messages", count);
}

void UIDetail::drawQoS(const TopicInfo & topic_info)
{
  if (topic_info.name.empty()) {
    return;
  }
  
  attron(A_BOLD);
  mvprintw(8, 2, "QoS Profile:");
  attroff(A_BOLD);
  
  // Clear the lines to avoid sparkline artifacts
  move(9, 0);
  clrtoeol();
  mvprintw(9, 4, "Reliability: %s", topic_info.qos_reliability.c_str());
  
  move(10, 0);
  clrtoeol();
  mvprintw(10, 4, "Durability:  %s", topic_info.qos_durability.c_str());
  
  move(11, 0);
  clrtoeol();
  if (topic_info.qos_history == "KEEP_LAST") {
    mvprintw(11, 4, "History:     %s (Depth: %d)", 
      topic_info.qos_history.c_str(), topic_info.qos_depth);
  } else {
    mvprintw(11, 4, "History:     %s", topic_info.qos_history.c_str());
  }
}

void UIDetail::drawMessageContent(const std::string & content)
{
  int start_row = 27;
  
  attron(A_BOLD);
  mvprintw(start_row, 2, "Last Message:");
  attroff(A_BOLD);
  
  start_row += 2;
  
  std::istringstream iss(content);
  std::string line;
  int row = start_row;
  int max_content_rows = max_rows_ - start_row - 2;
  int line_count = 0;
  
  // Skip lines based on scroll offset
  for (int i = 0; i < scroll_offset_ && std::getline(iss, line); i++) {
    // Skip
  }
  
  while (std::getline(iss, line) && line_count < max_content_rows) {
    // Truncate line if too long
    if (line.length() > static_cast<size_t>(max_cols_ - 4)) {
      line = line.substr(0, max_cols_ - 7) + "...";
    }
    
    mvprintw(row, 4, "%s", line.c_str());
    row++;
    line_count++;
  }
}

void UIDetail::drawSparkline(
  const std::vector<double> & data,
  int row,
  int col,
  int width)
{
  if (data.empty()) {
    return;
  }
  
  // Find min and max
  double min_val = *std::min_element(data.begin(), data.end());
  double max_val = *std::max_element(data.begin(), data.end());
  
  if (max_val - min_val < 0.01) {
    max_val = min_val + 1.0;
  }
  
  // Draw a bar chart with 8 rows of height
  const int chart_height = 8;
  
  size_t start_idx = data.size() > static_cast<size_t>(width) ? 
    data.size() - width : 0;
  size_t data_count = std::min(data.size() - start_idx, static_cast<size_t>(width));
  
  // Draw from top to bottom
  for (int h = chart_height - 1; h >= 0; h--) {
    move(row + (chart_height - 1 - h), col);
    
    for (size_t i = 0; i < data_count; i++) {
      double value = data[start_idx + i];
      double normalized = (value - min_val) / (max_val - min_val);
      double threshold = static_cast<double>(h) / chart_height;
      
      if (normalized >= threshold) {
        addch('|');  // Bar filled
      } else {
        addch(' ');  // Bar empty
      }
    }
  }
  
  // Draw baseline
  move(row + chart_height, col);
  for (size_t i = 0; i < data_count; i++) {
    addch('-');
  }
  
  // Show scale
  mvprintw(row + chart_height + 1, col, "Max: %.1f Hz", max_val);
  mvprintw(row + chart_height + 2, col, "Min: %.1f Hz", min_val);
}

}  // namespace ros2_topic_monitor
