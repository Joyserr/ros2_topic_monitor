#include "ros2_topic_monitor/ui_detail.hpp"
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>

namespace ros2_topic_monitor
{

UIDetail::UIDetail()
: max_rows_(0), max_cols_(0), scroll_offset_(0), total_content_lines_(0), max_visible_lines_(0), selected_line_(-1)
{
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
        
    // Draw message content
    if (subscriber->isDetailModeEnabled()) {
      std::string content = subscriber->getLastMessageContent();
      drawMessageContent(content);
    } else {
      mvprintw(15, 2, "Detail mode not enabled. Press 'D' to enable message parsing.");
    }
  }
  
  // Draw footer
  attron(COLOR_PAIR(5));
  move(max_rows_ - 1, 0);
  for (int i = 0; i < max_cols_; i++) {
    addch(' ');
  }
  
  // Show array expansion status
  std::string array_status = subscriber->isArrayExpansionEnabled() ? "ON" : "OFF";
  std::ostringstream footer;
  footer << "[Q] Back  [↑/↓] Navigate/Scroll  [PgUp/PgDn] Page  [D] Detail  [E] Expand Arrays (" << array_status << ")";
  mvprintw(max_rows_ - 1, 2, "%s", footer.str().c_str());
  attroff(COLOR_PAIR(5));
  
  refresh();
}

int UIDetail::getInput()
{
  return getch();
}

void UIDetail::scrollUp()
{
  if (scroll_offset_ > 0) {
    scroll_offset_--;
  }
}

void UIDetail::scrollDown()
{
  // Only scroll down if there's more content below the visible area
  if (total_content_lines_ > max_visible_lines_) {
    int max_scroll = total_content_lines_ - max_visible_lines_;
    if (scroll_offset_ < max_scroll) {
      scroll_offset_++;
    }
  }
}

void UIDetail::resetScroll()
{
  scroll_offset_ = 0;
  selected_line_ = -1;  // Reset selection as well
}

void UIDetail::selectPrevLine()
{
  if (selected_line_ < 0) {
    selected_line_ = 0;  // Start selection from first line
  } else if (selected_line_ > 0) {
    selected_line_--;
    // Auto-scroll if selection goes above visible area
    if (selected_line_ < scroll_offset_) {
      scroll_offset_ = selected_line_;
    }
  }
}

void UIDetail::selectNextLine()
{
  if (selected_line_ < 0) {
    selected_line_ = 0;  // Start selection from first line
  } else if (selected_line_ < total_content_lines_ - 1) {
    selected_line_++;
    // Auto-scroll if selection goes below visible area
    int last_visible = scroll_offset_ + max_visible_lines_ - 1;
    if (selected_line_ > last_visible) {
      scroll_offset_++;
    }
  }
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
  int start_row = 13;
  
  attron(A_BOLD);
  mvprintw(start_row, 2, "Last Message:");
  attroff(A_BOLD);
  
  start_row += 2;
  
  // Count total lines first
  std::istringstream iss_count(content);
  std::string line;
  int total_lines = 0;
  while (std::getline(iss_count, line)) {
    total_lines++;
  }
  
  // Update member variables for scroll boundary checking
  int max_content_rows = max_rows_ - start_row - 2;
  total_content_lines_ = total_lines;
  max_visible_lines_ = max_content_rows;
  
  // Clamp scroll_offset to valid range
  if (total_lines > max_content_rows) {
    int max_scroll = total_lines - max_content_rows;
    if (scroll_offset_ > max_scroll) {
      scroll_offset_ = max_scroll;
    }
  } else {
    scroll_offset_ = 0;  // No scrolling needed if content fits on screen
  }
  
  // Display scroll position indicator
  if (total_lines > 0) {
    std::ostringstream scroll_info;
    scroll_info << "(Line " << (scroll_offset_ + 1) << "-" 
                << std::min(scroll_offset_ + max_content_rows, total_lines) 
                << " of " << total_lines << ")";
    mvprintw(start_row - 1, max_cols_ - scroll_info.str().length() - 2, 
             "%s", scroll_info.str().c_str());
  }
  
  std::istringstream iss(content);
  int row = start_row;
  int line_count = 0;
  int absolute_line = 0;  // Track absolute line number in content
  
  // Skip lines based on scroll offset
  for (int i = 0; i < scroll_offset_ && std::getline(iss, line); i++) {
    absolute_line++;
  }
  
  while (std::getline(iss, line) && line_count < max_content_rows) {
    // Highlight selected line
    if (absolute_line == selected_line_) {
      attron(COLOR_PAIR(6) | A_BOLD);
    }
    
    // Truncate line if too long
    if (line.length() > static_cast<size_t>(max_cols_ - 4)) {
      line = line.substr(0, max_cols_ - 7) + "...";
    }
    
    mvprintw(row, 4, "%s", line.c_str());
    
    if (absolute_line == selected_line_) {
      attroff(COLOR_PAIR(6) | A_BOLD);
    }
    
    row++;
    line_count++;
    absolute_line++;
  }
}


}  // namespace ros2_topic_monitor
