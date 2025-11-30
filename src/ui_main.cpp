#include "ros2_topic_monitor/ui_main.hpp"
#include <sstream>
#include <iomanip>
#include <algorithm>

namespace ros2_topic_monitor
{

UIMain::UIMain()
: max_rows_(0), max_cols_(0)
{
}

UIMain::~UIMain()
{
  cleanup();
}

void UIMain::init()
{
  initscr();
  cbreak();
  noecho();
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);  // Non-blocking input
  curs_set(0);  // Hide cursor
  
  // Enable colors
  if (has_colors()) {
    start_color();
    init_pair(1, COLOR_CYAN, COLOR_BLACK);
    init_pair(2, COLOR_GREEN, COLOR_BLACK);
    init_pair(3, COLOR_YELLOW, COLOR_BLACK);
    init_pair(4, COLOR_RED, COLOR_BLACK);
    init_pair(5, COLOR_WHITE, COLOR_BLUE);
  }
  
  getmaxyx(stdscr, max_rows_, max_cols_);
}

void UIMain::cleanup()
{
  endwin();
}

void UIMain::render(
  const std::vector<std::string> & topics,
  const std::unordered_map<std::string, std::shared_ptr<LazySubscriber>> & subscribers,
  std::shared_ptr<TopicDiscovery> discovery,
  int selected_index)
{
  clear();
  
  drawHeader();
  drawTopicList(topics, subscribers, discovery, selected_index);
  
  // Draw footer
  attron(COLOR_PAIR(5));
  move(max_rows_ - 1, 0);
  for (int i = 0; i < max_cols_; i++) {
    addch(' ');
  }
  mvprintw(max_rows_ - 1, 2, "[↑/↓] Navigate  [ENTER] Details  [Q] Quit  [R] Refresh");
  attroff(COLOR_PAIR(5));
  
  refresh();
}

int UIMain::getInput()
{
  return getch();
}

void UIMain::drawHeader()
{
  attron(COLOR_PAIR(5) | A_BOLD);
  move(0, 0);
  for (int i = 0; i < max_cols_; i++) {
    addch(' ');
  }
  mvprintw(0, 2, "ROS2 Topic Monitor - Main View");
  attroff(COLOR_PAIR(5) | A_BOLD);
  
  // Column headers
  attron(A_BOLD);
  mvprintw(2, 2, "%-40s %-30s %8s %10s %12s", 
    "Topic", "Type", "FPS", "Delay(ms)", "BW(B/s)");
  attroff(A_BOLD);
  
  mvhline(3, 0, ACS_HLINE, max_cols_);
}

void UIMain::drawTopicList(
  const std::vector<std::string> & topics,
  const std::unordered_map<std::string, std::shared_ptr<LazySubscriber>> & subscribers,
  std::shared_ptr<TopicDiscovery> discovery,
  int selected_index)
{
  int start_row = 4;
  int max_visible = max_rows_ - 6;  // Leave room for header and footer
  
  for (size_t i = 0; i < topics.size() && i < static_cast<size_t>(max_visible); i++) {
    const auto & topic = topics[i];
    int row = start_row + i;
    
    // Highlight selected row
    if (static_cast<int>(i) == selected_index) {
      attron(A_REVERSE);
    }
    
    // Get subscriber info
    auto it = subscribers.find(topic);
    if (it != subscribers.end()) {
      auto metrics = it->second->getMetrics();
      
      double fps = metrics->getFPS();
      double delay = metrics->getDelay();
      double bw = metrics->getBandwidth();
      
      // Truncate topic name if too long
      std::string topic_display = topic;
      if (topic_display.length() > 38) {
        topic_display = topic_display.substr(0, 35) + "...";
      }
      
      // Get topic type from discovery
      std::string type_display = "unknown";
      if (discovery) {
        auto topic_info = discovery->getTopicInfo(topic);
        if (!topic_info.type.empty()) {
          type_display = topic_info.type;
          // Simplify type name (remove package prefix if too long)
          size_t last_slash = type_display.find_last_of('/');
          if (last_slash != std::string::npos && type_display.length() > 28) {
            type_display = type_display.substr(last_slash + 1);
          }
        }
      }
      
      if (type_display.length() > 28) {
        type_display = type_display.substr(0, 25) + "...";
      }
      
      mvprintw(row, 2, "%-40s %-30s %8.1f %10.1f %12s",
        topic_display.c_str(),
        type_display.c_str(),
        fps,
        delay,
        formatBytes(bw).c_str());
    } else {
      mvprintw(row, 2, "%-40s %-30s %8s %10s %12s",
        topic.c_str(), "N/A", "-", "-", "-");
    }
    
    if (static_cast<int>(i) == selected_index) {
      attroff(A_REVERSE);
    }
  }
  
  // Show scroll indicator if needed
  if (topics.size() > static_cast<size_t>(max_visible)) {
    mvprintw(max_rows_ - 2, max_cols_ - 20, "(%zu/%zu topics)", 
      std::min(static_cast<size_t>(selected_index + 1), topics.size()), 
      topics.size());
  }
}

std::string UIMain::formatBytes(double bytes) const
{
  std::ostringstream oss;
  
  if (bytes < 1024) {
    oss << std::fixed << std::setprecision(0) << bytes;
  } else if (bytes < 1024 * 1024) {
    oss << std::fixed << std::setprecision(1) << (bytes / 1024) << "K";
  } else {
    oss << std::fixed << std::setprecision(1) << (bytes / 1024 / 1024) << "M";
  }
  
  return oss.str();
}

std::string UIMain::formatDelay(double delay_ms) const
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(1) << delay_ms;
  return oss.str();
}

}  // namespace ros2_topic_monitor
