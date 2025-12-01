#include "ros2_topic_monitor/ui_main.hpp"
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cctype>

namespace ros2_topic_monitor
{

UIMain::UIMain()
: max_rows_(0), max_cols_(0), search_mode_(false), search_query_(""), page_offset_(0)
{
}

UIMain::~UIMain()
{
  cleanup();
}

void UIMain::init()
{
  // Initialize ncurses with optimized settings to reduce flicker
  initscr();
  
  // Immediately configure ncurses before any output
  cbreak();
  noecho();
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);  // Non-blocking input
  curs_set(0);  // Hide cursor
  
  // Enable colors before first draw
  if (has_colors()) {
    start_color();
    use_default_colors();  // Use terminal default colors for transparency
    init_pair(1, COLOR_CYAN, -1);
    init_pair(2, COLOR_GREEN, -1);
    init_pair(3, COLOR_YELLOW, -1);
    init_pair(4, COLOR_RED, -1);
    init_pair(5, COLOR_WHITE, COLOR_BLUE);
    init_pair(6, COLOR_BLACK, COLOR_CYAN);  // Selected row background
  }
  
  getmaxyx(stdscr, max_rows_, max_cols_);
  
  // Single clear and refresh - no intermediate displays
  clear();
  refresh();
}

void UIMain::cleanup()
{
  endwin();
}

void UIMain::showLoadingScreen(const std::string & message)
{
  clear();
  
  int msg_row = max_rows_ / 2;
  int msg_col = (max_cols_ - message.length()) / 2;
  
  // Draw a simple box around the message
  attron(COLOR_PAIR(5) | A_BOLD);
  
  // Top border
  mvhline(msg_row - 1, msg_col - 2, ' ', message.length() + 4);
  
  // Message line with border
  mvaddch(msg_row, msg_col - 2, ' ');
  mvaddch(msg_row, msg_col - 1, ' ');
  mvprintw(msg_row, msg_col, "%s", message.c_str());
  mvaddch(msg_row, msg_col + message.length(), ' ');
  mvaddch(msg_row, msg_col + message.length() + 1, ' ');
  
  // Bottom border
  mvhline(msg_row + 1, msg_col - 2, ' ', message.length() + 4);
  
  attroff(COLOR_PAIR(5) | A_BOLD);
  
  refresh();
}

void UIMain::render(
  const std::vector<std::string> & topics,
  const std::unordered_map<std::string, std::shared_ptr<LazySubscriber>> & subscribers,
  std::shared_ptr<TopicDiscovery> discovery,
  int selected_index)
{
  clear();
  
  // Get filtered topics based on search
  auto filtered_topics = getFilteredTopics(topics);
  
  drawHeader();
  drawTopicList(filtered_topics, subscribers, discovery, selected_index);
  
  // Calculate pagination info
  int max_visible = max_rows_ - 6;
  int total_pages = (filtered_topics.size() + max_visible - 1) / max_visible;
  int current_page = (page_offset_ / max_visible) + 1;
  
  // Draw footer
  attron(COLOR_PAIR(5));
  move(max_rows_ - 1, 0);
  for (int i = 0; i < max_cols_; i++) {
    addch(' ');
  }
  
  if (search_mode_) {
    mvprintw(max_rows_ - 1, 2, "[ESC] Exit Search  [ENTER] Confirm");
  } else {
    std::ostringstream footer;
    footer << "[↑/↓] Navigate  [Home/End] First/Last  [PgUp/PgDn] Page  [ENTER] Details  [/] Search  [Q] Quit";
    mvprintw(max_rows_ - 1, 2, "%s", footer.str().c_str());
    
    // Show pagination info on the right
    if (total_pages > 1) {
      std::ostringstream page_info;
      page_info << "Page " << current_page << "/" << total_pages 
                << " (" << filtered_topics.size() << " topics)";
      mvprintw(max_rows_ - 1, max_cols_ - page_info.str().length() - 2, 
               "%s", page_info.str().c_str());
    }
  }
  attroff(COLOR_PAIR(5));
  
  // Draw search bar if in search mode
  if (search_mode_) {
    drawSearchBar();
  }
  
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
  
  // Handle empty topic list
  if (topics.empty()) {
    attron(COLOR_PAIR(3));
    mvprintw(start_row + 2, max_cols_ / 2 - 15, "No topics discovered yet...");
    attroff(COLOR_PAIR(3));
    return;
  }
  
  // Calculate visible range based on page_offset
  size_t display_start = page_offset_;
  size_t display_end = std::min(display_start + max_visible, topics.size());
  
  for (size_t i = display_start; i < display_end; i++) {
    const auto & topic = topics[i];
    int row = start_row + (i - display_start);
    
    // Highlight selected row with custom color
    if (static_cast<int>(i) == selected_index) {
      attron(COLOR_PAIR(6) | A_BOLD);
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
      attroff(COLOR_PAIR(6) | A_BOLD);
    }
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

void UIMain::enterSearchMode()
{
  search_mode_ = true;
  search_query_.clear();
  curs_set(1);  // Show cursor
}

void UIMain::exitSearchMode()
{
  search_mode_ = false;
  search_query_.clear();
  curs_set(0);  // Hide cursor
  page_offset_ = 0;  // Reset pagination when exiting search
}

std::vector<std::string> UIMain::getFilteredTopics(
  const std::vector<std::string> & topics) const
{
  if (search_query_.empty()) {
    return topics;
  }
  
  std::vector<std::string> filtered;
  std::string query_lower = search_query_;
  std::transform(query_lower.begin(), query_lower.end(), query_lower.begin(),
    [](unsigned char c) { return std::tolower(c); });
  
  for (const auto & topic : topics) {
    std::string topic_lower = topic;
    std::transform(topic_lower.begin(), topic_lower.end(), topic_lower.begin(),
      [](unsigned char c) { return std::tolower(c); });
    
    if (topic_lower.find(query_lower) != std::string::npos) {
      filtered.push_back(topic);
    }
  }
  
  return filtered;
}

void UIMain::drawSearchBar()
{
  // Draw search bar at the top
  attron(COLOR_PAIR(3) | A_BOLD);
  mvprintw(1, 2, "Search: %s", search_query_.c_str());
  attroff(COLOR_PAIR(3) | A_BOLD);
  
  // Position cursor at end of search query
  move(1, 10 + search_query_.length());
}

}  // namespace ros2_topic_monitor
