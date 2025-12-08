#define SDL_MAIN_HANDLED
#include <SDL2/SDL.h>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

class JoyStickSDL {
public:
  JoyStickSDL();
  int InitController();
  std::unordered_map<std::string, int> UpdateState();

  std::unordered_map<std::string, int> getStatus();

  ~JoyStickSDL();

private:
  SDL_GameController *controller_ = nullptr;
  std::unordered_map<std::string, int> JoyStickMetaData{{"a", 0},
                                                        {"b", 0},
                                                        {"x", 0},
                                                        {"y", 0},
                                                        {"dpup", 0},
                                                        {"dpdown", 0},
                                                        {"dpleft", 0},
                                                        {"lefttrigger", 0},
                                                        {"righttrigger", 0},
                                                        {"start", 0},
                                                        {"back", 0},
                                                        {"guide", 0},
                                                        {"leftx", 0},
                                                        {"lefty", 0},
                                                        {"rightx", 0},
                                                        {"righty", 0},
                                                        {"leftshoulder", 0},
                                                        {"rightshoulder", 0}};
  std::mutex tex_;
  bool initialized_{false};
  std::thread run_thread_;
};
class KeyboardSDL {
public:
  KeyboardSDL();
  int InitKeyboard();
  std::unordered_map<std::string, int> UpdateState();
  std::unordered_map<std::string, int> getStatus();
  ~KeyboardSDL();

private:
  SDL_Window *window_ = nullptr;
  SDL_Renderer *renderer_ = nullptr; // 新增渲染器，确保窗口上下文有效
  std::unordered_map<std::string, int> KeyboardMetaData{
      {"a", 0}, {"s", 0}, {"d", 0}, {"w", 0}, {"r", 0}, {"f", 0},
      {"j", 0}, {"k", 0}, {"l", 0}, {"i", 0}, {"p", 0}, {";", 0}};
  std::unordered_map<std::string, int>
      last_status_; // 记录上一帧状态，避免重复输出
  std::mutex tex_;
  bool initialized_{false};
  const Uint8 *keyboard_state_;
};