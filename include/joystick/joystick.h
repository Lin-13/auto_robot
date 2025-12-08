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