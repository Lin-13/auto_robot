#include "joystick/joystick.h"
#include <iostream>
class GamePad {
public:
  void update(const std::unordered_map<std::string, int> &data,
              double scale = 1) {
    if (data.count("leftx")) {
      int value = abs(data.at("leftx")) > 8192 ? data.at("leftx") : 0;
      JoyStickAxis["leftx"] += value * scale;
    }
    if (data.count("lefty")) {
      int value = abs(data.at("lefty")) > 8192 ? data.at("lefty") : 0;
      JoyStickAxis["lefty"] -= value * scale;
    }
    if (data.count("rightx")) {
      int value = abs(data.at("rightx")) > 8192 ? data.at("rightx") : 0;
      JoyStickAxis["rightx"] += value * scale;
    }
    if (data.count("righty")) {
      int value = abs(data.at("righty")) > 8192 ? data.at("righty") : 0;
      JoyStickAxis["righty"] -= value * scale;
    }

    // 显示按钮状态
    if (data.count("leftshoulder"))
      left_shoulder_button_ = data.at("leftshoulder");
    if (data.count("rightshoulder"))
      right_shoulder_button_ = data.at("rightshoulder");
  }

  void print() {
#ifdef _WIN32
    system("cls");
#else
    std::cout << "\033[2J\033[H";
#endif
    std::cout << "===== 控制器状态 =====\n";
    std::cout << "左摇杆: (" << JoyStickAxis["leftx"] << ", "
              << JoyStickAxis["lefty"] << ")\n";
    std::cout << "右摇杆: (" << JoyStickAxis["rightx"] << ", "
              << JoyStickAxis["righty"] << ")\n";
    std::cout << "Left Shoulder: "
              << (left_shoulder_button_ ? "按下\n" : "释放\n");
    std::cout << "Right Shoulder: "
              << (right_shoulder_button_ ? "按下\n" : "释放\n");
    std::cout << "======================\n";
  }

private:
  std::unordered_map<std::string, double> JoyStickAxis{
      {"leftx", 0}, {"lefty", 0}, {"rightx", 0}, {"righty", 0}};
  bool left_shoulder_button_ = false;
  bool right_shoulder_button_ = false;
};

void ProcessGameData(JoyStickSDL &joystick, GamePad &gamepad,
                     std::atomic<bool> &running) {
  while (running) {
    auto status = joystick.getStatus();
    gamepad.update(status, 0.1 / 32768.0);
    gamepad.print();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

int main(int argc, char *argv[]) {
  JoyStickSDL joystick;
  GamePad gamepad;

  if (joystick.InitController() != 0) {
    std::cerr << "控制器初始化失败！" << std::endl;
    return -1;
  }
  while (1) {
    auto status = joystick.UpdateState();
    gamepad.update(status, 0.01 / 32768.0); // 转换为[-1, 1]范围
    gamepad.print();
    SDL_Delay(10);
  }
  return 0;
}