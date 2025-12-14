// 搭配test_hybrid_control 的 dual_move函数使用
#include "joystick/joystick.h"
#include <context_monitor/monitor_client.h>
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
// 接受JoyStick 的按键key-value, 返回XYZRPY的增量值
class JoySticlControlPad {
public:
  std::unordered_map<std::string, double>
  update(const std::unordered_map<std::string, int> &data, double scale = 1) {
    if (!data.count("rightshoulder") || data.at("rightshoulder") == 0) {
      // 未按下右肩键，不进行控制
      return JoyStickAxis;
    }
    if (data.count("a") && data.at("a") == 1 && !speed_changed_last_time) {
      speed_scale_ = speed_scale_ / 2;
      speed_changed_last_time = true;
    }
    if (data.count("y") && data.at("y") == 1 && !speed_changed_last_time) {
      speed_scale_ = speed_scale_ * 2;
      speed_changed_last_time = true;
    }
    if ((data.count("a") == 0 || data.at("a") == 0) &&
        (data.count("y") == 0 || data.at("y") == 0)) {
      speed_changed_last_time = false;
    }
    if (data.count("b") && data.at("b") == 1 && !rot_changed_last_time) {
      rot_scale_ = rot_scale_ / 2;
      rot_changed_last_time = true;
    }
    if (data.count("x") && data.at("x") == 1 && !rot_changed_last_time) {
      rot_scale_ = rot_scale_ * 2;
      rot_changed_last_time = true;
    }
    if ((data.count("b") == 0 || data.at("b") == 0) &&
        (data.count("x") == 0 || data.at("x") == 0)) {
      rot_changed_last_time = false;
    }
    if (data.count("leftx")) {
      int value = abs(data.at("leftx")) > 8192 ? data.at("leftx") : 0;
      if (data.count("leftshoulder") && data.at("leftshoulder") == 1) {
        JoyStickAxis["rotx"] -= value * scale * rot_scale_;
      } else {
        JoyStickAxis["z"] -= value * scale * speed_scale_;
      }
    }
    if (data.count("lefty")) {
      int value = abs(data.at("lefty")) > 8192 ? data.at("lefty") : 0;
      if (data.count("leftshoulder") && data.at("leftshoulder") == 1) {
        JoyStickAxis["rotz"] -= value * scale * rot_scale_;
      } else {
        JoyStickAxis["x"] += value * scale * speed_scale_;
      }
      // ! 在optitrack坐标系中，y轴是垂直向上的，z向左
    }
    if (data.count("righty")) {
      int value = abs(data.at("righty")) > 8192 ? data.at("righty") : 0;
      if (data.count("leftshoulder") && data.at("leftshoulder") == 1) {
        JoyStickAxis["roty"] -= value * scale * rot_scale_;
      } else {
        JoyStickAxis["y"] -= value * scale * speed_scale_;
      }
      // ！ 在optitrack坐标系中，y轴是垂直向上的
    }
    return JoyStickAxis;
  }
  void print() {
#ifdef _WIN32
    system("cls");
#else
    std::cout << "\033[2J\033[H";
#endif
    std::cout << "===== XYZRPY =====\n";
    std::cout << std::setprecision(3) << "位置: (" << JoyStickAxis["x"] << ", "
              << JoyStickAxis["y"] << ", " << JoyStickAxis["z"] << ")[m]\n";
    std::cout << std::setprecision(3) << "姿态: ("
              << JoyStickAxis["rotx"] * 180 / 3.1415 << ", "
              << JoyStickAxis["roty"] * 180 / 3.1415 << ", "
              << JoyStickAxis["rotz"] * 180 / 3.1415 << ")[deg]\n";
    std::cout << "velocity scale: " << speed_scale_ << "[m]\n";
    std::cout << "rotation scale: " << rot_scale_ * 180 / 3.1415 << "[deg]\n";
    std::cout << "==================\n";
  }

private:
  std::unordered_map<std::string, double> JoyStickAxis{
      {"x", 0}, {"y", 0}, {"z", 0}, {"rotx", 0}, {"roty", 0}, {"rotz", 0}};
  bool left_shoulder_button_ = false;
  bool right_shoulder_button_ = false;
  double speed_scale_ = 0.001;
  double rot_scale_ = 1 * 3.1415 / 180;
  int speed_changed_last_time = false;
  int rot_changed_last_time = false;
};

class KeyboardControlPad {
public:
  std::unordered_map<std::string, double>
  update(const std::unordered_map<std::string, int> &data, double scale = 1) {
    if (data.count("w") || data.count("s")) {
      int value = 0;
      if (data.count("w")) {
        value += data.at("w") > 0 ? 1 : 0;
      }
      if (data.count("s")) {
        value -= data.at("s") > 0 ? 1 : 0;
      }
      JoyStickAxis["y"] += value * scale;
    }
    if (data.count("a") || data.count("d")) {
      int value = 0;
      if (data.count("a")) {
        value -= data.at("a") > 0 ? 1 : 0;
      }
      if (data.count("d")) {
        value += data.at("d") > 0 ? 1 : 0;
      }
      JoyStickAxis["x"] += value * scale;
    }
    if (data.count("r") || data.count("f")) {
      int value = 0;
      if (data.count("r")) {
        value += data.at("r") > 0 ? 1 : 0;
      }
      if (data.count("f")) {
        value -= data.at("f") > 0 ? 1 : 0;
      }
      JoyStickAxis["z"] += value * scale;
    }
    if (data.count("j") || data.count("l")) {
      int value = 0;
      if (data.count("j")) {
        value -= data.at("j") > 0 ? 1 : 0;
      }
      if (data.count("l")) {
        value += data.at("l") > 0 ? 1 : 0;
      }
      JoyStickAxis["roty"] += value * scale;
    }
    if (data.count("i") || data.count("k")) {
      int value = 0;
      if (data.count("i")) {
        value += data.at("i") > 0 ? 1 : 0;
      }
      if (data.count("k")) {
        value -= data.at("k") > 0 ? 1 : 0;
      }
      JoyStickAxis["rotx"] += value * scale;
    }
    if (data.count("p") || data.count(";")) {
      int value = 0;
      if (data.count("p")) {
        value += data.at("p") > 0 ? 1 : 0;
      }
      if (data.count(";")) {
        value -= data.at(";") > 0 ? 1 : 0;
      }
      JoyStickAxis["rotz"] += value * scale;
    }
    return JoyStickAxis;
  }

  void print() {
#ifdef _WIN32
    system("cls");
#else
    std::cout << "\033[2J\033[H";
#endif
    std::cout << "===== XYZRPY =====\n";
    std::cout << std::setprecision(3) << "位置: (" << JoyStickAxis["x"] << ", "
              << JoyStickAxis["y"] << ", " << JoyStickAxis["z"] << ")\n";
    std::cout << std::setprecision(3) << "姿态: (" << JoyStickAxis["rotx"]
              << ", " << JoyStickAxis["roty"] << ", " << JoyStickAxis["rotz"]
              << ")\n";
    std::cout << "==================\n";
  }

private:
  std::unordered_map<std::string, double> JoyStickAxis{
      {"x", 0}, {"y", 0}, {"z", 0}, {"rotx", 0}, {"roty", 0}, {"rotz", 0}};
  bool left_shoulder_button_ = false;
  bool right_shoulder_button_ = false;
};
class PoseClient {
public:
  PoseClient() {
    client_ = std::make_shared<MonitorClient>(grpc::CreateChannel(
        "localhost:50051", grpc::InsecureChannelCredentials()));
    client_thread_ = std::thread(&PoseClient::clientLoop, this);
  }
  void UpdatePose(const std::unordered_map<std::string, double> &pos) {
    position_ = pos;
  }
  void Stop() {
    running_ = false;
    if (client_thread_.joinable()) {
      client_thread_.join();
    }
  }
  ~PoseClient() { Stop(); }

private:
  void clientLoop() {
    // TODO：依然存在静态条件导致机器人行为异常
    // 需要在comtext_monitor中增加对变量的原子操作支持
    static std::vector<std::string> varNames = {"x",    "y",    "z",
                                                "rotx", "roty", "rotz"};
    while (running_) {
      std::string joint_moving, string_type = "string";
      bool found_joint_moving =
          client_->GetVariable("joint_moving", joint_moving, string_type);
      // joint_moving == 1 的时候可以写入位置
      if (found_joint_moving && std::stoi(joint_moving) == 1) {
        for (const auto &varName : varNames) {
          std::string value, type;
          bool found = client_->GetVariable(varName, value, type);
          if (position_.count(varName)) {
            value = std::to_string(position_[varName]);
            type = "d";
            if (found) {
              client_->SetVariable(varName, value);
            }
          }
        }
      }
      client_->SetVariable("move_start_command", "1");
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }
  std::unique_ptr<MonitorService::Stub> stub_;
  std::shared_ptr<MonitorClient> client_;
  std::atomic<bool> running_{true};
  std::thread client_thread_;
  std::unordered_map<std::string, double> position_;
};
void printPressedKeys(const std::unordered_map<std::string, int> &status) {
  static std::unordered_map<std::string, int> last_printed =
      status; // 记录上一次输出状态
  bool has_pressed = false;

  // 只打印状态变化的按键（避免刷屏）
  for (const auto &pair : status) {
    const std::string &key = pair.first;
    int value = pair.second;

    // 仅当按键按下 且 上一次未按下时打印
    if (value != 0 && last_printed[key] == 0) {
      std::cout << "Key " << key << " pressed.";
      has_pressed = true;
    }
    // 记录当前状态
    last_printed[key] = value;
  }

  // 清空行（可选）
  if (!has_pressed) {
    std::cout << "\r" << std::flush;
  }
}
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
  JoySticlControlPad gamepad;
  KeyboardSDL keyboard;
  KeyboardControlPad keyboardpad;
  PoseClient pose_client;
  // keyboard
  // std::cout << "初始化键盘监听..." << std::endl;
  // if (keyboard.InitKeyboard() != 0) {
  //   std::cerr << "键盘初始化失败！" << std::endl;
  //   return -1;
  // }
  // while (1) {
  //   auto status = keyboard.UpdateState();
  //   // printPressedKeys(status);
  //   auto pose = keyboardpad.update(status, 0.01 * 0.005);
  //   keyboardpad.print();
  //   pose_client.UpdatePose(pose); // Update pose with keyboard input if
  //   needed SDL_Delay(10);
  // }
  // joystick
  std::cout << "初始化控制器监听..." << std::endl;
  //   runMonitorClient();
  if (joystick.InitController() != 0) {
    std::cerr << "控制器初始化失败！" << std::endl;
    return -1;
  }
  while (1) {
    auto status = joystick.UpdateState();
    auto pose = gamepad.update(status, 0.01 / 32768.0); // 转换为[-1, 1]范围
    gamepad.print();
    pose_client.UpdatePose(pose); // Update pose with joystick input if needed
    SDL_Delay(10);
  }
  return 0;
}