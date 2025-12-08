#include "joystick/joystick.h"
#include <SDL2/SDL.h>
#include <iostream>

JoyStickSDL::JoyStickSDL() {
  SDL_SetMainReady();
  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER | SDL_INIT_JOYSTICK) !=
      0) {
    std::cerr << "SDL初始化失败: " << SDL_GetError() << std::endl;
    initialized_ = false;
  } else {
    if (SDL_GameControllerAddMappingsFromFile("gamecontrollerdb.txt") == -1) {
      std::cout << "未成功加载gamecontrollerdb" << std::endl;
    }
    initialized_ = true;
  }
}

int JoyStickSDL::InitController() {
  if (!initialized_)
    return -1;

  // 列出所有连接的设备
  int numJoysticks = SDL_NumJoysticks();
  std::cout << "检测到 " << numJoysticks << " 个输入设备:" << std::endl;

  for (int i = 0; i < numJoysticks; ++i) {
    std::cout << "设备 " << i << ": " << SDL_JoystickNameForIndex(i)
              << " (是游戏控制器: " << (SDL_IsGameController(i) ? "是" : "否")
              << ")" << std::endl;
  }

  // 尝试打开第一个游戏控制器
  if (numJoysticks > 0 && SDL_IsGameController(0)) {
    controller_ = SDL_GameControllerOpen(0);
    if (controller_) {
      std::cout << "\n已连接控制器: " << SDL_GameControllerName(controller_)
                << std::endl;
      SDL_Joystick *joy = SDL_GameControllerGetJoystick(controller_);
      if (joy) {
        std::cout << "控制器ID: " << SDL_JoystickInstanceID(joy) << std::endl;
      }

      return 0;
    } else {
      std::cerr << "无法打开控制器: " << SDL_GetError() << std::endl;
      return -1;
    }
  } else {
    std::cerr << "没有可用的游戏控制器" << std::endl;
    return -1;
  }
}
std::unordered_map<std::string, int> JoyStickSDL::UpdateState() {
  if (!controller_)
    return JoyStickMetaData;

  // 先处理所有事件
  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    if (event.type == SDL_CONTROLLERBUTTONDOWN) {
      std::string name = SDL_GameControllerGetStringForButton(
          static_cast<SDL_GameControllerButton>(event.cbutton.button));
      std::cout << "按钮按下: " << name << std::endl;
    } else if (event.type == SDL_CONTROLLERBUTTONUP) {
      std::string name = SDL_GameControllerGetStringForButton(
          static_cast<SDL_GameControllerButton>(event.cbutton.button));
      std::cout << "按钮释放: " << name << std::endl;
    }
  }

  std::unique_lock<std::mutex> lock(tex_);

  // 直接读取按钮状态
  JoyStickMetaData["a"] =
      SDL_GameControllerGetButton(controller_, SDL_CONTROLLER_BUTTON_A);
  JoyStickMetaData["b"] =
      SDL_GameControllerGetButton(controller_, SDL_CONTROLLER_BUTTON_B);
  JoyStickMetaData["x"] =
      SDL_GameControllerGetButton(controller_, SDL_CONTROLLER_BUTTON_X);
  JoyStickMetaData["y"] =
      SDL_GameControllerGetButton(controller_, SDL_CONTROLLER_BUTTON_Y);
  JoyStickMetaData["dpup"] =
      SDL_GameControllerGetButton(controller_, SDL_CONTROLLER_BUTTON_DPAD_UP);
  JoyStickMetaData["dpdown"] =
      SDL_GameControllerGetButton(controller_, SDL_CONTROLLER_BUTTON_DPAD_DOWN);
  JoyStickMetaData["dpleft"] =
      SDL_GameControllerGetButton(controller_, SDL_CONTROLLER_BUTTON_DPAD_LEFT);
  JoyStickMetaData["dpright"] = SDL_GameControllerGetButton(
      controller_, SDL_CONTROLLER_BUTTON_DPAD_RIGHT);
  JoyStickMetaData["leftshoulder"] = SDL_GameControllerGetButton(
      controller_, SDL_CONTROLLER_BUTTON_LEFTSHOULDER);
  JoyStickMetaData["rightshoulder"] = SDL_GameControllerGetButton(
      controller_, SDL_CONTROLLER_BUTTON_RIGHTSHOULDER);
  // 读取特殊按钮
  JoyStickMetaData["start"] =
      SDL_GameControllerGetButton(controller_, SDL_CONTROLLER_BUTTON_START);
  JoyStickMetaData["back"] =
      SDL_GameControllerGetButton(controller_, SDL_CONTROLLER_BUTTON_BACK);
  JoyStickMetaData["guide"] =
      SDL_GameControllerGetButton(controller_, SDL_CONTROLLER_BUTTON_GUIDE);

  // 读取摇杆轴
  JoyStickMetaData["leftx"] =
      SDL_GameControllerGetAxis(controller_, SDL_CONTROLLER_AXIS_LEFTX);
  JoyStickMetaData["lefty"] =
      SDL_GameControllerGetAxis(controller_, SDL_CONTROLLER_AXIS_LEFTY);
  JoyStickMetaData["rightx"] =
      SDL_GameControllerGetAxis(controller_, SDL_CONTROLLER_AXIS_RIGHTX);
  JoyStickMetaData["righty"] =
      SDL_GameControllerGetAxis(controller_, SDL_CONTROLLER_AXIS_RIGHTY);
  JoyStickMetaData["lefttrigger"] =
      SDL_GameControllerGetAxis(controller_, SDL_CONTROLLER_AXIS_TRIGGERLEFT);
  JoyStickMetaData["righttrigger"] =
      SDL_GameControllerGetAxis(controller_, SDL_CONTROLLER_AXIS_TRIGGERRIGHT);
  return JoyStickMetaData;
}

std::unordered_map<std::string, int> JoyStickSDL::getStatus() {
  std::unique_lock<std::mutex> lock(tex_);
  return JoyStickMetaData;
}

JoyStickSDL::~JoyStickSDL() {
  if (controller_) {
    SDL_GameControllerClose(controller_);
  }
  if (initialized_) {
    SDL_Quit();
  }
}
KeyboardSDL::KeyboardSDL() {
  SDL_SetMainReady();
  // 初始化SDL所有必要子系统（包含事件子系统）
  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS) != 0) {
    std::cerr << "SDL初始化失败: " << SDL_GetError() << std::endl;
    initialized_ = false;
  } else {
    initialized_ = true;
    keyboard_state_ = SDL_GetKeyboardState(nullptr);
    // 初始化上一帧状态
    last_status_ = KeyboardMetaData;
  }
}

int KeyboardSDL::InitKeyboard() {
  if (!initialized_)
    return -1;

  // 创建可见窗口（隐藏窗口无法获取焦点，先可见，后续可最小化）
  window_ = SDL_CreateWindow("KeyboardListenerWindow", SDL_WINDOWPOS_CENTERED,
                             SDL_WINDOWPOS_CENTERED, 320,
                             240,             // 小尺寸窗口，不影响操作
                             SDL_WINDOW_SHOWN // 必须可见才能获取焦点
  );

  if (!window_) {
    std::cerr << "无法创建键盘监听窗口: " << SDL_GetError() << std::endl;
    return -1;
  }

  // 创建渲染器（确保窗口上下文完整）
  renderer_ = SDL_CreateRenderer(
      window_, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
  if (!renderer_) {
    std::cerr << "无法创建渲染器: " << SDL_GetError() << std::endl;
    SDL_DestroyWindow(window_);
    window_ = nullptr;
    return -1;
  }

  // 主动获取窗口焦点
  SDL_RaiseWindow(window_);
  SDL_SetWindowInputFocus(window_);

  std::cout << "键盘监听初始化成功（窗口已获取焦点）" << std::endl;
  return 0;
}

std::unordered_map<std::string, int> KeyboardSDL::UpdateState() {
  if (!initialized_ || !window_ || !renderer_)
    return KeyboardMetaData;

  // 必须持续处理事件队列（核心！否则键盘事件无法捕获）
  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    // 处理窗口关闭事件（避免死循环）
    if (event.type == SDL_QUIT) {
      exit(0); // 或设置退出标记
    }
    // 处理键盘事件（辅助，核心靠扫描码）
    if (event.type == SDL_KEYDOWN || event.type == SDL_KEYUP) {
      // 仅用于调试，不影响状态更新
      std::string key_name = SDL_GetKeyName(event.key.keysym.sym);
      //   std::cout << (event.type == SDL_KEYDOWN ? "按下" : "松开") << ": "
      //             << key_name << std::endl;
    }
  }
  std::unique_lock<std::mutex> lock(tex_);
  keyboard_state_ = SDL_GetKeyboardState(nullptr);
  last_status_ = KeyboardMetaData;
  // std::unordered_map<std::string, int> KeyboardMetaData{
  //       {"a", 0}, {"s", 0}, {"d", 0}, {"w", 0}, {"r", 0}, {"f", 0},
  //       {"j", 0}, {"k", 0}, {"l", 0}, {"i", 0}, {"p", 0}, {";", 0}};
  KeyboardMetaData["a"] = keyboard_state_[SDL_SCANCODE_A] ? 1 : 0;
  KeyboardMetaData["s"] = keyboard_state_[SDL_SCANCODE_S] ? 1 : 0;
  KeyboardMetaData["d"] = keyboard_state_[SDL_SCANCODE_D] ? 1 : 0;
  KeyboardMetaData["w"] = keyboard_state_[SDL_SCANCODE_W] ? 1 : 0;
  KeyboardMetaData["r"] = keyboard_state_[SDL_SCANCODE_R] ? 1 : 0;
  KeyboardMetaData["f"] = keyboard_state_[SDL_SCANCODE_F] ? 1 : 0;
  KeyboardMetaData["j"] = keyboard_state_[SDL_SCANCODE_J] ? 1 : 0;
  KeyboardMetaData["k"] = keyboard_state_[SDL_SCANCODE_K] ? 1 : 0;
  KeyboardMetaData["l"] = keyboard_state_[SDL_SCANCODE_L] ? 1 : 0;
  KeyboardMetaData["i"] = keyboard_state_[SDL_SCANCODE_I] ? 1 : 0;
  KeyboardMetaData["p"] = keyboard_state_[SDL_SCANCODE_P] ? 1 : 0;
  KeyboardMetaData[";"] = keyboard_state_[SDL_SCANCODE_SEMICOLON] ? 1 : 0;
  return KeyboardMetaData;
}

std::unordered_map<std::string, int> KeyboardSDL::getStatus() {
  std::unique_lock<std::mutex> lock(tex_);
  return KeyboardMetaData;
}

KeyboardSDL::~KeyboardSDL() {
  // 释放渲染器和窗口
  if (renderer_) {
    SDL_DestroyRenderer(renderer_);
    renderer_ = nullptr;
  }
  if (window_) {
    SDL_DestroyWindow(window_);
    window_ = nullptr;
  }
  // 退出SDL子系统
  if (initialized_) {
    SDL_QuitSubSystem(SDL_INIT_VIDEO | SDL_INIT_EVENTS);
  }
}