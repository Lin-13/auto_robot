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
