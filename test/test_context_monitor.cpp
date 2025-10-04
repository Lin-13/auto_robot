#include "context_monitor/monitor_client.h"
#include "context_monitor/monitor_server.h"
#include <Eigen/Core>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
// 主线程中的变量
int g_counter = 0;
float g_temperature = 25.5f;
double g_pressure = 1013.25;
std::string g_status = "running";

// 模拟主程序工作
void mainProgramLoop() {
  std::cout << "Main program started. Variables will update every second."
            << std::endl;

  while (true) {
    // 模拟变量变化
    g_counter++;
    g_temperature += 0.1f;
    g_pressure -= 0.05;

    // 每5秒打印一次当前状态
    if (g_counter % 5 == 0) {
      std::cout << "\nMain program status:" << std::endl;
      std::cout << "counter: " << g_counter << std::endl;
      std::cout << "temperature: " << g_temperature << std::endl;
      std::cout << "pressure: " << g_pressure << std::endl;
      std::cout << "status: " << g_status << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

// 客户端交互函数
void runClient() {
  MonitorClient client(grpc::CreateChannel("localhost:50051",
                                           grpc::InsecureChannelCredentials()));

  std::string command;
  std::cout << "\nDebug client commands:" << std::endl;
  std::cout << "1. list - List all variables" << std::endl;
  std::cout << "2. get <name> - Get variable value" << std::endl;
  std::cout << "3. set <name> <value> - Set variable value" << std::endl;
  std::cout << "4. exit - Exit client" << std::endl;
  while (true) {
    std::cout << "Enter command: ";
    std::getline(std::cin, command);

    if (command == "exit") {
      break;
    } else if (command == "list") {
      auto vars = client.ListVariables();
      std::cout << "Available variables: " << std::endl;
      for (const auto &var : vars) {
        std::cout << " - " << var << std::endl;
      }
    } else if (command.substr(0, 3) == "get") {
      std::string var_name = command.substr(4);
      std::string value, type;
      if (client.GetVariable(var_name, value, type)) {
        std::cout << var_name << " (" << type << "): " << value << std::endl;
      } else {
        std::cout << "Variable " << var_name << " not found" << std::endl;
      }
    } else if (command.substr(0, 3) == "set") {
      size_t first_space = command.find(' ', 4);
      if (first_space == std::string::npos) {
        std::cout << "Invalid set command. Usage: set <name> <value>"
                  << std::endl;
        continue;
      }

      std::string var_name = command.substr(4, first_space - 4);
      std::string value = command.substr(first_space + 1);

      if (client.SetVariable(var_name, value)) {
        std::cout << "Variable " << var_name << " set to " << value
                  << std::endl;
      } else {
        std::cout << "Failed to set variable " << var_name << std::endl;
      }
    } else {
      std::cout << "Unknown command" << std::endl;
    }
  }
}

int main(int argc, char *argv[]) {
  // 注册变量到反射系统
  REGISTER_MONITOR_VARIABLE(g_counter);
  REGISTER_MONITOR_VARIABLE(g_temperature);
  REGISTER_MONITOR_VARIABLE(g_pressure);
  REGISTER_MONITOR_VARIABLE(g_status);

  if (argc > 1 && std::string(argv[1]) == "client") {
    // 运行客户端
    runClient();
  } else {
    // 启动gRPC服务器线程
    std::thread server_thread(RunMonitorServer, 50051);
    static int static_counter = 0;
    REGISTER_MONITOR_VARIABLE(static_counter);
    int local_counter = 0;
    REGISTER_MONITOR_VARIABLE(local_counter);
    Eigen::Vector3d local_position(1.0, 2.0, 3.0);
    REGISTER_MONITOR_VARIABLE(local_position);
    std::shared_ptr<double> shared_counter = std::make_shared<double>(0);
    REGISTER_MONITOR_VARIABLE(shared_counter);
    Eigen::Matrix4d local_transform = Eigen::Matrix4d::Identity();
    REGISTER_MONITOR_VARIABLE(local_transform);
    // 运行主程序
    mainProgramLoop();

    // 等待服务器线程结束
    server_thread.join();
  }

  return 0;
}
