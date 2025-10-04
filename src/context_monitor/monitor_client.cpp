#include "context_monitor/monitor_client.h"
// 运行客户端
void runMonitorClient() {
  MonitorClient client(grpc::CreateChannel("localhost:50051",
                                           grpc::InsecureChannelCredentials()));

  std::string command;
  while (true) {
    std::cout << "\nEnter command (get <var>/set <var> <value>/list/exit): ";
    std::getline(std::cin, command);

    if (command == "exit") {
      break;
    } else if (command == "list") {
      std::vector<std::string> vars = client.ListVariables();
      std::cout << "Available variables: ";
      for (const std::string &var : vars) {
        std::cout << var << " ";
      }
      std::cout << std::endl;
    } else if (command.substr(0, 4) == "get ") {
      std::string varName = command.substr(4);
      std::string value, type;
      bool found = client.GetVariable(varName, value, type);
      if (found) {
        std::cout << varName << " (" << type << "): " << value << std::endl;
      } else {
        std::cout << "Variable " << varName << " not found." << std::endl;
      }
    } else if (command.substr(0, 4) == "set ") {
      std::string rest = command.substr(4);
      size_t spacePos = rest.find(' ');
      if (spacePos == std::string::npos) {
        std::cout << "Invalid set command. Usage: set <var> <value>"
                  << std::endl;
        continue;
      }
      std::string varName = rest.substr(0, spacePos);
      std::string value = rest.substr(spacePos + 1);
      int ret = client.SetVariable(varName, value);
      std::cout << "Set variable " << varName << " to " << value
                << " ret: " << ret << std::endl;
    } else {
      std::cout << "Unknown command. Available commands: get, set, list, exit"
                << std::endl;
    }
  }
}
