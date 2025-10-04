#include "context_monitor/monitor_server.h"
#include <iostream>
#include <thread>
/**
 * @brief 运行gRPC 监视服务器
 *
 * @param port 服务器监听的端口号
 */
void RunMonitorServer(int port) {
  std::string server_address = "0.0.0.0:" + std::to_string(port);
  MonitorServiceImpl service;

  ServerBuilder builder;
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  builder.RegisterService(&service);

  std::unique_ptr<Server> server(builder.BuildAndStart());
  std::cout << "Debug server listening on " << server_address << std::endl;

  // 运行服务器直到被中断
  server->Wait();
}
