#ifndef MONITOR_CLIENT_H
#define MONITOR_CLIENT_H

#include "context_monitor/proto/monitor.grpc.pb.h"

#include <grpcpp/grpcpp.h>
#include <iostream>
#include <string>
using context_monitor::EmptyRequest;
using context_monitor::MonitorService;
using context_monitor::StatusResponse;
using context_monitor::VariableRequest;
using context_monitor::VariableResponse;
using context_monitor::VariablesList;
using context_monitor::VariableUpdate;
using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;

// 客户端类
class MonitorClient {
private:
  std::unique_ptr<MonitorService::Stub> stub_;

public:
  MonitorClient(std::shared_ptr<Channel> channel)
      : stub_(MonitorService::NewStub(channel)) {}

  // 获取变量值
  bool GetVariable(const std::string &name, std::string &value,
                   std::string &type) {
    VariableRequest request;
    request.set_name(name);

    VariableResponse response;
    ClientContext context;

    Status status = stub_->GetVariable(&context, request, &response);

    if (status.ok() && response.found()) {
      value = response.value();
      type = response.type();
      return true;
    } else {
      return false;
    }
  }

  // 设置变量值
  bool SetVariable(const std::string &name, const std::string &value) {
    VariableUpdate request;
    request.set_name(name);
    request.set_value(value);

    StatusResponse response;
    ClientContext context;

    Status status = stub_->SetVariable(&context, request, &response);

    return status.ok() && response.success();
  }

  // 列出所有变量
  std::vector<std::string> ListVariables() {
    EmptyRequest request;
    VariablesList response;
    ClientContext context;

    Status status = stub_->ListVariables(&context, request, &response);

    std::vector<std::string> variables;
    if (status.ok()) {
      for (const auto &name : response.names()) {
        variables.push_back(name);
      }
    }

    return variables;
  }
};
void runMonitorClient();
#endif // MONITOR_CLIENT_H
