#ifndef MONITOR_SERVER_IMPL_H
#define MONITOR_SERVER_IMPL_H

#include "context_monitor/proto/monitor.grpc.pb.h"
#include "context_monitor/reflection.h"
#include <grpcpp/grpcpp.h>

using context_monitor::EmptyRequest;
using context_monitor::MonitorService;
using context_monitor::StatusResponse;
using context_monitor::VariableRequest;
using context_monitor::VariableResponse;
using context_monitor::VariablesList;
using context_monitor::VariableUpdate;
using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;

class MonitorServiceImpl final : public MonitorService::Service {
public:
  Status GetVariable(ServerContext *context, const VariableRequest *request,
                     VariableResponse *response) override {
    std::string value;
    std::string type;

    bool found = ReflectionSystem::getInstance().getVariable(request->name(),
                                                             value, type);

    response->set_name(request->name());
    response->set_type(type);
    response->set_value(value);
    response->set_found(found);

    return Status::OK;
  }

  Status SetVariable(ServerContext *context, const VariableUpdate *request,
                     StatusResponse *response) override {
    bool success = ReflectionSystem::getInstance().setVariable(
        request->name(), request->value());

    response->set_success(success);
    if (success) {
      response->set_message("Variable updated successfully");
    } else {
      response->set_message(
          "Failed to update variable (not found or invalid value)");
    }

    return Status::OK;
  }

  Status ListVariables(ServerContext *context, const EmptyRequest *request,
                       VariablesList *response) override {
    std::vector<std::string> variables =
        ReflectionSystem::getInstance().listVariables();

    for (const auto &var : variables) {
      response->add_names(var);
    }

    return Status::OK;
  }
};

// 启动gRPC服务器的函数
void RunMonitorServer(int port = 50051);

#endif // MONITOR_SERVER_IMPL_H
