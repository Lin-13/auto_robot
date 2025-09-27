#include <AuboRobotMetaType.h>
#include <algorithm>
#include <array>
#include <fmt/format.h>
#include <robot_interface/robot_controller.h>
#include <robot_interface/robot_topology.h>
#include <serviceinterface.h>
#include <string.h>
namespace aubo = aubo_robot_namespace;
class AuboController : public RobotController {
public:
  using Ptr = std::shared_ptr<AuboController>;
  AuboController(const char *host_name, int port, const char *user_name,
                 const char *password, std::string name = "aubo_controller",
                 ControllerConfig config = {.k_p = 10});
  int Initialize(std::chrono::milliseconds timer_period = 33ms) override;
  virtual RobotJointState getJointState() override;
  virtual int setTarget(RobotJointState target_joint_state) override;
  int Logout();
  virtual ~AuboController();
  ServiceInterface getAuboServiceInterface() { return robot_interface_; }

  int enable_log_ = 0;

private:
  int timer_cb() override;
  ServiceInterface robot_interface_;
  virtual int setJointState(RobotJointState joint_state) override;
};