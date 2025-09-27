#include "robot_interface/robot.h"

class AuboClient {
public:
  AuboClient();
  ~AuboClient();
};
RobotTopology::Ptr auboRobotTopology();
std::unique_ptr<Robot>
auboRobotLeft(std::chrono::milliseconds timer_period = 33ms,
              ControllerConfig config = {.k_p = 1});
std::unique_ptr<Robot>
auboRobotRight(std::chrono::milliseconds timer_period = 33ms,
               ControllerConfig config = {.k_p = 1});
