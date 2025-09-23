#include "robot_interface/robot.h"

class AuboClient {
public:
  AuboClient();
  ~AuboClient();
};
RobotTopology::Ptr auboRobotTopology();
Robot auboRobotLeft(std::chrono::milliseconds timer_period = 33ms,
                    ControllerConfig config = {k_p : 10});
Robot auboRobotRight(std::chrono::milliseconds timer_period = 33ms,
                     ControllerConfig config = {k_p : 10});
