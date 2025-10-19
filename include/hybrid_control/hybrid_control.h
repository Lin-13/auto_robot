#ifndef HYBRID_CONTROL_H_
#define HYBRID_CONTROL_H_
#include "robot_interface/robot.h"
#include "utils/matrix_utils.h"
#include "utils/transform_tree.h"
#include <chrono>
#include <fmt/format.h>
/**
 * @brief 双机器人力位混合控制
 *
 */
class HybridController {
public:
  HybridController();
  ~HybridController();

private:
  // 需要注入的依赖
  std::shared_ptr<TransformTree> transform_tree_;
  std::shared_ptr<Robot> robot_left_;
  std::shared_ptr<Robot> robot_right_;
};
#endif // HYBRID_CONTROL_H_