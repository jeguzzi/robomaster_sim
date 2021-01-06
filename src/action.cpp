#include "action.hpp"
#include "robot.hpp"
#include "command.hpp"

float time_to_goal(Pose2D & goal_pose, float linear_speed, float angular_speed) {
  return std::max(abs(normalize(goal_pose.theta)) / angular_speed, goal_pose.distance() / linear_speed);
}
