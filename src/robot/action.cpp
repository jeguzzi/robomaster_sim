#include "robot/robot.hpp"

void MoveAction::do_step(float time_step) {
  bool first = false;
  if (state == Action::State::started) {
    first = true;
    goal_odom = robot->chassis.get_pose() * goal;
    state = Action::State::running;
    spdlog::info("Start Move Action to {} [odom]", goal_odom);
  }
  if (state == Action::State::running) {
    Pose2D goal = goal_odom.relative_to(robot->chassis.get_pose());
    // Real robomaster is not normalizing!
    // goal.theta = normalize(goal.theta);
    spdlog::debug("Update Move Action to {} [frame] from {} [odom]", goal,
                  robot->chassis.get_pose());
    const float tau = 0.5f;
    Twist2D twist;
    if (goal.norm() < 0.01) {
      twist = {0, 0, 0};
      state = Action::State::succeed;
      remaining_duration = 0;
      spdlog::info("Move Action done", goal);
    } else {
      float f = std::min(1.0f, linear_speed / (goal.distance() / tau));
      twist = {f * goal.x / tau, f * goal.y / tau,
               std::clamp(goal.theta / tau, -angular_speed, angular_speed)};
      remaining_duration = time_to_goal(goal, linear_speed, angular_speed);
      if (first)
        predicted_duration = remaining_duration;
      spdlog::debug("Move Action continue [{:.2f} s / {:.2f} s]", remaining_duration,
                    predicted_duration);
    }
    // current = goal;
    robot->chassis.set_target_velocity(twist);
  }
}

// auto data = do_step(time_step);
// if(data.size()) {
//   spdlog::debug("Push action {} bytes: {:n}", data.size(), spdlog::to_hex(data));
//   commands->send(data);
//   if (state == Action::State::failed || state == Action::State::succeed) {
//     move_action = NULL;
//   }
// }

void MoveArmAction::do_step(float time_step) {
  bool first = false;
  if (state == Action::State::started) {
    if (absolute) {
    } else {
      goal_position = robot->arm.get_position() + goal_position;
    }
    state = Action::State::running;
    spdlog::info("[Move Arm Action] set goal to {}", goal_position);
    first = true;
  }
  if (state == Action::State::running) {
    robot->arm.update_control(goal_position);
    remaining_duration = robot->arm.distance();
    if (first)
      predicted_duration = remaining_duration;
    if (remaining_duration < 0.005) {
      state = Action::State::succeed;
      remaining_duration = 0;
      spdlog::info("[Move Arm Action] done");
    } else {
      spdlog::debug("[Move Arm Action] will continue for [{:.2f} rad / {:.2f} rad]",
                    remaining_duration, predicted_duration);
    }
  }
}

void PlaySoundAction::do_step(float time_step) {
  remaining_duration -= time_step;
  state = Action::State::running;
  if (remaining_duration < 0) {
    if (play_times == 0) {
      state = Action::State::succeed;
      spdlog::info("Finished playing sounds {}", sound_id);
    } else {
      spdlog::info("Start playing sound {} (#{} left)", sound_id, play_times);
      remaining_duration = predicted_duration;
      play_times--;
    }
  }
}

void MoveServoAction::do_step(float time_step) {
  // spdlog::info("[Move Servo Action] update {}", servo_id);
  Servo *servo = &robot->servos[servo_id];
  if (state == Action::State::started) {
    state = Action::State::running;
    spdlog::info("[Move Servo Action] set goal to {}", target_angle);
    servo->angle.desired = target_angle;
    predicted_duration = servo->distance();
  }
  if (state == Action::State::running) {
    remaining_duration = servo->distance();
    if (remaining_duration < 0.005 || time_left < 0) {
      state = Action::State::succeed;
      remaining_duration = 0;
      spdlog::info("[Move Servo Action] done {} ~= {}", servo->angle.current, servo->angle.desired);
    } else {
      spdlog::debug("[Move Servo Action] will continue for [{:.2f} rad / {:.2f} rad]",
                    remaining_duration, predicted_duration);
    }
  }
  current_angle = servo->angle.current;
  time_left -= time_step;
}

void MoveGimbalAction::do_step(float time_step) {
  Gimbal &gimbal = robot->gimbal;
  current = gimbal.attitude(Gimbal::Frame::fixed);
  if (state == Action::State::started) {
    state = Action::State::running;
    // gimbal.set_mode(Gimbal::Mode::attitude_mode);
    gimbal.performing_action = true;
    target = gimbal.fixed_angles(target, frame);
    gimbal.reset_target_speeds();
    gimbal.set_target_angles(target);
    gimbal.set_control_speeds(speed);
    spdlog::info("[Move Gimbal Action] set goal to {} [fixed frame]", target);
    predicted_duration = gimbal.distance();
  }
  if (state == Action::State::running) {
    remaining_duration = gimbal.distance();
    if (remaining_duration < 0.02) {
      state = Action::State::succeed;
      remaining_duration = 0;
      gimbal.reset_target_attitude();
      gimbal.performing_action = false;
      spdlog::info("[Move Gimbal Action] done {} ~= {}", current, gimbal.target_attitude);
    } else {
      spdlog::debug("[Move Gimbal Action] will continue for [{:.2f} rad / {:.2f} rad], {} != {}",
                    remaining_duration, predicted_duration, current, gimbal.target_attitude);
    }
  }
}
