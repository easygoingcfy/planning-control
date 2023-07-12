#include "vehicle_control/pid_controller.h"

#include <assert.h>

namespace shenlan {
namespace control {
PIDController::PIDController(const double kp, const double ki,
                             const double kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  previous_error_ = 0.0;
  previous_output_ = 0.0;
  integral_ = 0.0;
  first_hit_ = true;
}

// /**to-do**/ 实现PID控制
double PIDController::Control(const double error, const double dt) {
  if (dt <= 0) {
    return previous_output_;
  }
  double output = 0;
  double diff = 0;
  if (first_hit_) {
    first_hit_ = false;
  } else {
    diff = error - previous_error_;
  }
  integral_ += ki_ * diff * dt;
  output = kp_ * error + integral_ + diff / dt * kd_;
  previous_error_ += error;
  previous_output_ = output;
  return output;
}

// /**to-do**/ 重置PID参数
void PIDController::Reset() {
  previous_error_ = 0.0;
  previous_output_ = 0.0;
  integral_ = 0.0;
  first_hit_ = true;
}

}  // namespace control
}  // namespace shenlan