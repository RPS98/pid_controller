#ifndef PID_CONTROLLER_3D_HPP_
#define PID_CONTROLLER_3D_HPP_

#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

namespace pid_controller {

using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;

class PIDController3D {
public:
  PIDController3D();
  ~PIDController3D();

  void setGains(Vector3d &_kp, Vector3d &_ki, Vector3d &_kd);
  void setAntiWindup(Vector3d &_anti_windup);
  void setAlpha(Vector3d &_alpha);
  void setResetIntegralSaturationFlag(bool &_reset_integral_flag);

  Vector3d computeControl(const double &_dt, const Vector3d &_state, const Vector3d &_reference);
  Vector3d computeControl(const double &_dt,
                          const Vector3d &_state,
                          const Vector3d &_reference,
                          const Vector3d &_state_dot,
                          const Vector3d &_reference_dot);

  void resetController();

  static Vector3d limitOutput(const Vector3d &output,
                              const Vector3d &limits,
                              const bool &proportional_limitation);

private:
  Vector3d computeIntegral(const double &_dt, const Vector3d &_proportional_error);
  Vector3d computeDerivative(const double &_dt, const Vector3d &_proportional_error);
  Vector3d computeDerivative(const double &_dt,
                             const Vector3d &_state_dot,
                             const Vector3d &_reference_dot);

private:
  // PID gains
  Matrix3d Kp_lin_mat_ = Matrix3d::Identity();
  Matrix3d Ki_lin_mat_ = Matrix3d::Identity();
  Matrix3d Kd_lin_mat_ = Matrix3d::Identity();

  // PID params
  Vector3d antiwindup_cte_  = Vector3d::Zero();
  Vector3d alpha_           = Vector3d::Zero();
  bool reset_integral_flag_ = false;

  // PID state
  bool first_run_                  = true;
  Vector3d integral_accum_error_   = Vector3d::Zero();
  Vector3d last_proportional_error = Vector3d::Zero();
  Vector3d filtered_derivate_error = Vector3d::Zero();
};
}  // namespace controller

#endif  // PID_CONTROLLER_3D_HPP_
