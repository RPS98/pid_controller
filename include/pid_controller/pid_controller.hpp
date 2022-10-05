#ifndef PID_CONTROLLER_HPP_
#define PID_CONTROLLER_HPP_

#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

namespace controller {

using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;

class PIDController {
public:
  PIDController();
  ~PIDController();

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

class PIDController1D {
public:
  PIDController1D();
  ~PIDController1D();

  void setGains(double &_kp, double &_ki, double &_kd);
  void setAntiWindup(double &_anti_windup);
  void setAlpha(double &_alpha);
  void setResetIntegralSaturationFlag(bool &_reset_integral_flag);

  double computeControl(const double &_dt, const double &_state, const double &_reference);
  double computeControl(const double &_dt,
                        const double &_state,
                        const double &_reference,
                        const double &_state_dot,
                        const double &_reference_dot);

  void resetController();

  static double limitOutput(const double &output, const double &limit);

private:
  double computeIntegral(const double &_dt, const double &_proportional_error);
  double computeDerivative(const double &_dt, const double &_proportional_error);
  double computeDerivative(const double &_dt,
                           const double &_state_dot,
                           const double &_reference_dot);

private:
  // PID gains
  double Kp_ = 0.0;
  double Ki_ = 0.0;
  double Kd_ = 0.0;

  // PID params
  double antiwindup_cte_  = 0.0;
  double alpha_           = 0.0;
  bool reset_integral_flag_ = false;

  // PID state
  bool first_run_                  = true;
  double integral_accum_error_   = 0.0;
  double last_proportional_error = 0.0;
  double filtered_derivate_error = 0.0;
};
}  // namespace controller

#endif  // PID_CONTROLLER_HPP_
