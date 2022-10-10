#ifndef PID_CONTROLLER_HPP_
#define PID_CONTROLLER_HPP_

#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

namespace pid_controller {

using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;

class PIDController {
public:
  PIDController();
  ~PIDController();

  void setGains(double _kp, double _ki, double _kd);
  void setGainKp(double _kp);
  void setGainKi(double _ki);
  void setGainKd(double _kd);
  void setAntiWindup(double _anti_windup);
  void setAlpha(double _alpha);
  void setResetIntegralSaturationFlag(bool _reset_integral_flag);

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
