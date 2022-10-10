#include "PID.hpp"

using namespace pid_controller;

PIDController::PIDController() {}

PIDController::~PIDController() {}

void PIDController::setGains(double _kp, double _ki, double _kd) {
  Kp_ = _kp;
  Ki_ = _ki;
  Kd_ = _kd;
}

void PIDController::setGainKp(double _kp) { Kp_ = _kp; }

void PIDController::setGainKi(double _ki) { Ki_ = _ki; }

void PIDController::setGainKd(double _kd) { Kd_ = _kd; }

void PIDController::setAntiWindup(double _anti_windup) { antiwindup_cte_ = _anti_windup; }

void PIDController::setAlpha(double _alpha) { alpha_ = _alpha; }

void PIDController::setResetIntegralSaturationFlag(bool _reset_integral_flag) {
  reset_integral_flag_ = _reset_integral_flag;
}

double PIDController::limitOutput(const double &output, const double &limit) {
  double limited_output = output;

  if (limit != 0.0f) {
    limited_output = (limited_output > limit) ? limit : limited_output;
    limited_output = (limited_output < -limit) ? -limit : limited_output;
  }
  return limited_output;
}

double PIDController::computeIntegral(const double &_dt, const double &_proportional_error) {
  // If sing of the error changes and the integrator is saturated, reset the integral for each
  // axis
  if (reset_integral_flag_ != 0) {
    if (std::abs(integral_accum_error_) > antiwindup_cte_) {
      if (std::signbit(integral_accum_error_) != std::signbit(_proportional_error)) {
        integral_accum_error_ = 0.0f;
      }
    }
  }

  // Update de acumulated error
  integral_accum_error_ += _proportional_error * _dt;

  // Compute anti-windup. Limit integral contribution
  integral_accum_error_ = limitOutput(integral_accum_error_, antiwindup_cte_);

  // Compute de integral contribution
  double i_position_error_contribution = Ki_ * integral_accum_error_;
  return i_position_error_contribution;
}

double PIDController::computeDerivative(const double &_dt, const double &_proportional_error) {
  // Compute the derivative contribution of the error filtered with a first
  // order filter
  double proportional_error_increment = (_proportional_error - last_proportional_error);

  filtered_derivate_error =
      alpha_ * proportional_error_increment + (1.0 - alpha_) * filtered_derivate_error;

  // Compute the derivate contribution
  double derivate_error_contribution = Kd_ * filtered_derivate_error / _dt;
  return derivate_error_contribution;
}

double PIDController::computeDerivative(const double &_dt,
                                          const double &_state_dot,
                                          const double &_reference_dot) {
  // Get the derivate error
  double derivate_error = _state_dot - _reference_dot;

  // Compute the derivate contribution
  double derivate_error_contribution = Kd_ * derivate_error / _dt;
  return derivate_error_contribution;
}

double PIDController::computeControl(const double &_dt,
                                       const double &_state,
                                       const double &_reference) {
  // Get the error
  double proportional_error = _reference - _state;

  // Initialize values for the integral and derivative contributions
  if (first_run_) {
    first_run_              = false;
    integral_accum_error_   = 0.0;
    last_proportional_error = proportional_error;
    filtered_derivate_error = 0.0;
  }

  // Compute the proportional contribution
  double p_error_contribution = Kp_ * proportional_error;

  // // Compute de integral contribution (position integrate)
  double integral_error_contribution = computeIntegral(_dt, proportional_error);

  // // Compute the derivate contribution
  double derivate_error_contribution = computeDerivative(_dt, proportional_error);

  // Compute output speed
  return proportional_error + integral_error_contribution + derivate_error_contribution;
}

double PIDController::computeControl(const double &_dt,
                                       const double &_state,
                                       const double &_reference,
                                       const double &_state_dot,
                                       const double &_reference_dot) {
  // Get the error
  double proportional_error = _reference - _state;

  // Initialize values for the integral and derivative contributions
  if (first_run_) {
    first_run_              = false;
    integral_accum_error_   = 0.0;
    last_proportional_error = proportional_error;
    filtered_derivate_error = 0.0;
  }

  // Compute the proportional contribution
  double p_error_contribution = Kp_ * proportional_error;

  // // Compute de integral contribution (position integrate)
  double integral_error_contribution = computeIntegral(_dt, proportional_error);

  // // Compute the derivate contribution
  double derivate_error_contribution = computeDerivative(_dt, _state_dot, _reference_dot);

  // Compute output speed
  return proportional_error + integral_error_contribution + derivate_error_contribution;
}

void PIDController::resetController() { first_run_ = true; }
