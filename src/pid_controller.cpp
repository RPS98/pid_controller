#include "pid_controller.hpp"

using namespace controller;

PIDController::PIDController() {}

PIDController::~PIDController() {}

void PIDController::setGains(Vector3d &_kp, Vector3d &_ki, Vector3d &_kd) {
  Kp_lin_mat_ = _kp.asDiagonal();
  Ki_lin_mat_ = _ki.asDiagonal();
  Kd_lin_mat_ = _kd.asDiagonal();
}

void PIDController::setAntiWindup(Vector3d &_anti_windup) { antiwindup_cte_ = _anti_windup; }

void PIDController::setAlpha(Vector3d &_alpha) { alpha_ = _alpha; }

void PIDController::setResetIntegralSaturationFlag(bool &_reset_integral_flag) {
  reset_integral_flag_ = _reset_integral_flag;
}

Vector3d PIDController::limitOutput(const Vector3d &output,
                                    const Vector3d &limits,
                                    const bool &proportional_limitation) {
  Vector3d limited_output = output;

  // Delimit the speed for each axis
  if (proportional_limitation) {
    for (short j = 0; j < 3; j++) {
      if (limits[j] == 0.0f || output[j] == 0.0) {
        continue;
      };

      if (limited_output[j] > limits[j] || limited_output[j] < -limits[j]) {
        limited_output *= std::abs(limits[j] / limited_output[j]);
      }
    }
  } else {
    for (short j = 0; j < 3; j++) {
      if (limits[j] == 0.0f) {
        continue;
      }
      limited_output[j] = (limited_output[j] > limits[j]) ? limits[j] : limited_output[j];
      limited_output[j] = (limited_output[j] < -limits[j]) ? -limits[j] : limited_output[j];
    }
  }
  return limited_output;
}

Vector3d PIDController::computeIntegral(const double &_dt, const Vector3d &_proportional_error) {
  // If sing of the error changes and the integrator is saturated, reset the integral for each
  // axis
  if (reset_integral_flag_ != 0) {
    for (short j = 0; j < 3; j++) {
      if (std::abs(integral_accum_error_[j]) > antiwindup_cte_[j]) {
        if (std::signbit(integral_accum_error_[j]) != std::signbit(_proportional_error[j])) {
          integral_accum_error_[j] = 0.0f;
        }
      }
    }
  }

  // Update de acumulated error
  integral_accum_error_ += _proportional_error * _dt;

  // Compute anti-windup. Limit integral contribution
  integral_accum_error_ = limitOutput(integral_accum_error_, antiwindup_cte_, false);

  // Compute de integral contribution
  Vector3d i_position_error_contribution = Ki_lin_mat_ * integral_accum_error_;
  return i_position_error_contribution;
}

Vector3d PIDController::computeDerivative(const double &_dt, const Vector3d &_proportional_error) {
  // Compute the derivative contribution of the error filtered with a first
  // order filter
  Vector3d proportional_error_increment = (_proportional_error - last_proportional_error);

  filtered_derivate_error = alpha_.cwiseProduct(proportional_error_increment) +
                            (Vector3d::Identity() - alpha_).cwiseProduct(filtered_derivate_error);

  // Compute the derivate contribution
  Vector3d derivate_error_contribution = Kd_lin_mat_ * filtered_derivate_error / _dt;
  return derivate_error_contribution;
}

Vector3d PIDController::computeDerivative(const double &_dt,
                                          const Vector3d &_state_dot,
                                          const Vector3d &_reference_dot) {
  // Get the derivate error
  Vector3d derivate_error = _state_dot - _reference_dot;

  // Compute the derivate contribution
  Vector3d derivate_error_contribution = Kd_lin_mat_ * derivate_error / _dt;
  return derivate_error_contribution;
}

Vector3d PIDController::computeControl(const double &_dt,
                                       const Vector3d &_state,
                                       const Vector3d &_reference) {
  // Get the error
  Vector3d proportional_error = _reference - _state;

  // Initialize values for the integral and derivative contributions
  if (first_run_) {
    first_run_              = false;
    integral_accum_error_   = Vector3d::Zero();
    last_proportional_error = proportional_error;
    filtered_derivate_error = Vector3d::Zero();
  }

  // Compute the proportional contribution
  Vector3d p_error_contribution = Kp_lin_mat_ * proportional_error;

  // // Compute de integral contribution (position integrate)
  Vector3d integral_error_contribution = computeIntegral(_dt, proportional_error);

  // // Compute the derivate contribution
  Vector3d derivate_error_contribution = computeDerivative(_dt, proportional_error);

  // Compute output speed
  return proportional_error + integral_error_contribution + derivate_error_contribution;
}

Vector3d PIDController::computeControl(const double &_dt,
                                       const Vector3d &_state,
                                       const Vector3d &_reference,
                                       const Vector3d &_state_dot,
                                       const Vector3d &_reference_dot) {
  // Get the error
  Vector3d proportional_error = _reference - _state;

  // Initialize values for the integral and derivative contributions
  if (first_run_) {
    first_run_              = false;
    integral_accum_error_   = Vector3d::Zero();
    last_proportional_error = proportional_error;
    filtered_derivate_error = Vector3d::Zero();
  }

  // Compute the proportional contribution
  Vector3d p_error_contribution = Kp_lin_mat_ * proportional_error;

  // // Compute de integral contribution (position integrate)
  Vector3d integral_error_contribution = computeIntegral(_dt, proportional_error);

  // // Compute the derivate contribution
  Vector3d derivate_error_contribution = computeDerivative(_dt, _state_dot, _reference_dot);

  // Compute output speed
  return proportional_error + integral_error_contribution + derivate_error_contribution;
}

void PIDController::resetController() { first_run_ = true; }