/*!*******************************************************************************************
 *  \file       pid_1.hpp
 *  \brief      PID 1D Controller definition
 *  \authors    Rafael Pérez Seguí
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#ifndef PID_CONTROLLER_1D_HPP
#define PID_CONTROLLER_1D_HPP

#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

namespace pid_1d_controller {

template <typename T = double>
struct PIDParams {
  // PID gains
  T Kp_gains = 0.0;  // Proportional gains
  T Ki_gains = 0.0;  // Integral gains
  T Kd_gains = 0.0;  // Derivative gains

  // PID params
  T antiwindup_cte         = 0.0;    // Integral anti-windup
  T alpha                  = 0.0;    // Derivative filter
  bool reset_integral_flag = false;  // Reset integral flag when error sign changes

  // PID Output saturation
  T upper_output_saturation = 0.0;  // Upper output saturation
  T lower_output_saturation = 0.0;  // Lower output saturation
};

/**
 * @brief PID Controller
 *
 * PID 1 dimension controller. It can be used to control any 1D system.
 *
 * @tparam T Precision
 */
template <typename T = double>
class PID {
public:
  /**
   * @brief Construct a new pid
   *
   * @param verbose Verbosity flag. Default: false
   */
  PID(const PIDParams<T> &pid_params, const bool &verbose = false) : verbose_(verbose) {
    update_params(pid_params);
    reset_controller();
  };

  ~PID(){};

private:
  bool verbose_ = false;  // Verbosity flag

  // PID gains
  T Kp_ = 0.0;
  T Ki_ = 0.0;
  T Kd_ = 0.0;

  // PID params
  T antiwindup_cte_         = 0.0;    // Integral anti-windup
  T alpha_                  = 0.0;    // Derivative filter
  bool reset_integral_flag_ = false;  // Reset integral flag when error sign changes

  // PID Output saturation
  bool saturation_flag_      = false;  // Output saturation flag
  T upper_output_saturation_ = 0.0;
  T lower_output_saturation_ = 0.0;

  // PID state
  bool first_run_            = true;  // First run flag
  T integral_accum_error_    = 0.0;   // Integral accumulator error
  T last_proportional_error_ = 0.0;   // Last proportional error
  T filtered_derivate_error_ = 0.0;   // Filtered derivative error

public:
  // Public methods

  /**
   * @brief Update the PID controller with pid params
   *
   * @param params PIDParams struct
   */
  void update_params(const PIDParams<T> &params) {
    set_gains(params.Kp_gains, params.Ki_gains, params.Kd_gains);
    set_anti_windup(params.antiwindup_cte);
    set_alpha(params.alpha);
    set_reset_integral_saturation_flag(params.reset_integral_flag);

    if ((params.upper_output_saturation != 0.0) || (params.lower_output_saturation != 0.0)) {
      set_output_saturation(params.upper_output_saturation, params.lower_output_saturation);
    } else {
      disable_output_saturation();
    }
  }

  /**
   * @brief Reset the controller
   *
   * Reset the integral error and the saturation
   */
  inline void reset_controller() { first_run_ = true; }

  /**
   * @brief Set the output saturation
   *
   * @param upper_saturation Upper saturation
   * @param lower_saturation Lower saturation
   */
  void set_output_saturation(const T upper_saturation, const T lower_saturation) {
    assert(upper_saturation > lower_saturation);
    upper_output_saturation_ = upper_saturation;
    lower_output_saturation_ = lower_saturation;
    saturation_flag_         = true;
  }

  /**
   * @brief Disable the output saturation
   *
   * Disable the output saturation. The output is not limited by the saturation
   * limits. To enable the output saturation, use the set_output_saturation
   * method.
   *
   * @param saturation_flag Saturation flag
   */
  inline void disable_output_saturation() { saturation_flag_ = false; }

  /**
   * @brief Get the proportional error
   *
   * @param state Current state
   * @param reference Reference state
   * @return T Error
   */
  static inline T get_error(const T state, const T reference) {
    // Compute proportional error
    return reference - state;
  }

  /**
   * @brief Get the proportional and derivative error
   *
   * @param state State
   * @param reference Reference
   * @param state_dot State derivative
   * @param reference_dot Reference derivative
   * @param proportional_error Output proportional error
   * @param derivative_error Output derivative error
   */
  static inline void get_error(const T state,
                               const T reference,
                               const T state_dot,
                               const T reference_dot,
                               T &proportional_error,
                               T &derivative_error) {
    // Compute proportional error
    proportional_error = reference - state;

    // Compute the derivate error
    derivative_error = reference_dot - state_dot;
  }

  /**
   * @brief Process the PID controller
   *
   * @param dt Time step
   * @param proportional_error Proportional error
   * @return PID output
   */
  T compute_control(const T dt, const T proportional_error) {
    // Initialize values for the integral and derivative contributions
    if (first_run_) {
      first_run_               = false;
      integral_accum_error_    = 0.0;
      last_proportional_error_ = proportional_error;
      filtered_derivate_error_ = 0.0;
    }

    // Compute the proportional contribution
    T proportional_error_contribution = Kp_ * proportional_error;

    // Compute de integral contribution (position integrate)
    T integral_error_contribution = compute_integral_contribution(dt, proportional_error);

    // Compute the derivate contribution
    T derivate_error_contribution =
        compute_derivative_contribution_by_deriving(dt, proportional_error);

    // Compute output speed
    T output =
        proportional_error_contribution + integral_error_contribution + derivate_error_contribution;

    if (saturation_flag_) {
      output = saturate_output(output, upper_output_saturation_, lower_output_saturation_);
    }
    return output;
  }

  /**
   * @brief Process the PID controller
   *
   * @param dt = Time step
   * @param proportional_error = Proportional error
   * @param derivative_error = Derivative error
   * @return PID output
   */
  T computeControl(const T dt, const T proportional_error, const T derivative_error) {
    // Initialize values for the integral and derivative contributions
    if (first_run_) {
      first_run_               = false;
      integral_accum_error_    = 0.0;
      last_proportional_error_ = proportional_error;
      filtered_derivate_error_ = 0.0;
    }

    // Compute the proportional contribution
    T proportional_error_contribution = Kp_ * proportional_error;

    // // Compute de integral contribution (position integrate)
    T integral_error_contribution = compute_integral_contribution(dt, proportional_error);

    // // Compute the derivate contribution
    T derivate_error_contribution =
        compute_derivative_contribution(dt, proportional_error, derivative_error);

    // Compute output speed
    T output =
        proportional_error_contribution + integral_error_contribution + derivate_error_contribution;

    if (saturation_flag_) {
      output = saturate_output(output, upper_output_saturation_, lower_output_saturation_);
    }
    return output;
  }

  /**
   * @brief Saturation function
   *
   * If the output is greater than the upper limit, the output is saturated to
   * the upper limit. If the output is lower than the lower limit, the output is
   * saturated to the lower limit.
   *
   * @param output Value to saturate
   * @param upper_limits Upper limits vector
   * @param lower_limits Lower limits vector
   * @return Saturated value
   */
  static inline T saturate_output(const T output, const T upper_limits, const T lower_limits) {
    // With std::clamp
    return std::clamp(output, lower_limits, upper_limits);
  }

  // Getters and setters

  /**
   * @brief Get the params
   *
   * @return PIDParams<T> PID parameters
   */
  PIDParams<T> get_params() const {
    PIDParams<T> params;
    get_gains(params.Kp_gains, params.Ki_gains, params.Kd_gains);
    params.antiwindup_cte      = get_anti_windup();
    params.alpha               = get_alpha();
    params.reset_integral_flag = get_reset_integral_saturation_flag();
    get_saturation_limits(params.upper_output_saturation, params.lower_output_saturation);
    return params;
  }

  /**
   * @brief Set the Gains of the controller
   *
   * @param kp Proportional gain
   * @param ki Integral gain
   * @param kd Derivative gain
   */
  inline void set_gains(T kp, T ki, T kd) {
    Kp_ = kp;
    Ki_ = ki;
    Kd_ = kd;
  }

  /**
   * @brief Get the gains
   *
   * @param kp Proportional gain
   * @param ki Integral gain
   * @param kd Derivative gain
   */
  inline void get_gains(T &kp, T &ki, T &kd) const {
    kp = Kp_;
    ki = Ki_;
    kd = Kd_;
  };

  /**
   * @brief Set the Proportional Gain of the controller
   *
   * @param kp Proportional gain
   */
  inline void set_kp(T kp) { Kp_ = kp; }

  /**
   * @brief Get the Proportional Gain of the controller
   *
   * @return T Proportional gain
   */
  inline T get_kp() const { return Kp_; }

  /**
   * @brief Set the Integral Gain of the controller
   *
   * @param ki Integral gain
   */
  inline void set_ki(T ki) { Ki_ = ki; }

  /**
   * @brief Get the Integral Gain of the controller
   *
   * @return T Integral gain
   */
  inline T get_ki() const { return Ki_; }

  /**
   * @brief Set the Derivative Gain of the controller
   *
   * @param kd Derivative gain
   */
  inline void set_kd(T kd) { Kd_ = kd; }

  /**
   * @brief Get the Derivative Gain of the controller
   *
   * @return T Derivative gain
   */
  inline T get_kd() const { return Kd_; }

  /**
   * @brief Set the Anti Windup of the controller
   *
   * @param anti_windup Anti windup
   */
  inline void set_anti_windup(T anti_windup) { antiwindup_cte_ = anti_windup; }

  /**
   * @brief Get the Anti Windup of the controller
   *
   * @return T Anti windup
   */
  inline T get_anti_windup() const { return antiwindup_cte_; }

  /**
   * @brief Set the Alpha of the controller
   *
   * @param alpha Alpha
   */
  inline void set_alpha(T alpha) { alpha_ = alpha; }

  /**
   * @brief Get the Alpha of the controller
   *
   * @return T Alpha
   */
  inline T get_alpha() const { return alpha_; }

  /**
   * @brief Set the Reset Integral Saturation Flag of the controller
   *
   * If the flag is true, the integral contribution is reset to zero when the
   * integral error is grater than the anti windup and the sign of the integral
   * error is different from the sign of the proportional error.
   *
   * @param reset_integral_flag Reset integral saturation flag
   */
  inline void set_reset_integral_saturation_flag(bool reset_integral_flag) {
    reset_integral_flag_ = reset_integral_flag;
  }

  /**
   * @brief Get the Reset Integral Saturation Flag of the controller
   *
   * If the flag is true, the integral contribution is reset to zero when the
   * integral error is grater than the anti windup and the sign of the integral
   * error is different from the sign of the proportional error.
   *
   * @return true Reset integral saturation flag is enabled
   * @return false Reset integral saturation flag is disabled
   */
  inline bool get_reset_integral_saturation_flag() const { return reset_integral_flag_; }

  /**
   * @brief Get the Saturation Limits of the controller
   *
   * @param upper_limit Upper limit
   * @param lower_limit Lower limit
   */
  inline void get_saturation_limits(T &upper_limit, T &lower_limit) const {
    upper_limit = upper_output_saturation_;
    lower_limit = lower_output_saturation_;
  }

  /**
   * @brief Get the output saturation flag
   *
   * @return true Saturation is enabled
   * @return false Saturation is disabled
   */
  inline bool get_output_saturation_flag() const { return saturation_flag_; };

protected:
  /**
   * @brief Compute the integral contribution of the controller
   *
   * @param dt Delta time (s)
   * @param proportional_error Proportional error
   * @return T Integral contribution
   */
  T compute_integral_contribution(const T dt, const T proportional_error) {
    // If sing of the error changes and the integrator is saturated, reset the
    // integral for each axis
    if (reset_integral_flag_ != 0) {
      if (std::abs(integral_accum_error_) > antiwindup_cte_) {
        if (std::signbit(integral_accum_error_) != std::signbit(proportional_error)) {
          integral_accum_error_ = 0.0;
        }
      }
    }

    // Update de acumulated error
    integral_accum_error_ += proportional_error * dt;

    // Compute anti-windup. Limit integral contribution
    if (antiwindup_cte_ != 0.0) {
      integral_accum_error_ =
          saturate_output(integral_accum_error_, antiwindup_cte_, -1.0 * antiwindup_cte_);
    }

    // Compute de integral contribution
    T integral_error_contribution = Ki_ * integral_accum_error_;
    return integral_error_contribution;
  }

  /**
   * @brief Compute the derivative contribution of the controller
   *
   * @param dt Delta time (s)
   * @param proportional_error Proportional error
   * @return Derivative contribution
   */
  T compute_derivative_contribution_by_deriving(const T dt, const T proportional_error) {
    // Compute the derivative contribution of the error filtered with a first
    // order filter
    T proportional_error_increment = (proportional_error - last_proportional_error_);

    filtered_derivate_error_ =
        alpha_ * proportional_error_increment + (1.0 - alpha_) * filtered_derivate_error_;

    // Compute the derivate contribution
    T derivate_error_contribution = Kd_ * filtered_derivate_error_ / dt;
    return derivate_error_contribution;
  }

  /**
   * @brief Compute the derivative contribution of the controller
   *
   * For controllers with derivative feedback, the derivative contribution is
   * computed using the state and reference derivatives.
   *
   * @param dt Delta time (s)
   * @param state_dot State derivative
   * @param reference_dot Reference derivative
   * @return Derivative contribution
   */
  T compute_derivative_contribution(const T dt, const T derivate_error) {
    // Compute the derivate contribution
    T derivate_error_contribution = Kd_ * derivate_error / dt;
    return derivate_error_contribution;
  }
};
}  // namespace pid_1d_controller

#endif  // PID_CONTROLLER_1D_HPP
