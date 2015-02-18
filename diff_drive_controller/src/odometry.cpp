/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the PAL Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Luca Marchionni
 * Author: Bence Magyar
 * Author: Enrique Fernández
 * Author: Paul Mathieu
 * Author: Jeremie Deray
 */

#include <diff_drive_controller/odometry.h>

#include <cmath>

#include <boost/bind.hpp>

namespace diff_drive_controller
{
  namespace bacc = boost::accumulators;

  Odometry::Odometry(size_t velocity_rolling_window_size)
  : timestamp_(0.0)
  , x_(0.0)
  , y_(0.0)
  , error_constant_right_(0.001)
  , error_constant_left_(0.001)
  , pose_cov_() //this init the whole array to 0
  , update_pose_cov_(false)
  , heading_(0.0)
  , linear_(0.0)
  , angular_(0.0)
  , wheel_separation_(0.0)
  , left_wheel_radius_(0.0)
  , right_wheel_radius_(0.0)
  , left_wheel_old_pos_(0.0)
  , right_wheel_old_pos_(0.0)
  , velocity_rolling_window_size_(velocity_rolling_window_size)
  , linear_acc_(RollingWindow::window_size = velocity_rolling_window_size)
  , angular_acc_(RollingWindow::window_size = velocity_rolling_window_size)
  , integrate_fun_(boost::bind(&Odometry::integrateExact, this, _1, _2, _3, _4))
  {
  }

  void Odometry::init(const ros::Time& time)
  {
    // Reset accumulators and timestamp:
    resetAccumulators();
    timestamp_ = time;
  }

  bool Odometry::update(double left_pos, double right_pos, const ros::Time &time)
  {
    /// We cannot estimate the speed with very small time intervals:
    const double dt = (time - timestamp_).toSec();
    if (dt < 1e-9)
      return false; // Interval too small to integrate with

    timestamp_ = time;

    /// Get current wheel joint positions:
    const double left_wheel_cur_pos  = left_pos  * left_wheel_radius_;
    const double right_wheel_cur_pos = right_pos * right_wheel_radius_;

    /// Estimate velocity of wheels using old and current position:
    const double left_wheel_est_vel  = left_wheel_cur_pos  - left_wheel_old_pos_;
    const double right_wheel_est_vel = right_wheel_cur_pos - right_wheel_old_pos_;

    /// Update old position with current:
    left_wheel_old_pos_  = left_wheel_cur_pos;
    right_wheel_old_pos_ = right_wheel_cur_pos;

    /// Compute linear and angular diff:
    const double linear  = (right_wheel_est_vel + left_wheel_est_vel) * 0.5 ;
    const double angular = (right_wheel_est_vel - left_wheel_est_vel) / wheel_separation_;

    /// Update odometry state:
    updateState(linear, angular, right_wheel_est_vel, left_wheel_est_vel);

    /// Estimate speeds using a rolling mean to filter them out:
    linear_acc_(linear/dt);
    angular_acc_(angular/dt);

    linear_ = bacc::rolling_mean(linear_acc_);
    angular_ = bacc::rolling_mean(angular_acc_);

    return true;
  }

  void Odometry::updateOpenLoop(double linear, double angular, const ros::Time &time)
  {
    /// Save last linear and angular velocity:
    linear_ = linear;
    angular_ = angular;

    /// Update odometry state:
    const double vr = (linear - angular * wheel_separation_ / 2.0) / wheel_radius_;
    const double vl = (linear + angular * wheel_separation_ / 2.0) / wheel_radius_;
    const double dt = (time - timestamp_).toSec();
    timestamp_ = time;
    updateState(linear * dt, angular * dt, vr, vl);
  }

  void Odometry::updateState(double linear, double angular, double vr, double vl)
  {
    if (update_pose_cov_)
    {
      /// Integrate odometry state and compute jacobian:
      StateJacobian jacobian_state;
      MotionJacobian jacobian_motion;
      integrate_fun_(linear, angular, &jacobian_state, &jacobian_motion);

      /// Update motion increment covariance:
      Eigen::Matrix2d S; S << error_constant_right_ * std::abs(vr), 0.0,
                              0.0, error_constant_left_ * std::abs(vl);

      pose_cov_ = jacobian_state  * pose_cov_ * jacobian_state.transpose() +
                  jacobian_motion *     S     * jacobian_motion.transpose();
    }
    else
    {
      /// Integrate odometry state:
      integrate_fun_(linear, angular, NULL, NULL);
    }
  }

  void Odometry::setWheelParams(double wheel_separation, double left_wheel_radius, double right_wheel_radius)
  {
    wheel_separation_   = wheel_separation;
    left_wheel_radius_  = left_wheel_radius;
    right_wheel_radius_ = right_wheel_radius;
  }

  void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
  {
    velocity_rolling_window_size_ = velocity_rolling_window_size;

    resetAccumulators();
  }

  void Odometry::integrateRungeKutta2(double linear, double angular, StateJacobian* jacobian_state, MotionJacobian* jacobian_motion)
  {
    const double direction = heading_ + angular * 0.5;

    const double cos_direction = cos(direction);
    const double sin_direction = sin(direction);

    /// Runge-Kutta 2nd order integration:
    x_       += linear * cos_direction;
    y_       += linear * sin_direction;
    heading_ += angular;

    /// Jacobians:
    if (jacobian_state)
    {
      *jacobian_state << 1.0, 0.0, -linear * sin_direction,
                         0.0, 1.0,  linear * cos_direction,
                         0.0, 0.0, 1.0;
    }

    if (jacobian_motion)
    {
      const double b_inv = 1.0 / wheel_separation_;

      const double linear_b_inv = linear * b_inv;

      const double linear_b_inv_sin = linear_b_inv * sin_direction;
      const double linear_b_inv_cos = linear_b_inv * cos_direction;

      *jacobian_motion << 0.5 * (cos_direction - linear_b_inv_sin), 0.5 * (cos_direction + linear_b_inv_sin),
                          0.5 * (sin_direction + linear_b_inv_cos), 0.5 * (sin_direction - linear_b_inv_cos),
                                                             b_inv, -b_inv;
    }
  }

  void Odometry::integrateExact(double linear, double angular, StateJacobian* jacobian_state, MotionJacobian* jacobian_motion)
  {
    if (fabs(angular) < 10e-3)
    {
      integrateRungeKutta2(linear, angular, jacobian_state, jacobian_motion);
    }
    else
    {
      /// Exact integration (should solve problems when angular is zero):
      const double heading_old = heading_;
      const double r = linear/angular;

      heading_ += angular;

      const double sin_heading     = sin(heading_);
      const double cos_heading     = cos(heading_);
      const double sin_heading_old = sin(heading_old);
      const double cos_heading_old = cos(heading_old);

      const double s_heading = sin_heading - sin_heading_old;
      const double c_heading = cos_heading - cos_heading_old;

      x_ +=  r * s_heading;
      y_ += -r * c_heading;

      // Jacobians:
      if (jacobian_state)
      {
        *jacobian_state << 1.0, 0.0, -r * c_heading,
                           0.0, 1.0,  r * s_heading,
                           0.0, 0.0, 1.0;
      }

      if (jacobian_motion)
      {
        const double b_inv = 1.0 / wheel_separation_;

        const double linear_b_inv = linear * b_inv;
        const double r_b_inv      = r * b_inv;

        const double angular_2_inv = 1.0 / (angular * angular);

        const double dr_vl = (0.5 * angular + linear_b_inv) * angular_2_inv;
        const double dr_vr = (0.5 * angular - linear_b_inv) * angular_2_inv;

        const double r_b_inv_cos = r_b_inv * cos_heading;
        const double r_b_inv_sin = r_b_inv * sin_heading;

        *jacobian_motion <<  dr_vr * s_heading + r_b_inv_cos,  dr_vl * s_heading - r_b_inv_cos,
                            -dr_vr * c_heading + r_b_inv_sin, -dr_vl * c_heading - r_b_inv_sin,
                                                       b_inv, -b_inv;
      }
    }
  }

  void Odometry::resetAccumulators()
  {
    linear_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
    angular_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
  }

} // namespace diff_drive_controller
