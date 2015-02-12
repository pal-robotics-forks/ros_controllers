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
 */

#include <diff_drive_controller/odometry.h>

#include <math.h>

#include <boost/bind.hpp>

namespace diff_drive_controller
{
  namespace bacc = boost::accumulators;

  Odometry::Odometry(size_t velocity_rolling_window_size)
  : timestamp_(0.0)
  , x_(0.0)
  , y_(0.0)
  , kr_(0.001)
  , kl_(0.001)
  , heading_(0.0)
  , linear_(0.0)
  , angular_(0.0)
  , wheel_separation_(0.0)
  , wheel_radius_(0.0)
  , left_wheel_old_pos_(0.0)
  , right_wheel_old_pos_(0.0)
  , velocity_rolling_window_size_(velocity_rolling_window_size)
  , linear_acc_(RollingWindow::window_size = velocity_rolling_window_size)
  , angular_acc_(RollingWindow::window_size = velocity_rolling_window_size)
  , integrate_fun_(boost::bind(&Odometry::integrateExact, this, _1, _2))
  {
    memset(pose_cov_, 0, 9);
  }

  void Odometry::init(const ros::Time& time)
  {
    // Reset accumulators and timestamp:
    resetAccumulators();
    timestamp_ = time;
  }

  bool Odometry::update(double left_pos, double right_pos, const ros::Time &time)
  {
    /// Get current wheel joint positions:
    const double left_wheel_cur_pos  = left_pos  * wheel_radius_;
    const double right_wheel_cur_pos = right_pos * wheel_radius_;

    /// Estimate velocity of wheels using old and current position:
    const double left_wheel_est_vel  = left_wheel_cur_pos  - left_wheel_old_pos_;
    const double right_wheel_est_vel = right_wheel_cur_pos - right_wheel_old_pos_;

    /// Update old position with current:
    left_wheel_old_pos_  = left_wheel_cur_pos;
    right_wheel_old_pos_ = right_wheel_cur_pos;

    /// Compute linear and angular diff:
    const double linear  = (right_wheel_est_vel + left_wheel_est_vel) * 0.5 ;
    const double angular = (right_wheel_est_vel - left_wheel_est_vel) / wheel_separation_;

    /// Integrate odometry:
    integrate_fun_(linear, angular);

    /// We cannot estimate the speed with very small time intervals:
    const double dt = (time - timestamp_).toSec();
    if (dt < 0.0001)
      return false; // Interval too small to integrate with

    /// Update motion increment cov
    const double drr = std::abs(right_wheel_est_vel) * kr_;
    const double dll = std::abs(left_wheel_est_vel) * kr_;

    const double sha = sin(heading_ + angular/2);
    const double cha = cos(heading_ + angular/2);

    /// Jacobian position
    const double jp_2 = -linear * sha;
    const double jp_5 =  linear * cha;

    /// Jacobian motion
    const double jrl_0 = 0.5 * cha - (linear / (2*wheel_separation_)) * sha;
    const double jrl_1 = 0.5 * cha + (linear / (2*wheel_separation_)) * sha;
    const double jrl_2 = 0.5 * sha + (linear / (2*wheel_separation_)) * cha;
    const double jrl_3 = 0.5 * sha - (linear / (2*wheel_separation_)) * cha;
    const double jrl_4 = 1 / wheel_separation_;
    const double jrl_5 = 1 / wheel_separation_;

    /// Update position covariance
    //xx
    pose_cov_[0] += jp_2*(pose_cov_[6] + pose_cov_[8]*jp_2) + pose_cov_[2]*jp_2
                    + dll*jrl_1*jrl_1 + drr*jrl_0*jrl_0;
    //xy
    pose_cov_[1] += pose_cov_[2]*jp_5 + dll*jrl_1*jrl_3 + drr*jrl_0*jrl_2
                    +jp_2*(pose_cov_[5] + pose_cov_[8]*jp_5);
    //xth
    pose_cov_[2] += pose_cov_[8]*jp_2 + dll*jrl_1*jrl_5 + drr*jrl_0*jrl_4;

    //yx
    pose_cov_[3] = pose_cov_[1];
    //yy
    pose_cov_[4] += jp_5*(pose_cov_[5]+pose_cov_[8]*jp_5) + pose_cov_[5]*jp_5
                    + dll*jrl_3*jrl_3 + drr*jrl_2*jrl_2;
    //yth
    pose_cov_[5] += pose_cov_[8]*jp_5 + dll*jrl_3*jrl_5 + drr*jrl_2*jrl_4;

    //thx
    pose_cov_[6] = pose_cov_[2];
    //thy
    pose_cov_[7] = pose_cov_[5];
    //thth
    pose_cov_[8] += dll*jrl_5*jrl_5 + drr*jrl_4*jrl_4;

    timestamp_ = time;

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

    /// Integrate odometry:
    const double dt = (time - timestamp_).toSec();
    timestamp_ = time;
    integrate_fun_(linear * dt, angular * dt);
  }

  void Odometry::setWheelParams(double wheel_separation, double wheel_radius)
  {
    wheel_separation_ = wheel_separation;
    wheel_radius_     = wheel_radius;
  }

  void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
  {
    velocity_rolling_window_size_ = velocity_rolling_window_size;

    resetAccumulators();
  }

  void Odometry::integrateRungeKutta2(double linear, double angular)
  {
    const double direction = heading_ + angular * 0.5;

    /// Runge-Kutta 2nd order integration:
    x_       += linear * cos(direction);
    y_       += linear * sin(direction);
    heading_ += angular;
  }

  /**
   * \brief Other possible integration method provided by the class
   * \param linear
   * \param angular
   */
  void Odometry::integrateExact(double linear, double angular)
  {
    if (fabs(angular) < 10e-3)
      integrateRungeKutta2(linear, angular);
    else
    {
      /// Exact integration (should solve problems when angular is zero):
      const double heading_old = heading_;
      const double r = linear/angular;
      heading_ += angular;
      x_       +=  r * (sin(heading_) - sin(heading_old));
      y_       += -r * (cos(heading_) - cos(heading_old));
    }
  }

  void Odometry::resetAccumulators()
  {
    linear_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
    angular_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
  }

} // namespace diff_drive_controller
