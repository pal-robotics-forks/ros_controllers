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

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include <ros/time.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/function.hpp>
#include <Eigen/Core>

namespace diff_drive_controller
{
  namespace bacc = boost::accumulators;

  /**
   * \brief The Odometry class handles odometry readings
   * (2D pose and velocity with related timestamp)
   */
  class Odometry
  {
  public:

    /// Jacobian types:
    typedef Eigen::Matrix3d             StateJacobian;
    typedef Eigen::Matrix<double, 3, 2> MotionJacobian;

    /// Covariance type:
    typedef StateJacobian Covariance;

    /// Integration function, used to integrate the odometry:
    typedef boost::function<void(double, double, StateJacobian*, MotionJacobian*)> IntegrationFunction;

    /**
     * \brief Constructor
     * Timestamp will get the current time value
     * Value will be set to zero
     * \param velocity_rolling_window_size Rolling window size used to compute the velocity mean
     */
    Odometry(size_t velocity_rolling_window_size = 10);

    /**
     * \brief Initialize the odometry
     * \param left_pos  Left  wheel position [rad]
     * \param right_pos Right wheel position [rad]
     * \param time Current time
     */
    void init(double left_pos, double right_pos, const ros::Time &time);

    /**
     * \brief Updates the odometry class with latest wheels position
     * \param left_pos  Left  wheel position [rad]
     * \param right_pos Right wheel position [rad]
     * \param time      Current time
     * \return true if the odometry is actually updated
     */
    bool update(double left_pos, double right_pos, const ros::Time &time);

    /**
     * \brief Updates the odometry class with latest velocity command
     * \param linear  Linear velocity [m/s]
     * \param angular Angular velocity [rad/s]
     * \param time    Current time
     */
    void updateOpenLoop(double linear, double angular, const ros::Time &time);

    /**
     * \brief Updates the odometry state
     * \param linear  Linear velocity [m/s]
     * \param angular Angular velocity [rad/s]
     * \param vr Right wheel speed (increment)
     * \param vl Left wheel speed (increment)
     */
    void updateState(double linear, double angular, double vr, double vl);

    /**
     * \brief heading getter
     * \return heading [rad]
     */
    double getHeading() const
    {
      return heading_;
    }

    /**
     * \brief x position getter
     * \return x position [m]
     */
    double getX() const
    {
      return x_;
    }

    /**
     * \brief y position getter
     * \return y position [m]
     */
    double getY() const
    {
      return y_;
    }

    /**
     * \brief linear velocity getter
     * \return linear velocity [m/s]
     */
    double getLinear() const
    {
      return linear_;
    }

    /**
     * \brief angular velocity getter
     * \return angular velocity [rad/s]
     */
    double getAngular() const
    {
      return angular_;
    }

    /**
     * \brief pose covariance entry getter
     * \return pose covariance
     */
    Covariance getPoseCovariance() const
    {
      return pose_cov_;
    }

    /**
     * \brief Sets pose covariance entry
     * \param cov covariance
     */
    void setPoseCovariance(const Covariance& cov)
    {
      pose_cov_ = cov;
    }

    /**
     * \brief Enable/disable pose covariance matrix update
     * \param enable true enable, false disable
     */
    void enablePoseCovUpdate(bool enable)
    {
      update_pose_cov_ = enable;
    }

    /**
     * \brief Sets error constant of the right wheel
     * \param val value of the error constant
     */
    void setErrorCstRight(double val)
    {
      error_constant_right_ = val;
    }

    /**
     * \brief Sets error constant of the left wheel
     * \param val value of the error constant
     */
    void setErrorCstLeft(double val)
    {
      error_constant_left_ = val;
    }

    /**
     * \brief Sets the wheel parameters: radius and separation
     * \param wheel_separation Seperation between left and right wheels [m]
     * \param wheel_radius     Left wheel radius [m]
     * \param wheel_radius     Right wheel radius [m]
     */
    void setWheelParams(double wheel_separation, double left_wheel_radius, double right_wheel_radius);

    /**
     * \brief Velocity rolling window size setter
     * \param velocity_rolling_window_size Velocity rolling window size
     */
    void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

  private:

    /// Rolling mean accumulator and window:
    typedef bacc::accumulator_set<double, bacc::stats<bacc::tag::rolling_mean> > RollingMeanAcc;
    typedef bacc::tag::rolling_window RollingWindow;

    /**
     * \brief Integrates the velocities (linear and angular) using 2nd order Runge-Kutta
     * \param linear  Linear  velocity   [m] (linear  displacement, i.e. m/s * dt) computed by encoders
     * \param angular Angular velocity [rad] (angular displacement, i.e. m/s * dt) computed by encoders
     * \param jacobian_state  Jacobian wrt state [x, y, theta]
     * \param jacobian_motion Jacobian wrt wheel traveled distances (motion) [v_r, v_l]
     */
    void integrateRungeKutta2(double linear, double angular, StateJacobian* jacobian_state, MotionJacobian* jacobian_motion);

    /**
     * \brief Integrates the velocities (linear and angular) using exact method
     * \param linear  Linear  velocity   [m] (linear  displacement, i.e. m/s * dt) computed by encoders
     * \param angular Angular velocity [rad] (angular displacement, i.e. m/s * dt) computed by encoders
     * \param jacobian_state  Jacobian wrt state [x, y, theta]
     * \param jacobian_motion Jacobian wrt wheel traveled distances (motion) [v_r, v_l]
     */
    void integrateExact(double linear, double angular, StateJacobian* jacobian_state, MotionJacobian* jacobian_motion);

    /**
     *  \brief Reset linear and angular accumulators
     */
    void resetAccumulators();

    /// Current timestamp:
    ros::Time timestamp_;

    /// Current pose:
    double x_;        //   [m]
    double y_;        //   [m]
    double heading_;  // [rad]

    /// Error constants kr and kl depend on the robot
    /// and the environment and should be experimentally
    /// established by performing and analyzing representative movements
    double error_constant_right_;
    double error_constant_left_;

    /** Position covariance
    *   |  xx  xy  xth |
    *   |  yx  yy  yth |
    *   | thx thy thth |
    */
    Covariance pose_cov_;

    /// Enable Covariance matrix update
    bool update_pose_cov_;

    /// Current velocity:
    double linear_;  //   [m/s]
    double angular_; // [rad/s]

    /// Wheel kinematic parameters [m]:
    double wheel_separation_;
    double left_wheel_radius_;
    double right_wheel_radius_;

    /// Previou wheel position/state [rad]:
    double left_wheel_old_pos_;
    double right_wheel_old_pos_;

    /// Rolling mean accumulators for the linar and angular velocities:
    size_t velocity_rolling_window_size_;
    RollingMeanAcc linear_acc_;
    RollingMeanAcc angular_acc_;

    /// Integration funcion, used to integrate the odometry:
    IntegrationFunction integrate_fun_;
  };
}

#endif /* ODOMETRY_H_ */
