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
 * Author: Enrique Fernández
 */

#ifndef TWIST_LIMITER_H
#define TWIST_LIMITER_H

#include <diff_drive_controller/speed_limiter.h>

namespace diff_drive_controller
{

  class TwistLimiter
  {
  public:

    /**
     * \brief Constructor
     * \param [in] max_wheel_joint_velocity Maximum wheel joint velocity [rad/s], usually >= 0
     */
    TwistLimiter(
      double max_left_wheel_joint_velocity = 0.0,
      double max_right_wheel_joint_velocity = 0.0,
      double left_wheel_radius = 1.0,
      double right_wheel_radius = 1.0,
      double wheel_separation = 1.0
    );

    void setMaxLeftWheelJointVelocity(double x)
    {
      max_left_wheel_joint_velocity = x;
    }

    void setMaxRightWheelJointVelocity(double x)
    {
      max_right_wheel_joint_velocity = x;
    }

    void setLeftWheelRadius(double x)
    {
      left_wheel_radius = x;
    }

    void setRightWheelRadius(double x)
    {
      right_wheel_radius = x;
    }

    void setWheelSeparation(double x)
    {
      wheel_separation = x;
    }

    /**
     * \brief Limit the twist
     * \param [in, out] v Linear  velocity [  m/s]
     * \param [in, out] w Angular velocity [rad/s]
     * \param [in]      v0 Previous linear  velocity to v  [  m/s]
     * \param [in]      w0 Previous angular velocity to v  [rad/s]
     * \param [in]      v1 Previous linear  velocity to v0 [  m/s]
     * \param [in]      w1 Previous angular velocity to v0 [rad/s]
     * \param [in]      dt Time step [s]
     * \return Limiting factor (1.0 if none)
     */
    double limit(double& v, double& w, double v0, double w0, double v1, double w1, double dt);

    /**
     * \brief Limit the twist velocity
     * \param [in, out] v Linear  velocity [  m/s]
     * \param [in, out] w Angular velocity [rad/s]
     * \return Limiting factor (1.0 if none)
     */
    double limitVelocity(double& v, double &w);

  public:
    // Wheel joint velocity limit:
    double max_left_wheel_joint_velocity;
    double max_right_wheel_joint_velocity;

    // Wheel radius:
    double left_wheel_radius;
    double right_wheel_radius;

    // Wheel separation:
    double wheel_separation;

    // Linear/Angular speed limiters:
    SpeedLimiter linear;
    SpeedLimiter angular;
  };

} // namespace diff_drive_controller

#endif // TWIST_LIMITER_H
