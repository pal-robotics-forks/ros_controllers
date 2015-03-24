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

#include <cmath>

#include <algorithm>

#include <diff_drive_controller/twist_limiter.h>

template<typename T>
T clamp(T x, T min, T max)
{
  return std::min(std::max(min, x), max);
}

namespace diff_drive_controller
{

  TwistLimiter::TwistLimiter(
    double max_left_wheel_joint_velocity,
    double max_right_wheel_joint_velocity,
    double left_wheel_radius,
    double right_wheel_radius,
    double wheel_separation
  )
  : max_left_wheel_joint_velocity(max_left_wheel_joint_velocity)
  , max_right_wheel_joint_velocity(max_right_wheel_joint_velocity)
  , left_wheel_radius(left_wheel_radius)
  , right_wheel_radius(right_wheel_radius)
  , wheel_separation(wheel_separation)
  {
  }

  double TwistLimiter::limit(double& v, double& w, double v0, double w0, double v1, double w1, double dt)
  {
    limitVelocity(v, w);

    const double dv = v - v0;
    const double dw = w - w0;

    const double tmp_v = v;
    const double tmp_w = w;

    linear.limitAcceleration(v, v0, dt);
    linear.limitJerk(v, v0, v1, dt);

    angular.limitAcceleration(w, w0, dt);
    angular.limitJerk(w, w0, w1, dt);

    const double kv = dv / (v - v0);
    const double kw = dw / (w - w0);

    const double k = std::min(kv, kw);

    v = tmp_v + k * dv;
    w = tmp_w + k * dw;

    return k; // @todo THIS ISN'T THE CORRECT k!!!
  }

  double TwistLimiter::limitVelocity(double& v, double& w)
  {
    const double vW = std::abs(v) + std::abs(w) * wheel_separation / 2.0;
    const double vL = vW / left_wheel_radius;
    const double vR = vW / right_wheel_radius;

    const double kL = max_left_wheel_joint_velocity  / vL;
    const double kR = max_right_wheel_joint_velocity / vR;
    const double kv = linear.limitVelocity(v);
    const double kw = angular.limitVelocity(w);

    const double k = std::min(kL,
                     std::min(kR,
                     std::min(kv,
                     std::min(kw, 1.0))));

    v *= k;
    w *= k;

    return k;
  }

} // namespace diff_drive_controller
