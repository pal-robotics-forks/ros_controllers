///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
// Copyright (c) 2008, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
// * Neither the name of PAL Robotics S.L. nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

// Pluginlib
#include <pluginlib/class_list_macros.h>

// Project
#include <trajectory_interface/quintic_spline_segment.h>
#include <pal_joint_trajectory_controller/joint_trajectory_controller.h>

namespace pal
{
namespace position_controllers
{
/**
 * \brief Joint trajectory controller that represents trajectory segments as <b>quintic
 * splines</b> and sends
 * commands to a \b position interface.
 */
typedef pal::joint_trajectory_controller::JointTrajectoryPositionController<trajectory_interface::QuinticSplineSegment<double> >
    JointSplineController;
}

namespace velocity_controllers
{
/**
 * \brief Joint trajectory controller that represents trajectory segments as <b>quintic
 * splines</b> and sends
 * commands to a \b velocity interface.
 */
typedef pal::joint_trajectory_controller::JointTrajectoryVelocityController<trajectory_interface::QuinticSplineSegment<double> >
    JointSplineController;
}

namespace effort_controllers
{
/**
 * \brief Joint trajectory controller that represents trajectory segments as <b>quintic
 * splines</b> and sends
 * commands to an \b effort interface.
 */
typedef pal::joint_trajectory_controller::JointTrajectoryEffortController<trajectory_interface::QuinticSplineSegment<double> > JointSplineController;
}

namespace pos_vel_controllers
{
/**
 * \brief Joint trajectory controller that represents trajectory segments as <b>quintic
 * splines</b> and sends
 * commands to an \b pos_vel interface.
 */
typedef pal::joint_trajectory_controller::JointTrajectoryPosVelController<trajectory_interface::QuinticSplineSegment<double> > JointSplineController;
}

namespace pos_vel_acc_controllers
{
/**
 * \brief Joint trajectory controller that represents trajectory segments as <b>quintic
 * splines</b> and sends
 * commands to a \b pos_vel_acc interface.
 */
typedef pal::joint_trajectory_controller::JointTrajectoryPosVelAccController<trajectory_interface::QuinticSplineSegment<double> >
    JointSplineController;
}
}


PLUGINLIB_EXPORT_CLASS(pal::position_controllers::JointSplineController,
                       controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(pal::velocity_controllers::JointSplineController,
                       controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(pal::effort_controllers::JointSplineController,
                       controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(pal::pos_vel_controllers::JointSplineController,
                       controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(pal::pos_vel_acc_controllers::JointSplineController,
                       controller_interface::ControllerBase)
