///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of PAL Robotics S.L. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
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

/// \author Adolfo Rodriguez Tsouroukdissian

#pragma once

// URDF
#include <urdf/model.h>

#include <cassert>
#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <ros/node_handle.h>
#include <ros/time.h>

#include <controller_interface/controller.h>
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <hardware_interface/posvelacc_command_interface.h>
#include <hardware_interface/internal/demangle_symbol.h>

#include <pal_joint_trajectory_controller/joint_trajectory_handler.h>

namespace pal
{
namespace joint_trajectory_controller
{
template <class t_HardwareInterface, class t_TrajectoryGenerator>
class JointTrajectoryControllerBase
    : public controller_interface::Controller<t_HardwareInterface>,
      protected JointTrajectoryHandler<t_TrajectoryGenerator, JointTrajectoryControllerBase<t_HardwareInterface, t_TrajectoryGenerator> >
{
protected:
  typedef
      typename ::joint_trajectory_controller::JointTrajectorySegment<t_TrajectoryGenerator>::State State;
  using JointTrajectoryHandler<t_TrajectoryGenerator, JointTrajectoryControllerBase>::updateDesiredState;
  using JointTrajectoryHandler<t_TrajectoryGenerator, JointTrajectoryControllerBase>::reportStatus;
  using JointTrajectoryHandler<t_TrajectoryGenerator, JointTrajectoryControllerBase>::setDummyTrajectory;
  using JointTrajectoryHandler<t_TrajectoryGenerator, JointTrajectoryControllerBase>::preemptActiveGoal;


private:
  std::string name_;                      ///< Controller name.
  std::vector<std::string> joint_names_;  ///< Controlled joint names.
  std::vector<bool> angle_wraparound_;  ///< Whether controlled joints wrap around or not.
  ros::NodeHandle controller_nh_;

protected:
  ///< Handles to controlled joints.
  std::vector<typename t_HardwareInterface::ResourceHandleType> joints_;


public:
  const void getCurrentState(State& current_state) const
  {
    const std::size_t n_joints = joints_.size();
    for (std::size_t i = 0; i < n_joints; ++i)
    {
      current_state.position[i] = joints_[i].getPosition();
      current_state.velocity[i] = joints_[i].getVelocity();
    }
  }

  const std::vector<std::string>& getJointNames() const
  {
    return (joint_names_);
  }

  const std::vector<bool>& getAngleWraparound() const
  {
    return (angle_wraparound_);
  }

  const ros::NodeHandle& getNodeHandle() const
  {
    return (controller_nh_);
  }

  /**
   * @brief const version of isRunning method defined in
   * controller_interface::Controller<>
   *
   * @return
   */
  bool isRunning() const
  {
    return (controller_interface::Controller<t_HardwareInterface>::RUNNING ==
            controller_interface::Controller<t_HardwareInterface>::state_);
  }


public:
  /** \name Non Real-Time Safe Functions
   *\{*/
  bool init(t_HardwareInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
  {
    controller_nh_ = controller_nh;


    // Controller name
    name_ = ::joint_trajectory_controller::internal::getLeafNamespace(controller_nh);


    // List of controlled joints
    joint_names_ = ::joint_trajectory_controller::internal::getStrings(controller_nh, "joints");
    if (joint_names_.empty())
    {
      return false;
    }
    const std::size_t n_joints = joint_names_.size();


    // URDF joints
    urdf::ModelSharedPtr urdf =
        ::joint_trajectory_controller::internal::getUrdf(root_nh, "robot_description");
    if (!urdf)
    {
      return false;
    }

    std::vector<urdf::JointConstSharedPtr> urdf_joints =
        ::joint_trajectory_controller::internal::getUrdfJoints(*urdf, joint_names_);
    if (urdf_joints.empty())
    {
      return false;
    }
    assert(n_joints == urdf_joints.size());



    angle_wraparound_.resize(n_joints);
    for (std::size_t i = 0; i < n_joints; ++i)
    {
      // Whether a joint is continuous (ie. has angle wraparound)
      angle_wraparound_[i] = urdf_joints[i]->type == urdf::Joint::CONTINUOUS;

      ROS_DEBUG_STREAM_NAMED(name_,
                             "Found " << (angle_wraparound_[i] ? "" : "non-")
                                      << "continuous joint '" << joint_names_[i] << "' in '"
                                      << this->getHardwareInterfaceType() << "'.");
    }

    joints_.resize(n_joints);
    for (std::size_t i = 0; i < n_joints; ++i)
    {
      // Joint handle
      try
      {
        joints_[i] = hw->getHandle(joint_names_[i]);
      }
      catch (...)
      {
        ROS_ERROR_STREAM_NAMED(name_,
                               "Could not find joint '"
                                   << joint_names_[i] << "' in '"
                                   << this->getHardwareInterfaceType() << "'.");
        return (false);
      }
    }

    if (false == configure(controller_nh))
    {
      return (false);
    }


    ROS_DEBUG_STREAM_NAMED(
        name_,
        "Initialized controller '"
            << name_ << "' with:"
            << "\n- Number of joints: " << n_joints << "\n- Hardware interface type: '"
            << this->getHardwareInterfaceType() << "'"
            << "\n- Trajectory segment type: '"
            << hardware_interface::internal::demangledTypeName<t_TrajectoryGenerator>() << "'");

    return (JointTrajectoryHandler<t_TrajectoryGenerator, JointTrajectoryControllerBase>::init(
        controller_nh));
  }
  /*\}*/


  /** \name Real-Time Safe Functions
   *\{*/
  void update(const ros::Time& time, const ros::Duration& period)
  {
    updateDesiredState(time, period);

    computeAndSendCommands(period);

    reportStatus();
  }

  /** \brief Holds the current position. */
  void starting(const ros::Time& time)
  {
    setDummyTrajectory(time);
    resetControllers();
  }


  /** \brief Cancels the active action goal, if any. */
  void stopping(const ros::Time& /*time*/)
  {
    preemptActiveGoal();
  }
  /*\}*/



protected:
  virtual bool configure(ros::NodeHandle& /*controller_nh*/)
  {
    return true;
  }

  virtual void resetControllers()
  {
  }

  virtual void computeAndSendCommands(const ros::Duration& period) = 0;
};



/**
 * \brief Adapter for a position-controlled hardware interface. Forwards desired positions
 * as commands.
 *
 * The following is an example configuration of a controller that uses this adapter.
 * \code
 * head_controller:
 *   type: "position_controllers/JointTrajectoryController"
 *   joints:
 *     - head_1_joint
 *     - head_2_joint
 *
 *   constraints:
 *     goal_time: 0.6
 *     stopped_velocity_tolerance: 0.02
 *     head_1_joint: {trajectory: 0.05, goal: 0.02}
 *     head_2_joint: {trajectory: 0.05, goal: 0.02}
 *   stop_trajectory_duration: 0.5
 *   state_publish_rate:  25
 * \endcode
 */
template <class t_TrajectoryGenerator>
class JointTrajectoryPositionController
    : public JointTrajectoryControllerBase<hardware_interface::PositionJointInterface, t_TrajectoryGenerator>
{
private:
  using JointTrajectoryControllerBase<hardware_interface::PositionJointInterface, t_TrajectoryGenerator>::joints_;
  using JointTrajectoryControllerBase<hardware_interface::PositionJointInterface, t_TrajectoryGenerator>::getDesiredState;
  typedef
      typename ::joint_trajectory_controller::JointTrajectorySegment<t_TrajectoryGenerator>::State State;


public:
  void resetControllers()
  {
    // Semantic zero for commands
    for (std::size_t i = 0; i < joints_.size(); ++i)
    {
      joints_[i].setCommand(joints_[i].getPosition());
    }
  }


  void computeAndSendCommands(const ros::Duration& /*period*/)
  {
    const State& desired_state = getDesiredState();

    // Forward desired position to command
    const std::size_t n_joints = joints_.size();
    for (std::size_t i = 0; i < n_joints; ++i)
    {
      joints_[i].setCommand(desired_state.position[i]);
    }
  }
};


/**
 * \brief Adapter for an velocity-controlled hardware interface. Maps position and
 * velocity errors to velocity commands
 * through a velocity PID loop.
 *
 * The following is an example configuration of a controller that uses this adapter.
 * Notice the \p gains entry:
 * \code
 * head_controller:
 *   type: "velocity_controllers/JointTrajectoryController"
 *   joints:
 *     - head_1_joint
 *     - head_2_joint
 *   gains:
 *     head_1_joint: {p: 200, d: 1, i: 5, i_clamp: 1}
 *     head_2_joint: {p: 200, d: 1, i: 5, i_clamp: 1}
 *   constraints:
 *     goal_time: 0.6
 *     stopped_velocity_tolerance: 0.02
 *     head_1_joint: {trajectory: 0.05, goal: 0.02}
 *     head_2_joint: {trajectory: 0.05, goal: 0.02}
 *   stop_trajectory_duration: 0.5
 *   state_publish_rate:  25
 * \endcode
 */
template <class t_TrajectoryGenerator>
class JointTrajectoryVelocityController
    : public JointTrajectoryControllerBase<hardware_interface::VelocityJointInterface, t_TrajectoryGenerator>
{
private:
  using JointTrajectoryControllerBase<hardware_interface::VelocityJointInterface, t_TrajectoryGenerator>::joints_;
  using JointTrajectoryControllerBase<hardware_interface::VelocityJointInterface, t_TrajectoryGenerator>::getDesiredStateError;
  typedef
      typename ::joint_trajectory_controller::JointTrajectorySegment<t_TrajectoryGenerator>::State State;

  typedef boost::shared_ptr<control_toolbox::Pid> PidPtr;
  std::vector<PidPtr> pids_;


public:
  bool configure(ros::NodeHandle& controller_nh)
  {
    // Initialize PIDs
    pids_.resize(joints_.size());
    for (std::size_t i = 0; i < pids_.size(); ++i)
    {
      // Node handle to PID gains
      ros::NodeHandle joint_nh(controller_nh, std::string("gains/") + joints_[i].getName());

      // Init PID gains from ROS parameter server
      pids_[i].reset(new control_toolbox::Pid());
      if (!pids_[i]->init(joint_nh))
      {
        ROS_WARN_STREAM("Failed to initialize PID gains from ROS parameter server.");
        return false;
      }
    }

    return true;
  }


  void resetControllers()
  {
    // Reset PIDs, zero velocity commands
    for (std::size_t i = 0; i < pids_.size(); ++i)
    {
      pids_[i]->reset();
      joints_[i].setCommand(0.0);
    }
  }


  void computeAndSendCommands(const ros::Duration& period)
  {
    const State& state_error = getDesiredStateError();

    const std::size_t n_joints = joints_.size();

    // Preconditions
    assert(n_joints == state_error.position.size());
    assert(n_joints == state_error.velocity.size());

    // Update PIDs
    for (std::size_t i = 0; i < n_joints; ++i)
    {
      const double command =
          pids_[i]->computeCommand(state_error.position[i], state_error.velocity[i], period);
      joints_[i].setCommand(command);
    }
  }
};


/**
 * \brief Adapter for an effort-controlled hardware interface. Maps position and velocity
 * errors to effort commands
 * through a position PID loop.
 *
 * The following is an example configuration of a controller that uses this adapter.
 * Notice the \p gains entry:
 * \code
 * head_controller:
 *   type: "effort_controllers/JointTrajectoryController"
 *   joints:
 *     - head_1_joint
 *     - head_2_joint
 *   gains:
 *     head_1_joint: {p: 200, d: 1, i: 5, i_clamp: 1}
 *     head_2_joint: {p: 200, d: 1, i: 5, i_clamp: 1}
 *   constraints:
 *     goal_time: 0.6
 *     stopped_velocity_tolerance: 0.02
 *     head_1_joint: {trajectory: 0.05, goal: 0.02}
 *     head_2_joint: {trajectory: 0.05, goal: 0.02}
 *   stop_trajectory_duration: 0.5
 *   state_publish_rate:  25
 * \endcode
 */
template <class t_TrajectoryGenerator>
class JointTrajectoryEffortController
    : public JointTrajectoryControllerBase<hardware_interface::EffortJointInterface, t_TrajectoryGenerator>
{
private:
  using JointTrajectoryControllerBase<hardware_interface::EffortJointInterface, t_TrajectoryGenerator>::joints_;
  using JointTrajectoryControllerBase<hardware_interface::EffortJointInterface, t_TrajectoryGenerator>::getDesiredStateError;
  typedef
      typename ::joint_trajectory_controller::JointTrajectorySegment<t_TrajectoryGenerator>::State State;

  typedef boost::shared_ptr<control_toolbox::Pid> PidPtr;
  std::vector<PidPtr> pids_;


public:
  bool configure(ros::NodeHandle& controller_nh)
  {
    // Initialize PIDs
    pids_.resize(joints_.size());
    for (std::size_t i = 0; i < pids_.size(); ++i)
    {
      // Node handle to PID gains
      ros::NodeHandle joint_nh(controller_nh, std::string("gains/") + joints_[i].getName());

      // Init PID gains from ROS parameter server
      pids_[i].reset(new control_toolbox::Pid());
      if (!pids_[i]->init(joint_nh))
      {
        ROS_WARN_STREAM("Failed to initialize PID gains from ROS parameter server.");
        return false;
      }
    }

    return true;
  }


  void resetControllers()
  {
    // Reset PIDs, zero effort commands
    for (std::size_t i = 0; i < pids_.size(); ++i)
    {
      pids_[i]->reset();
      joints_[i].setCommand(0.0);
    }
  }


  void computeAndSendCommands(const ros::Duration& period)
  {
    const State& state_error = getDesiredStateError();

    const std::size_t n_joints = joints_.size();
    assert(n_joints == state_error.position.size());
    assert(n_joints == state_error.velocity.size());

    // Update PIDs
    for (std::size_t i = 0; i < n_joints; ++i)
    {
      const double command =
          pids_[i]->computeCommand(state_error.position[i], state_error.velocity[i], period);
      joints_[i].setCommand(command);
    }
  }
};


/**
 * \brief Adapter for a pos-vel hardware interface. Forwards desired positions with
 * velcities as commands.
 */
template <class t_TrajectoryGenerator>
class JointTrajectoryPosVelController
    : public JointTrajectoryControllerBase<hardware_interface::PosVelJointInterface, t_TrajectoryGenerator>
{
private:
  using JointTrajectoryControllerBase<hardware_interface::PosVelJointInterface, t_TrajectoryGenerator>::joints_;
  using JointTrajectoryControllerBase<hardware_interface::PosVelJointInterface, t_TrajectoryGenerator>::getDesiredState;
  typedef
      typename ::joint_trajectory_controller::JointTrajectorySegment<t_TrajectoryGenerator>::State State;


public:
  void computeAndSendCommands(const ros::Duration& period)
  {
    const State& desired_state = getDesiredState();

    // Forward desired position to command
    const std::size_t n_joints = joints_.size();
    for (std::size_t i = 0; i < n_joints; ++i)
    {
      joints_[i].setCommand(desired_state.position[i], desired_state.velocity[i]);
    }
  }
};


/**
 * \brief Adapter for a spline-controlled hardware interface. Forwards desired positions
 * as commands.
 */
template <class t_TrajectoryGenerator>
class JointTrajectoryPosVelAccController
    : public JointTrajectoryControllerBase<hardware_interface::PosVelAccJointInterface, t_TrajectoryGenerator>
{
private:
  using JointTrajectoryControllerBase<hardware_interface::PosVelAccJointInterface, t_TrajectoryGenerator>::joints_;
  using JointTrajectoryControllerBase<hardware_interface::PosVelAccJointInterface, t_TrajectoryGenerator>::getDesiredState;
  typedef
      typename ::joint_trajectory_controller::JointTrajectorySegment<t_TrajectoryGenerator>::State State;


public:
  void computeAndSendCommands(const ros::Duration& /*period*/)
  {
    const State& desired_state = getDesiredState();

    // Forward desired position to command
    const std::size_t n_joints = joints_.size();
    for (std::size_t i = 0; i < n_joints; ++i)
    {
      joints_[i].setCommand(desired_state.position[i], desired_state.velocity[i],
                            desired_state.acceleration[i]);
    }
  }
};
}
}
