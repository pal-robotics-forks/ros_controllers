///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
// Copyright (c) 2008, Willow Garage, Inc.
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

/// \author Adolfo Rodriguez Tsouroukdissian, Stuart Glaser

#pragma once

namespace pal
{
namespace joint_trajectory_controller
{
template <class SegmentImpl, class t_DerivedExecutor>
JointTrajectoryHandler<SegmentImpl, t_DerivedExecutor>::JointTrajectoryHandler()
  : verbose_(false)
  ,  // Set to true during debugging
  hold_trajectory_ptr_(new Trajectory)
{
  executor_ = static_cast<t_DerivedExecutor*>(this);
  state_publisher_trigger_ = false;

  // The verbose parameter is for advanced use as it breaks real-time safety
  // by enabling ROS logging services
  if (verbose_)
  {
    ROS_WARN_STREAM("The trajectory_handler verbose flag is enabled. "
                    << "This flag breaks real-time safety and should only be "
                    << "used for debugging");
  }
}


//
// ============================ Utility ===============================
//


template <class SegmentImpl, class t_DerivedExecutor>
void JointTrajectoryHandler<SegmentImpl, t_DerivedExecutor>::setHoldPosition(
    const ros::Time& time, RealtimeGoalHandlePtr gh)
{
  // Settle position in a fixed time. We do the following:
  // - Create segment that goes from current (pos,vel) to (pos,-vel) in 2x the desired
  // stop time
  // - Assuming segment symmetry, sample segment at its midpoint (desired stop time). It
  // should have zero velocity
  // - Create segment that goes from current state to above zero velocity state, in the
  // desired time
  // NOTE: The symmetry assumption from the second point above might not hold for all
  // possible segment types

  assert(n_joints_ == hold_trajectory_ptr_->size());

  State hold_start_state_(1);
  State hold_end_state_(1);

  const typename Segment::Time start_time = time.toSec();
  const typename Segment::Time end_time = time.toSec() + stop_trajectory_duration_;
  const typename Segment::Time end_time_2x = time.toSec() + 2.0 * stop_trajectory_duration_;

  // Create segment that goes from current (pos,vel) to (pos,-vel)
  executor_->getCurrentState(current_state_);
  for (unsigned int i = 0; i < n_joints_; ++i)
  {
    hold_start_state_.position[0] = current_state_.position[i];
    hold_start_state_.velocity[0] = current_state_.velocity[i];
    hold_start_state_.acceleration[0] = 0.0;

    hold_end_state_.position[0] = current_state_.position[i];
    hold_end_state_.velocity[0] = -current_state_.velocity[i];
    hold_end_state_.acceleration[0] = 0.0;


    (*hold_trajectory_ptr_)[i].front().init(start_time, hold_start_state_, end_time_2x,
                                            hold_end_state_);

    // Sample segment at its midpoint, that should have zero velocity
    (*hold_trajectory_ptr_)[i].front().sample(end_time, hold_end_state_);

    // Now create segment that goes from current state to one with zero end velocity
    (*hold_trajectory_ptr_)[i].front().init(start_time, hold_start_state_, end_time,
                                            hold_end_state_);

    // Set goal handle for the segment
    (*hold_trajectory_ptr_)[i].front().setGoalHandle(gh);
  }
  curr_trajectory_box_.set(hold_trajectory_ptr_);
}


/** \brief Holds the current position. */
template <class SegmentImpl, class t_DerivedExecutor>
inline void JointTrajectoryHandler<SegmentImpl, t_DerivedExecutor>::setDummyTrajectory(
    const ros::Time& time)
{
  // Update time data
  TimeData time_data;
  time_data.time = time;
  time_data.uptime = ros::Time(0.0);
  time_data_.initRT(time_data);

  // Hold current position
  setHoldPosition(time_data.uptime);

  // Initialize last state update time
  last_state_publish_time_ = time_data.uptime;
}


template <class SegmentImpl, class t_DerivedExecutor>
inline void JointTrajectoryHandler<SegmentImpl, t_DerivedExecutor>::preemptActiveGoal()
{
  RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);

  // Cancels the currently active goal
  if (current_active_goal)
  {
    // Marks the current goal as canceled
    rt_active_goal_.reset();
    current_active_goal->gh_.setCanceled();
  }
}


template <class SegmentImpl, class t_DerivedExecutor>
void JointTrajectoryHandler<SegmentImpl, t_DerivedExecutor>::reportStatus()
{
  // Set action feedback
  if (rt_active_goal_)
  {
    rt_active_goal_->preallocated_feedback_->header.stamp = time_data_.readFromRT()->time;
    rt_active_goal_->preallocated_feedback_->desired.positions = desired_state_.position;
    rt_active_goal_->preallocated_feedback_->desired.velocities = desired_state_.velocity;
    rt_active_goal_->preallocated_feedback_->desired.accelerations = desired_state_.acceleration;
    rt_active_goal_->preallocated_feedback_->actual.positions = current_state_.position;
    rt_active_goal_->preallocated_feedback_->actual.velocities = current_state_.velocity;
    rt_active_goal_->preallocated_feedback_->error.positions = state_error_.position;
    rt_active_goal_->preallocated_feedback_->error.velocities = state_error_.velocity;
    rt_active_goal_->setFeedback(rt_active_goal_->preallocated_feedback_);
  }

  // Check if it's time to publish
  if (true == state_publisher_trigger_)
  {
    if (state_publisher_ && state_publisher_->trylock())
    {
      last_state_publish_time_ += state_publisher_period_;
      state_publisher_trigger_ = false;

      state_publisher_->msg_.header.stamp = time_data_.readFromRT()->time;
      state_publisher_->msg_.desired.positions = desired_state_.position;
      state_publisher_->msg_.desired.velocities = desired_state_.velocity;
      state_publisher_->msg_.desired.accelerations = desired_state_.acceleration;
      state_publisher_->msg_.actual.positions = current_state_.position;
      state_publisher_->msg_.actual.velocities = current_state_.velocity;
      state_publisher_->msg_.error.positions = state_error_.position;
      state_publisher_->msg_.error.velocities = state_error_.velocity;

      state_publisher_->unlockAndPublish();
    }
  }
}


template <class SegmentImpl, class t_DerivedExecutor>
bool JointTrajectoryHandler<SegmentImpl, t_DerivedExecutor>::updateTrajectoryCommand(
    const JointTrajectoryConstPtr& msg, RealtimeGoalHandlePtr gh)
{
  // Preconditions
  if (false == executor_->isRunning())
  {
    ROS_ERROR_NAMED("JointTrajectoryHandler",
                    "Can't accept new commands. Controller is not running.");
    return false;
  }

  if (!msg)
  {
    ROS_WARN_NAMED("JointTrajectoryHandler", "Received null-pointer trajectory message, skipping.");
    return false;
  }

  // Time data
  TimeData* time_data =
      time_data_.readFromRT();  // TODO: Grrr, we need a lock-free data structure here!

  // Time of the next update
  const ros::Time next_update_time = time_data->time + time_data->period;

  // Uptime of the next update
  ros::Time next_update_uptime = time_data->uptime + time_data->period;

  // Hold current position if trajectory is empty
  if (msg->points.empty())
  {
    setHoldPosition(time_data->uptime, gh);
    ROS_DEBUG_NAMED("JointTrajectoryHandler", "Empty trajectory command, stopping.");
    return true;
  }

  // Trajectory initialization options
  TrajectoryPtr curr_traj_ptr;
  curr_trajectory_box_.get(curr_traj_ptr);

  pal::joint_trajectory_controller::InitJointTrajectoryOptions<Trajectory> options;
  options.other_time_base = &next_update_uptime;
  options.current_trajectory = curr_traj_ptr.get();
  options.joint_names = &executor_->getJointNames();
  options.angle_wraparound = &executor_->getAngleWraparound();
  options.rt_goal_handle = gh;
  options.default_tolerances = &default_tolerances_;
  options.allow_partial_joints_goal = allow_partial_joints_goal_;

  // Update currently executing trajectory
  try
  {
    TrajectoryPtr traj_ptr(new Trajectory);
    *traj_ptr = pal::joint_trajectory_controller::initJointTrajectory<Trajectory>(
        *msg, next_update_time, options);
    if (!traj_ptr->empty())
    {
      curr_trajectory_box_.set(traj_ptr);
    }
    else
    {
      // All trajectory points are in the past, nothing new to execute. Keep on executing
      // current trajectory
      return false;
    }
  }
  catch (const std::invalid_argument& ex)
  {
    ROS_ERROR_STREAM_NAMED("JointTrajectoryHandler", ex.what());
    return false;
  }
  catch (...)
  {
    ROS_ERROR_NAMED("JointTrajectoryHandler",
                    "Unexpected exception caught when initializing trajectory from ROS message data.");
    return false;
  }

  return true;
}


//
// ============================ Callbacks ===============================
//


template <class SegmentImpl, class t_DerivedExecutor>
void JointTrajectoryHandler<SegmentImpl, t_DerivedExecutor>::cb_processGoal(GoalHandle gh)
{
  ROS_DEBUG_STREAM_NAMED("JointTrajectoryHandler", "Received new action goal");

  // Precondition: Running controller
  if (false == executor_->isRunning())
  {
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    result.error_string = "Can't accept new action goals. Controller is not running.";
    ROS_ERROR_NAMED("JointTrajectoryHandler",
                    "Can't accept new action goals. Controller is not running.");
    gh.setRejected(result);
    return;
  }

  // If partial joints goals are not allowed, goal should specify all controller joints
  if (!allow_partial_joints_goal_)
  {
    if (gh.getGoal()->trajectory.joint_names.size() != n_joints_)
    {
      control_msgs::FollowJointTrajectoryResult result;
      result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
      result.error_string =
          "Number of joints in the goal doesn't match the number of controller joints.";
      ROS_ERROR_NAMED("JointTrajectoryHandler",
                      "Number of joints in the goal doesn't match the number of controller joints.");
      gh.setRejected(result);
      return;
    }
  }

  // Goal should specify valid controller joints (they can be ordered differently). Reject
  // if this is not the case
  std::vector<unsigned int> mapping_vector =
      internal::mapping(gh.getGoal()->trajectory.joint_names, executor_->getJointNames());

  if (mapping_vector.empty())
  {
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
    result.error_string = "Could not match the goal joint names to the controller joint names.";
    ROS_ERROR_NAMED("JointTrajectoryHandler",
                    "Could not match the goal joint names to the controller joint names.");
    gh.setRejected(result);
    return;
  }

  // Try to update new trajectory
  RealtimeGoalHandlePtr rt_goal(new RealtimeGoalHandle(gh));
  const bool update_ok = updateTrajectoryCommand(
      ::joint_trajectory_controller::internal::share_member(gh.getGoal(), gh.getGoal()->trajectory),
      rt_goal);
  rt_goal->preallocated_feedback_->joint_names = executor_->getJointNames();

  if (update_ok)
  {
    // Accept new goal
    preemptActiveGoal();
    gh.setAccepted();
    rt_active_goal_ = rt_goal;

    // Setup goal status checking timer
    goal_handle_timer_ = executor_->getNodeHandle().createTimer(
        action_monitor_period_, &RealtimeGoalHandle::runNonRealtime, rt_goal);
    goal_handle_timer_.start();
  }
  else
  {
    // Reject invalid goal
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    result.error_string = "Could not process command, see logs for more info.";
    gh.setRejected(result);
  }
}


template <class SegmentImpl, class t_DerivedExecutor>
void JointTrajectoryHandler<SegmentImpl, t_DerivedExecutor>::cb_cancelGoal(GoalHandle gh)
{
  RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);

  // Check that cancel request refers to currently active goal (if any)
  if (current_active_goal && current_active_goal->gh_ == gh)
  {
    // Reset current goal
    rt_active_goal_.reset();

    // Controller uptime
    const ros::Time uptime = time_data_.readFromRT()->uptime;

    // Enter hold current position mode
    setHoldPosition(uptime);
    ROS_DEBUG_NAMED("JointTrajectoryHandler",
                    "Canceling active action goal because cancel callback recieved from actionlib.");

    // Mark the current goal as canceled
    current_active_goal->gh_.setCanceled();
  }
}


template <class SegmentImpl, class t_DerivedExecutor>
bool JointTrajectoryHandler<SegmentImpl, t_DerivedExecutor>::cb_queryStateService(
    control_msgs::QueryTrajectoryState::Request& req,
    control_msgs::QueryTrajectoryState::Response& resp)
{
  // Preconditions
  if (false == executor_->isRunning())
  {
    ROS_ERROR_NAMED("JointTrajectoryHandler",
                    "Can't sample trajectory. Controller is not running.");
    return false;
  }

  // Convert request time to internal monotonic representation
  TimeData* time_data = time_data_.readFromRT();
  const ros::Duration time_offset = req.time - time_data->time;
  const ros::Time sample_time = time_data->uptime + time_offset;

  // Sample trajectory at requested time
  TrajectoryPtr curr_traj_ptr;
  curr_trajectory_box_.get(curr_traj_ptr);
  Trajectory& curr_traj = *curr_traj_ptr;

  State response_point(n_joints_);

  for (unsigned int i = 0; i < n_joints_; ++i)
  {
    State state;
    typename TrajectoryPerJoint::const_iterator segment_it =
        sample(curr_traj[i], sample_time.toSec(), state);
    if (curr_traj[i].end() == segment_it)
    {
      ROS_ERROR_STREAM_NAMED("JointTrajectoryHandler",
                             "Requested sample time precedes trajectory start time.");
      return false;
    }

    response_point.position[i] = state.position[0];
    response_point.velocity[i] = state.velocity[0];
    response_point.acceleration[i] = state.acceleration[0];
  }

  // Populate response
  resp.name = executor_->getJointNames();
  resp.position = response_point.position;
  resp.velocity = response_point.velocity;
  resp.acceleration = response_point.acceleration;

  return true;
}



template <class SegmentImpl, class t_DerivedExecutor>
inline void JointTrajectoryHandler<SegmentImpl, t_DerivedExecutor>::cb_trajectoryCommand(
    const JointTrajectoryConstPtr& msg)
{
  const bool update_ok = updateTrajectoryCommand(msg, RealtimeGoalHandlePtr());
  if (update_ok)
  {
    preemptActiveGoal();
  }
}


//
// ============================ init / update ===============================
//



template <class SegmentImpl, class t_DerivedExecutor>
bool JointTrajectoryHandler<SegmentImpl, t_DerivedExecutor>::init(ros::NodeHandle& controller_nh)
{
  // State publish rate
  double state_publish_rate = 50.0;
  controller_nh.getParam("state_publish_rate", state_publish_rate);
  ROS_DEBUG_STREAM_NAMED("JointTrajectoryHandler",
                         "Controller state will be published at " << state_publish_rate << "Hz.");
  state_publisher_period_ = ros::Duration(1.0 / state_publish_rate);

  // Action status checking update rate
  double action_monitor_rate = 20.0;
  controller_nh.getParam("action_monitor_rate", action_monitor_rate);
  action_monitor_period_ = ros::Duration(1.0 / action_monitor_rate);
  ROS_DEBUG_STREAM_NAMED("JointTrajectoryHandler",
                         "Action status changes will be monitored at "
                             << action_monitor_rate << "Hz.");

  // Stop trajectory duration
  stop_trajectory_duration_ = 0.0;
  if (!controller_nh.getParam("stop_trajectory_duration", stop_trajectory_duration_))
  {
    // TODO: Remove this check/warning in Indigo
    if (controller_nh.getParam("hold_trajectory_duration", stop_trajectory_duration_))
    {
      ROS_WARN("The 'hold_trajectory_duration' has been deprecated in favor of the 'stop_trajectory_duration' parameter. Please update your controller configuration.");
    }
  }
  ROS_DEBUG_STREAM_NAMED("JointTrajectoryHandler",
                         "Stop trajectory has a duration of " << stop_trajectory_duration_
                                                              << "s.");

  // Checking if partial trajectories are allowed
  controller_nh.param<bool>("allow_partial_joints_goal", allow_partial_joints_goal_, false);
  if (allow_partial_joints_goal_)
  {
    ROS_DEBUG_NAMED("JointTrajectoryHandler", "Goals with partial set of joints are allowed");
  }



  // Default tolerances
  ros::NodeHandle tol_nh(controller_nh, "constraints");
  default_tolerances_ = ::joint_trajectory_controller::getSegmentTolerances<Scalar>(
      tol_nh, executor_->getJointNames());

  // ROS API: Published topics
  state_publisher_.reset(new StatePublisher(controller_nh, "state", 1));

  // ROS API: Subscribed topics
  trajectory_command_sub_ =
      controller_nh.subscribe("command", 1, &JointTrajectoryHandler::cb_trajectoryCommand, this);


  // ROS API: Action interface
  action_server_ = boost::make_shared<ActionServer>(
      controller_nh, "follow_joint_trajectory",
      boost::bind(&JointTrajectoryHandler::cb_processGoal, this, _1),
      boost::bind(&JointTrajectoryHandler::cb_cancelGoal, this, _1), false);
  action_server_->start();

  // ROS API: Provided services
  query_state_service_ = controller_nh.advertiseService(
      "query_state", &JointTrajectoryHandler::cb_queryStateService, this);

  n_joints_ = executor_->getJointNames().size();

  // Preeallocate resources
  current_state_ = State(n_joints_);
  desired_state_ = State(n_joints_);
  state_error_ = State(n_joints_);
  desired_joint_state_ = State(1);
  state_joint_error_ = State(1);

  successful_joint_traj_ = boost::dynamic_bitset<>(n_joints_);

  // Initialize trajectory with all joints
  State current_joint_state_(1);
  for (unsigned int i = 0; i < n_joints_; ++i)
  {
    current_joint_state_.position[0] = current_state_.position[i];
    current_joint_state_.velocity[0] = current_state_.velocity[i];
    Segment hold_segment(0.0, current_joint_state_, 0.0, current_joint_state_);

    TrajectoryPerJoint joint_segment;
    joint_segment.resize(1, hold_segment);
    hold_trajectory_ptr_->push_back(joint_segment);
  }

  {
    state_publisher_->lock();
    state_publisher_->msg_.joint_names = executor_->getJointNames();
    state_publisher_->msg_.desired.positions.resize(n_joints_);
    state_publisher_->msg_.desired.velocities.resize(n_joints_);
    state_publisher_->msg_.desired.accelerations.resize(n_joints_);
    state_publisher_->msg_.actual.positions.resize(n_joints_);
    state_publisher_->msg_.actual.velocities.resize(n_joints_);
    state_publisher_->msg_.error.positions.resize(n_joints_);
    state_publisher_->msg_.error.velocities.resize(n_joints_);
    state_publisher_->unlock();
  }

  return true;
}


template <class SegmentImpl, class t_DerivedExecutor>
void JointTrajectoryHandler<SegmentImpl, t_DerivedExecutor>::updateDesiredState(
    const ros::Time& time, const ros::Duration& period)
{
  // Get currently followed trajectory
  TrajectoryPtr curr_traj_ptr;
  curr_trajectory_box_.get(curr_traj_ptr);
  Trajectory& curr_traj = *curr_traj_ptr;

  // Update time data
  TimeData time_data;
  time_data.time = time;      // Cache current time
  time_data.period = period;  // Cache current control period
  time_data.uptime =
      time_data_.readFromRT()->uptime + period;  // Update controller uptime
  time_data_.writeFromNonRT(
      time_data);  // TODO: Grrr, we need a lock-free data structure here!

  // NOTE: It is very important to execute the two above code blocks in the specified
  // sequence: first get current
  // trajectory, then update time data. Hopefully the following paragraph sheds a bit of
  // light on the rationale.
  // The non-rt thread responsible for processing new commands enqueues trajectories that
  // can start at the _next_
  // control cycle (eg. zero start time) or later (eg. when we explicitly request a start
  // time in the future).
  // If we reverse the order of the two blocks above, and update the time data first; it's
  // possible that by the time we
  // fetch the currently followed trajectory, it has been updated by the non-rt thread
  // with something that starts in the
  // next control cycle, leaving the current cycle without a valid trajectory.

  // Update current state and state error
  executor_->getCurrentState(current_state_);
  for (unsigned int i = 0; i < n_joints_; ++i)
  {
    // There's no acceleration data available in a joint handle

    typename TrajectoryPerJoint::const_iterator segment_it =
        sample(curr_traj[i], time_data.uptime.toSec(), desired_joint_state_);
    if (curr_traj[i].end() == segment_it)
    {
      // Non-realtime safe, but should never happen under normal operation
      ROS_ERROR_NAMED("JointTrajectoryHandler",
                      "Unexpected error: No trajectory defined at current time. Please contact the package maintainer.");
      return;
    }
    desired_state_.position[i] = desired_joint_state_.position[0];
    desired_state_.velocity[i] = desired_joint_state_.velocity[0];
    desired_state_.acceleration[i] = desired_joint_state_.acceleration[0];

    state_joint_error_.position[0] = angles::shortest_angular_distance(
        current_state_.position[i], desired_joint_state_.position[0]);
    state_joint_error_.velocity[0] =
        desired_joint_state_.velocity[0] - current_state_.velocity[i];
    state_joint_error_.acceleration[0] = 0.0;

    state_error_.position[i] = angles::shortest_angular_distance(
        current_state_.position[i], desired_joint_state_.position[0]);
    state_error_.velocity[i] = desired_joint_state_.velocity[0] - current_state_.velocity[i];
    state_error_.acceleration[i] = 0.0;

    // Check tolerances
    const RealtimeGoalHandlePtr rt_segment_goal = segment_it->getGoalHandle();
    if (rt_segment_goal && rt_segment_goal == rt_active_goal_)
    {
      // Check tolerances
      if (time_data.uptime.toSec() < segment_it->endTime())
      {
        // Currently executing a segment: check path tolerances
        const ::joint_trajectory_controller::SegmentTolerancesPerJoint<Scalar>& joint_tolerances =
            segment_it->getTolerances();
        if (!checkStateTolerancePerJoint(state_joint_error_, joint_tolerances.state_tolerance))
        {
          if (verbose_)
          {
            ROS_ERROR_STREAM_NAMED(
                "JointTrajectoryHandler",
                "Path tolerances failed for joint: " << executor_->getJointNames()[i]);
            checkStateTolerancePerJoint(state_joint_error_,
                                        joint_tolerances.state_tolerance, true);
          }
          rt_segment_goal->preallocated_result_->error_code =
              control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
          rt_segment_goal->setAborted(rt_segment_goal->preallocated_result_);
          rt_active_goal_.reset();
          successful_joint_traj_.reset();
        }
      }
      else if (segment_it == --curr_traj[i].end())
      {
        if (verbose_)
          ROS_DEBUG_STREAM_THROTTLE_NAMED(1, "JointTrajectoryHandler",
                                          "Finished executing last segment, checking goal tolerances");

        // Controller uptime
        const ros::Time uptime = time_data_.readFromRT()->uptime;

        // Checks that we have ended inside the goal tolerances
        const ::joint_trajectory_controller::SegmentTolerancesPerJoint<Scalar>& tolerances =
            segment_it->getTolerances();
        const bool inside_goal_tolerances =
            checkStateTolerancePerJoint(state_joint_error_, tolerances.goal_state_tolerance);

        if (inside_goal_tolerances)
        {
          successful_joint_traj_[i] = 1;
        }
        else if (uptime.toSec() < segment_it->endTime() + tolerances.goal_time_tolerance)
        {
          // Still have some time left to meet the goal state tolerances
        }
        else
        {
          if (verbose_)
          {
            ROS_ERROR_STREAM_NAMED(
                "JointTrajectoryHandler",
                "Goal tolerances failed for joint: " << executor_->getJointNames()[i]);
            // Check the tolerances one more time to output the errors that occurs
            checkStateTolerancePerJoint(state_joint_error_, tolerances.goal_state_tolerance, true);
          }

          rt_segment_goal->preallocated_result_->error_code =
              control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED;
          rt_segment_goal->setAborted(rt_segment_goal->preallocated_result_);
          rt_active_goal_.reset();
          successful_joint_traj_.reset();
        }
      }
    }
  }


  // If there is an active goal and all segments finished successfully then set goal as
  // succeeded
  RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);
  if (current_active_goal and successful_joint_traj_.count() == n_joints_)
  {
    current_active_goal->preallocated_result_->error_code =
        control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    current_active_goal->setSucceeded(current_active_goal->preallocated_result_);
    rt_active_goal_.reset();
    successful_joint_traj_.reset();
  }

  state_publisher_trigger_ = (!state_publisher_period_.isZero() &&
                              last_state_publish_time_ + state_publisher_period_ < time);
}

}  // namespace
}
