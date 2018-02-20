/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, University of Colorado, Boulder
 *  Copyright (c) 2014, PAL Robotics SL.
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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

/* Author: Dave Coleman, Bence Magyar
   Desc:   Controller to allow joint controllers to easily switch modes between position, velocity, and effort-based control
*/

#include <pluginlib/class_list_macros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_mode_interface.h>

namespace joint_mode_controller {

  hardware_interface::JointCommandModes stringToMode(const std::string& s)
  {
    if(s == "hardware_interface/PositionJointInterface")
        return hardware_interface::MODE_POSITION;
    else if(s == "hardware_interface/VelocityJointInterface")
        return hardware_interface::MODE_VELOCITY;
    else if(s == "hardware_interface/EffortJointInterface")
        return hardware_interface::MODE_EFFORT;
    else
    {
      //TODO: what to do here?
        ROS_FATAL_STREAM("Invalid mode to set joint to " << s);
        return hardware_interface::MODE_OTHER;
    }
  }

class JointModeController: public controller_interface::Controller<hardware_interface::JointModeInterface>
{

private:
  std::map<std::string, hardware_interface::JointCommandModes> joint_modes_;

  std::vector<hardware_interface::JointModeHandle> mode_handles_;
  std::vector<hardware_interface::JointCommandModes> modes_;

public:
  JointModeController()
  {}

  ~JointModeController()
  {}

  bool init(
    hardware_interface::JointModeInterface *hw, ros::NodeHandle &nh)
  {
    typedef std::map<std::string, std::string> map_s_s;

    // Get the names of handles and their respective target modes
    map_s_s joint_modes;
    if (!nh.getParam("mode_handles", joint_modes))
    {
      ROS_DEBUG_STREAM_NAMED("init","No mode_handles specified in namespace '"
                             << nh.getNamespace());
    }

    // Save the handles and parse mode names for usage in the starting() function
    for(map_s_s::const_iterator i = joint_modes.begin(); i != joint_modes.end(); ++i)
    {
      mode_handles_.push_back(hardware_interface::JointModeHandle(hw->getHandle(i->first)));
      ROS_WARN_STREAM("Switching " << i->first << " to " << i->second);
      modes_.push_back(stringToMode(i->second));
    }

    return true;
  }

  void update(const ros::Time& time, const ros::Duration& period)
  {}

  void starting(const ros::Time& time) 
  {
    // The controller changes the mode when the controller is started, not when it is loaded
    for(size_t i=0; i<mode_handles_.size(); ++i)
      mode_handles_[i].setMode(modes_[i]);
  }

};
} // namespace

PLUGINLIB_EXPORT_CLASS( joint_mode_controller::JointModeController, controller_interface::ControllerBase)
