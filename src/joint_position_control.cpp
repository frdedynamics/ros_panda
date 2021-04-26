#include <mojo_controllers/joint_position_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace panda_controllers {

double JOINT_POSITION_COMMAND[7];
ros::Time JOINT_POSITION_COMMAND_STAMP_CURRENT;
ros::Time JOINT_POSITION_COMMAND_STAMP_PREVIOUS;

void cmdCallback(const ros_panda::JointCommandPosition::ConstPtr& msg){

  JOINT_POSITION_COMMAND_STAMP_CURRENT = ros::Time::now();
  for (int i=0; i< 7; i++)
  {
      JOINT_POSITION_COMMAND[i] = msg->values[i];
  }
  JOINT_POSITION_COMMAND_STAMP_PREVIOUS = JOINT_POSITION_COMMAND_STAMP_CURRENT;

}

bool JointPositionController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointPositionController: Error getting position joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointPositionController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointPositionController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "JointPositionController: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
  for (size_t i = 0; i < q_start.size(); i++) {
    if (std::abs(position_joint_handles_[i].getPosition() - q_start[i]) > 0.1) {
      ROS_ERROR_STREAM(
          "JointPositionController: Robot is not in the expected starting position for "
          "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
          "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first. Balls, I forgot this part..");
      return false;
    }
  }

  joint_command_position_sub = ros_nh.subscribe("panda_joint_cmd_pos", 1, cmdCallback);

  return true;
}

void JointPositionController::starting(const ros::Time& /* time */) {
  
  for (size_t i = 0; i < 7; ++i) {
    JOINT_POSITION_COMMAND[i] = position_joint_handles_[i].getPosition();
  }

}

void JointPositionController::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {
  
  if (std::abs(position_joint_handles_[i].getPosition() - JOINT_POSITION_COMMAND[i]) > 0.1) {
      ROS_ERROR_STREAM(
          "Balls, do stuff here..");
      return false;
      }

  for (size_t i = 0; i < 7; ++i) {   
    position_joint_handles_[i].setCommand(JOINT_POSITION_COMMAND[i]);
  }

}

}  // namespace panda_controllers

PLUGINLIB_EXPORT_CLASS(panda_controllers::JointPositionExampleController,
                       controller_interface::ControllerBase)
