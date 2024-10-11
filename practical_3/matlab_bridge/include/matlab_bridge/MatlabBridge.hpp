////////////////////////////////////////////////////////////////////////
//   Copyright (C) 2024 Learning Algorithms and Systems Laboratory, EPFL,
//     Switzerland
//    Author: Aude Billard
//    email: aude.billard@epfl.ch
//    website: lasa.epfl.ch
//     
//    Permission is granted to copy, distribute, and/or modify this program
//    under the terms of the GNU General Public License, version 3 or any
//   later version published by the Free Software Foundation.
// 
//    This program is distributed in the hope that it will be useful, but
//    WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
//    Public License for more details
////////////////////////////////////////////////////////////////////////

#pragma once

#include <rclcpp/node.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <controllers/ControllerFactory.hpp>
#include <dynamical_systems/PointAttractor.hpp>

#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "matlab_bridge/FrankaInterface.hpp"

enum FSMType{
    IDLE,
    CONTROL,
    FREEZE
};

namespace matlab_bridge {

class MatlabBridge : public rclcpp::Node {
public:
  MatlabBridge(const std::string& node_name, matlab_bridge::InterfaceType robot_type);

protected:
  // control gains for controller
  std::vector<double> control_gains;
  // maximal allowed desired twist
  std::vector<double> max_twist;

  FSMType stateMachine;

private:

  void publish_state();

  void publish_joint_state();

  void record_path();

  void publish_path();

  void clear_path();

  void run();

  int nb_joints;

  bool publish_once;

  std::shared_ptr<state_representation::CartesianTwist> twist_command_;
  std::shared_ptr<Eigen::Vector<double, 3>> attractor_;
  
  // Controller to track trajectories
  std::shared_ptr<controllers::IController<state_representation::JointState>> controller_;
  Eigen::MatrixXd stiffness;
  Eigen::MatrixXd damping;
  std::shared_ptr<controllers::IController<state_representation::CartesianState>> position_controller_;

  // controller and dynamical system for uncontrolled dimensions
  std::shared_ptr<controllers::IController<state_representation::CartesianState>> orientation_controller_;
  std::shared_ptr<dynamical_systems::PointAttractor<state_representation::CartesianState>> attractor_ds_;
  state_representation::JointVelocities jointVelocity;
  state_representation::CartesianTwist cartesianTwist;
  Eigen::Vector<double, 9> null_positions_;

  // robot interface
  std::shared_ptr<matlab_bridge::FrankaInterface> franka_;
  network_interfaces::zmq::CommandMessage command_message_;
  network_interfaces::zmq::StateMessage state_message_;
  std::shared_ptr<nav_msgs::msg::Path> path_msg_;
  

  // ROS2 publishers and timers
  std::shared_ptr<rclcpp::TimerBase> state_receive_timer_;
  std::shared_ptr<rclcpp::TimerBase> control_timer_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> state_publisher_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> joint_state_publisher_;
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::TwistStamped>> twist_subscription_;
  std::shared_ptr<rclcpp::TimerBase> path_timer_;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> path_publisher_;
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Vector3>> attractor_subscription_;
};
}// namespace matlab_bridge