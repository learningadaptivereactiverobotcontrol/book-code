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

#include "matlab_bridge/MatlabBridge.hpp"
#include <modulo_core/translators/message_readers.hpp>
#include <modulo_core/translators/message_writers.hpp>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

const int NB_JOINTS = 7;

namespace matlab_bridge
{

  MatlabBridge::MatlabBridge(const std::string &node_name, matlab_bridge::InterfaceType robot_type) : rclcpp::Node(node_name),
                                                                                                      control_gains({10.0, 5.0, 8.0, 6.0}),
                                                                                                      max_twist({0.25, 0.25}),
                                                                                                      twist_command_(std::make_shared<state_representation::CartesianTwist>("panda_ee", "panda_base")),
                                                                                                      attractor_(std::make_shared<Eigen::Vector<double, 3>>(Eigen::Vector<double, 3>::Zero()))

  {
    this->stateMachine = FSMType::IDLE;

    this->command_message_.control_type = std::vector<int>{static_cast<int>(network_interfaces::control_type_t::EFFORT)};

    // An impedance controller to transform the desired velocity to a torque command
    this->nb_joints = 9;
    this->controller_ = controllers::JointControllerFactory::create_controller(controllers::CONTROLLER_TYPE::VELOCITY_IMPEDANCE, this->nb_joints);

    // ROS param for Null Space control
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "Activate/Deactivate Null Space Projected Control";
    bool activate_ns = true;
    this->declare_parameter("activate_ns", activate_ns, param_desc);

    // ROS params for compliant twist controller
    param_desc.description = "Scaling factor of the principle damping gains. Range : [0.0 ; 10.0]";
    this->declare_parameter("principle_damping", this->control_gains.at(0), param_desc);

    param_desc.description = "Scaling factor of the orthogonal damping gains. Range : [0.0 ; 10.0]";
    this->declare_parameter("orthogonal_damping", this->control_gains.at(1), param_desc);

    // ROS param for FREEZE mode
    param_desc.description = "Activate/Deactivate Freeze mode";
    bool freeze = false;
    this->declare_parameter("freeze_mode", freeze, param_desc);

    // ROS param for reset robot
    param_desc.description = "Resets robot joint configuration";
    bool reset = false;
    this->declare_parameter("reset_robot", reset, param_desc);

    // ROS param for attractor
    param_desc.description = "Sets attractor";
    // <std::vector<double>> attractor = {0, 0, 0};
    // this->declare_parameter("attractor", attractor, param_desc);
    this->declare_parameter<std::vector<double>>("attractor", {0.0, 0.0, 0.0}, param_desc);


    this->null_positions_ << 0.0, 0.1479, 0.0, -2.2249, 0.0, 2.3723, 0.7, 0.0, 0.0;

    // A controller to control the position of the end-effector 
    this->position_controller_ =
        controllers::CartesianControllerFactory::create_controller(controllers::CONTROLLER_TYPE::COMPLIANT_TWIST);

    // We are only using this controller for position, thus with only linear part activated
    this->position_controller_->set_parameter_value("linear_principle_damping", this->control_gains.at(0));
    this->position_controller_->set_parameter_value("linear_orthogonal_damping", this->control_gains.at(1));
    this->position_controller_->set_parameter_value("angular_stiffness", 0.0);
    this->position_controller_->set_parameter_value("angular_damping", 0.0);


    // A controller to control the orientation of the end-effector (using attractor_ds_)
    this->orientation_controller_ =
        controllers::CartesianControllerFactory::create_controller(controllers::CONTROLLER_TYPE::COMPLIANT_TWIST);

    // We are only using this controller controller in teach mode, thus with only angular part activated
    this->orientation_controller_->set_parameter_value("linear_principle_damping", 0.0);
    this->orientation_controller_->set_parameter_value("linear_orthogonal_damping", 0.0);
    this->orientation_controller_->set_parameter_value("angular_stiffness", this->control_gains.at(2));
    this->orientation_controller_->set_parameter_value("angular_damping", this->control_gains.at(3));

    // configure dynamical system for uncontrolled dimensions, this only affects the desired angular
    // velocity, not the linear, which is given by the learned DS
    this->attractor_ds_ = std::make_shared<dynamical_systems::PointAttractor<state_representation::CartesianState>>();
    this->attractor_ds_->set_parameter_value(
        "attractor", state_representation::CartesianPose(
                         "attractor", Eigen::Quaterniond(0, 1, 0, 0), "panda_base"));
    // this->attractor_ds_->set_parameter_value("gain", std::vector<double>{0, 0, 0, 10, 10, 10});
    this->attractor_ds_->set_parameter_value("gain", std::vector<double>{0, 0, 0, 5, 5, 5});

    // create Franka interface
    this->franka_ = std::make_shared<matlab_bridge::FrankaInterface>(robot_type);
    // check if the connection to the robot is established
    if (!this->franka_->receive(this->state_message_, true))
    {
      RCLCPP_FATAL(this->get_logger(), "Could not receive the robot state");
      rclcpp::shutdown();
    }

    std::vector<std::string> joint_names = {
        "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7", "panda_finger_joint1", "panda_finger_joint2"};
    this->command_message_.joint_state = state_representation::JointState::Zero("panda", joint_names);
    this->jointVelocity = state_representation::JointVelocities::Zero("panda", joint_names);
    this->cartesianTwist = state_representation::CartesianTwist::Zero("panda_twist", "panda_base");

    // receive and publish robot state
    this->state_receive_timer_ = this->create_wall_timer(2ms, [this]
                                                         { this->franka_->receive(this->state_message_); });
    this->state_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/robot_state", 10);
    this->joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    // subscribe to twist command
    this->twist_subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/command_twist", 10, [this](const std::shared_ptr<geometry_msgs::msg::TwistStamped> msg)
        { modulo_core::translators::read_message(*this->twist_command_, *msg); }

    );

    // control loop
    this->control_timer_ = this->create_wall_timer(1ms, [this]
                                                   { this->run(); });

    // Publisher for the end effector trajectory
    this->path_msg_ = std::make_shared<nav_msgs::msg::Path>();
    this->path_msg_->header.frame_id = "world";
    this->path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/robot_trajectory", 10);
    this->path_timer_ = this->create_wall_timer(100ms, [this]
                                                { this->record_path(); });
    publish_once = true;

    // Subscriber for attractor 
    this->attractor_subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/attractor_pos", 10, [this](const std::shared_ptr<geometry_msgs::msg::Vector3> msg)
      {
        if (this->attractor_ != nullptr)
        {
            (*this->attractor_)(0) = msg->x;
            (*this->attractor_)(1) = msg->y;
            (*this->attractor_)(2) = msg->z;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Attractor pointer is null");
        }
      }
);
  }

  void MatlabBridge::publish_state()
  {
    std::vector<double> data(7);
    data.at(0) = this->get_clock()->now().seconds() + 1e-9 * this->get_clock()->now().nanoseconds();
    for (std::size_t i = 0; i < 3; ++i)
    {
      data.at(i + 1) = this->state_message_.ee_state.get_position()(i);
      data.at(i + 4) = this->state_message_.ee_state.get_linear_velocity()(i);
    }
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = data;
    this->state_publisher_->publish(msg);
  }

   void MatlabBridge::publish_joint_state()
  {
    auto msg = sensor_msgs::msg::JointState();
    modulo_core::translators::write_message(
        msg, this->state_message_.joint_state, this->get_clock()->now());
    this->joint_state_publisher_->publish(msg);
  }

  void MatlabBridge::record_path()
  {
    // auto ee_vel = (*this->twist_command_).get_linear_velocity().norm(); //this->state_message_.ee_state.get_linear_velocity().norm();
    // auto vel_threshold = 0.1;

    // // std::vector<double> attractor = this->get_parameter("attractor").as_double_array();
    // Eigen::Vector<double, 3> attractor = (*this->attractor_);
    // Eigen::Vector3d attractor_vec(attractor[0], attractor[1], attractor[2]);

    double pos_diff = (*this->attractor_ - this->state_message_.ee_state.get_position()).norm();
    double pos_threshold = 0.02;
    
    if (this->stateMachine == FSMType::CONTROL)
    {
      // if(ee_vel > vel_threshold){ // far from target, record path
      if(pos_diff > pos_threshold){ // far from target, record path
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = this->get_clock()->now();
        pose.header.frame_id = "world";
        pose.pose.position.x = this->state_message_.ee_state.get_position().x();
        pose.pose.position.y = this->state_message_.ee_state.get_position().y();
        pose.pose.position.z = this->state_message_.ee_state.get_position().z();
        // pose.pose.orientation.x = this->state_message_.ee_state.get_orientation().x();
        // pose.pose.orientation.y = this->state_message_.ee_state.get_orientation().y();
        // pose.pose.orientation.z = this->state_message_.ee_state.get_orientation().z();
        // pose.pose.orientation.w = this->state_message_.ee_state.get_orientation().w();
        this->path_msg_->header.stamp = this->get_clock()->now();
        this->path_msg_->poses.push_back(pose);
      }
      // else if(ee_vel <= vel_threshold && publish_once){ /// close to target, publish path
      else if(pos_diff <= pos_threshold && publish_once){ /// close to target, publish path
        this->path_publisher_->publish(*this->path_msg_);
        publish_once = false;
      }
    }
    else{ // reset path and bool 
      clear_path();
      publish_once = true;
    }
    
  }

  void MatlabBridge::clear_path()
  {

    this->path_msg_->header.stamp = this->get_clock()->now();
    this->path_msg_->poses.clear();
    this->path_publisher_->publish(*this->path_msg_);
  }

  void MatlabBridge::run()
  {

    state_representation::CartesianTwist twist_command = this->attractor_ds_->evaluate(this->state_message_.ee_state);
    if (!this->twist_command_->is_empty())
    {
      twist_command += *this->twist_command_;

      switch (this->stateMachine)
      {

        case FSMType::IDLE:
        {         
          if (!((*this->twist_command_).get_linear_velocity().isZero()))
          {
            RCLCPP_INFO(this->get_logger(), "---- STARTING CONTROL -----\n");
            this->stateMachine = FSMType::CONTROL;
          }
          if(this->get_parameter("freeze_mode").as_bool())
          {
            RCLCPP_INFO(this->get_logger(), "---- STARTING FREEZE MODE -----\n");
            this->stateMachine = FSMType::FREEZE;
          }
          break;
        }

        case FSMType::CONTROL:
        {
          if ((*this->twist_command_).get_linear_velocity().isZero())
          {        
            RCLCPP_INFO(this->get_logger(), "---- STOPPED CONTROL -----\n");
            this->stateMachine = FSMType::IDLE;
          }
          break;
        }

        case FSMType::FREEZE:
        {
          if(!this->get_parameter("freeze_mode").as_bool() || !((*this->twist_command_).get_linear_velocity().isZero())){
            RCLCPP_INFO(this->get_logger(), "---- STOPPING FREEZE MODE -----\n");
            this->set_parameter(rclcpp::Parameter("freeze_mode", false));
            this->command_message_.control_type = std::vector<int>{static_cast<int>(network_interfaces::control_type_t::EFFORT)};
            this->stateMachine = FSMType::IDLE;
          }
          break;
        }

      }
    }
    // starting freeze mode even if not controlling yet
    else if(this->get_parameter("freeze_mode").as_bool() && this->stateMachine == FSMType::IDLE)
    {
      RCLCPP_INFO(this->get_logger(), "---- STARTING FREEZE MODE -----\n");
      this->stateMachine = FSMType::FREEZE;
    }


    twist_command.clamp(this->max_twist.at(0), this->max_twist.at(1));

    // --------------- Velocity filtering ---------------
    double alpha = 0.2;
    this->cartesianTwist = alpha * state_representation::CartesianTwist(this->state_message_.ee_state) + (1 - alpha) * this->cartesianTwist;

    // --------------- Computing total torque ---------------
    if (this->stateMachine == FSMType::CONTROL)
    { 
      // Update gains 
      this->position_controller_->set_parameter_value("linear_principle_damping", this->get_parameter("principle_damping").as_double());
      this->position_controller_->set_parameter_value("linear_orthogonal_damping", this->get_parameter("orthogonal_damping").as_double());

      // 2024 test with compliant twist
      this->command_message_.joint_state = this->position_controller_->compute_command(
          twist_command, this->cartesianTwist, this->state_message_.jacobian);

      // Clamp torques
      this->command_message_.joint_state.clamp_state_variable(20.0, state_representation::JointStateVariable::TORQUES);

    }
    else
    {
      // Passive control
      this->command_message_.joint_state = this->orientation_controller_->compute_command(
          twist_command, this->cartesianTwist, this->state_message_.jacobian);
    }

    // --------------- Null space control ---------------
    bool activate_ns = this->get_parameter("activate_ns").as_bool();

    if (activate_ns){
      
      // Eigen::Vector<double, 9> null_positions_{0.0, 0.1479, 0.0, -2.2249, 0.0, 2.3723, 0.7, 0.0, 0.0};
      Eigen::Vector<double, 9> null_command;

      // Eigen::Vector<double, 9> null_stiffness{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
      // Eigen::Vector<double, 9> null_damping{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
      double null_stiffness = 1; //1 modify to be 9d -> j1 should be lower i think
      double null_damping = 2; //2

      state_representation::JointState null_state = this->state_message_.joint_state;
      null_state.set_positions(this->null_positions_);

      // Using jacobian to get null space projector
      auto jacobianMatrix = this->state_message_.jacobian.data();
      auto jacobianDampedInverseMatrix = jacobianMatrix.transpose() * (jacobianMatrix * jacobianMatrix.transpose() + 0.01 * Eigen::MatrixXd::Identity(6, 6)).inverse();
      auto null_space_projector = Eigen::MatrixXd::Identity(this->state_message_.joint_state.get_size(), this->state_message_.joint_state.get_size()) - jacobianDampedInverseMatrix * jacobianMatrix;
      
      auto null_error = null_stiffness*(null_state.get_positions() - this->state_message_.joint_state.get_positions()) - null_damping*this->state_message_.joint_state.get_velocities();
      null_command = null_space_projector * null_error;

      // Apply null space correction
      this->command_message_.joint_state.set_torques(this->command_message_.joint_state.get_torques() + null_command);

    }

    // ----------------------FREEZE MODE -----------------------
    if (this->stateMachine == FSMType::FREEZE){
      // Set position to current position
      this->command_message_.joint_state = this->state_message_.joint_state;
      this->command_message_.control_type = std::vector<int>{static_cast<int>(network_interfaces::control_type_t::POSITION)};
    }

    // ----------------------RESET ROBOT ----------------------
    if(this->get_parameter("reset_robot").as_bool())
    {
      // Set position to NS configuration
      this->command_message_.joint_state = this->state_message_.joint_state;
      this->command_message_.joint_state.set_positions(this->null_positions_);
      this->command_message_.control_type = std::vector<int>{static_cast<int>(network_interfaces::control_type_t::POSITION)};

      // Deactivate when close enough 
      double distance = (this->state_message_.joint_state.get_positions() - this->null_positions_).norm();
      double threshold = 0.01;
      if (distance < threshold && this->state_message_.joint_state.get_velocities().norm() < threshold) {
        RCLCPP_INFO(this->get_logger(), "---- ROBOT RESET -----\n");
        this->set_parameter(rclcpp::Parameter("reset_robot", false));
        this->command_message_.control_type = std::vector<int>{static_cast<int>(network_interfaces::control_type_t::EFFORT)};
      }
    }

    this->franka_->send(this->command_message_);

    this->publish_state();
    this->publish_joint_state();

  }

}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<matlab_bridge::MatlabBridge>("matlab_bridge", matlab_bridge::FRANKA_PAPA_16));
  rclcpp::shutdown();
  return 0;
}
