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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace matlab_bridge {

class RobotStateSubscriber : public rclcpp::Node {
public:
  RobotStateSubscriber() : Node("robot_state_subscriber") {
    subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/robot_state", 10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) { this->callback(msg, *this->get_clock()); }
    );
  }

private:
  void callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg, rclcpp::Clock& clock) const {
    RCLCPP_INFO_THROTTLE(this->get_logger(), clock, 1500, "Robot state: [%.2f, %.2f, %.2f]", msg->data.at(1),
                                msg->data.at(2), msg->data.at(3));
  }

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
};
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<matlab_bridge::RobotStateSubscriber>());
  rclcpp::shutdown();
  return 0;
}