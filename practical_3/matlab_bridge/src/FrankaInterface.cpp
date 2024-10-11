
#include "matlab_bridge/FrankaInterface.hpp"

namespace matlab_bridge {

FrankaInterface::FrankaInterface(InterfaceType type) : context_(1) {
  std::string ip, state_port, command_port;
  switch (type) {
    case FRANKA_PAPA_16:
      ip = "0.0.0.0";
      state_port = "1601";
      command_port = "1602";
      break;
    case FRANKA_QUEBEC_17:
      ip = "0.0.0.0";
      state_port = "1701";
      command_port = "1702";
      break;
  }
  network_interfaces::zmq::configure_publisher(context_, publisher_, ip + ":" + command_port, true);
  network_interfaces::zmq::configure_subscriber(context_, subscriber_, ip + ":" + state_port, true);
}

bool FrankaInterface::receive(network_interfaces::zmq::StateMessage& state_message, bool wait) {
  if (this->context_.handle() == nullptr) {
    return false;
  }
  if(network_interfaces::zmq::receive(state_message, this->subscriber_, wait)) {
    // std::cout << state_message <<std::endl;
    return true;
  }
  return false;
}

bool FrankaInterface::send(network_interfaces::zmq::CommandMessage& command_message) {
  if (this->context_.handle() == nullptr) {
    return false;
  }
  if(network_interfaces::zmq::send(command_message, this->publisher_)) {
    // std::cout << command_message <<std::endl;
    return true;
  }
  return false;
}
}// namespace matlab_bridge