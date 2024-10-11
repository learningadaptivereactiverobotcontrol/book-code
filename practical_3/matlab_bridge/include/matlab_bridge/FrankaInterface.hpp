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

#include <network_interfaces/zmq/network.h>

namespace matlab_bridge {

enum InterfaceType {
  FRANKA_PAPA_16, FRANKA_QUEBEC_17
};

class FrankaInterface {
public:
  explicit FrankaInterface(InterfaceType type);

  bool receive(network_interfaces::zmq::StateMessage& state_message, bool wait = false);

  bool send(network_interfaces::zmq::CommandMessage& command_message);

private:
  zmq::context_t context_;
  zmq::socket_t publisher_;
  zmq::socket_t subscriber_;
};
}// namespace matlab_bridge