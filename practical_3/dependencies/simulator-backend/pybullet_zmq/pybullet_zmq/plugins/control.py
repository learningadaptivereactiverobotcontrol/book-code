from network_interfaces.control_type import ControlType
from network_interfaces.zmq import network


class Control:
    """
    Control the robot in position, velocity, and torque mode.
    """

    def __init__(self, zmq_context, pybullet, robot, **kwargs):
        """
        Constructor of the Control plugin.

        :param zmq_context: ZMQ context to create publisher
        :param pybullet: Imported pybullet library
        :param robot: Robot object
        :type zmq_context: zmq.Context
        :type pybullet: types.ModuleType
        :type robot: pybullet_simulation.Robot
        """
        self._pb = pybullet
        self._robot = robot
        self._subscriber = network.configure_subscriber(zmq_context, str(kwargs["URI"]), False)

        self._control_params = {"bodyUniqueId": self._robot.id, "jointIndices": self._robot.joint_indices}
        self._last_command_type = ControlType.UNDEFINED
        self._last_torque_command = [0] * self._robot.nb_joints
        self._force_commands = [100] * self._robot.nb_joints

    def execute(self):
        """
        Execution function of the plugin.
        """
        command = network.receive_command(self._subscriber)
        if command:
            if len(set(command.control_type)) > 1:
                raise ValueError("Different control types per joint are currently not allowed. "
                                 "Make sure all the joints have the same control type.")
            elif command.control_type[0] == ControlType.POSITION.value:
                self._robot.set_torque_control(False)
                self._control_params["controlMode"] = self._pb.POSITION_CONTROL
                self._control_params["targetPositions"] = command.joint_state.get_positions()
                self._control_params["forces"] = self._force_commands
            elif command.control_type[0] == ControlType.VELOCITY.value:
                self._robot.set_torque_control(False)
                self._control_params["controlMode"] = self._pb.VELOCITY_CONTROL
                self._control_params["targetVelocities"] = command.joint_state.get_velocities()
                self._control_params["forces"] = self._force_commands
            elif command.control_type[0] == ControlType.EFFORT.value:
                if self._last_command_type != ControlType.EFFORT:
                    self._pb.setJointMotorControlArray(self._robot.id, self._robot.joint_indices,
                                                       self._pb.VELOCITY_CONTROL,
                                                       forces=[0] * self._robot.nb_joints)
                self._robot.set_torque_control(True)
                self._last_torque_command = command.joint_state.get_torques()
            else:
                self._robot.set_torque_control(False)
                if "controlMode" in self._control_params.keys():
                    self._control_params.pop("controlMode")

        if self._robot.is_torque_controlled:
            self._control_params["controlMode"] = self._pb.TORQUE_CONTROL
            torques = self._robot.compensate_gravity(feed_forward=self._last_torque_command)
            self._control_params["forces"] = torques
            self._robot.set_applied_motor_torques(torques)

        if "controlMode" in self._control_params.keys():
            # print(f"Sending command to robot : {self._control_params}")
            self._pb.setJointMotorControlArray(**self._control_params)
