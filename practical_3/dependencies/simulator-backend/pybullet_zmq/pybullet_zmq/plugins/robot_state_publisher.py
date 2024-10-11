from network_interfaces.zmq import network
from state_representation import Parameter, ParameterType


class RobotStatePublisher:
    """
    Query robot state and publish it with the ZMQ socket.
    """

    def __init__(self, zmq_context, pybullet, robot, **kwargs):
        """
        Constructor of the RobotStatePublisher plugin.

        :param zmq_context: ZMQ context to create publisher
        :param pybullet: Imported pybullet library
        :param robot: Robot object
        :type zmq_context: zmq.Context
        :type pybullet: types.ModuleType
        :type robot: pybullet_simulation.Robot
        """
        self._pb = pybullet
        self._robot = robot
        self._publisher = network.configure_publisher(zmq_context, str(kwargs["URI"]), False)

    def execute(self):
        """
        Execution function of the plugin.
        """
        joint_state = self._robot.get_joint_state()
        mass = Parameter(self._robot.name + "_mass", self._robot.get_inertia(joint_state.get_positions()),
                         ParameterType.MATRIX)
        state = network.StateMessage(self._robot.get_ee_link_state(), joint_state,
                                     self._robot.get_jacobian(joint_state.get_positions()), mass)
        network.send_state(state, self._publisher)
