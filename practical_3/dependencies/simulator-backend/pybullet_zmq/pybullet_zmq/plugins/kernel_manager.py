from network_interfaces.control_type import ControlType
from network_interfaces.zmq import network
import zmq


class KernelManager:
    """
    Display the obstacles for the robot as either spheres, cylinders or 'planes'
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

        self.kernels = []
        self.color = [0, 0.4470, 0.7410]
        self.width = 3

    def zmq_try_recv(self):
        try:
            msg_dict = self._subscriber.recv_pyobj(flags=zmq.DONTWAIT)
            # print("Received ZMQ message")
            return msg_dict
        except zmq.Again as e:
            # No message received
            return None
        except Exception as e:
            print(f"Error receiving ZMQ message: {e}")
            return None
             

    def create_line(self, positions):
        #draw a line
        tmp_arr = []
        for i in range(len(positions)-1):
            pos_current = positions[i]
            pos_next = positions[i+1]
            lineId = self._pb.addUserDebugLine(pos_current, pos_next, self.color, self.width, lifeTime=0)
            tmp_arr.append(lineId)
        self.kernels.append(tmp_arr)

    def initialize_kernels(self, kernel_array):
        for kernel in kernel_array:
            self.create_line(kernel)

    def delete_lines(self, line_arr):
        for line in line_arr:
            self._pb.removeUserDebugItem(line)

    def delete_kernels(self):
        self._pb.removeAllUserDebugItems()
        # for kernel in self.kernels:
        #     self.delete_lines(kernel)
        self.kernels = []

    def update_kernels(self, policy_data, key='kernel_fk'):

        kernel_array = policy_data[key]
        if len(self.kernels) < len(kernel_array):
            for i in range(len(self.kernels), len(kernel_array)):
                self.create_line(kernel_array[i])
        elif len(kernel_array) == 0:
            self.delete_kernels()
        elif len(self.kernels) > len(kernel_array):
            print("Number of kernels do not match")
            self.delete_kernels()
            self.initialize_kernels(kernel_array)


    def execute(self):
        """
        Execution function of the plugin.
        """

        policy_data = self.zmq_try_recv()
        
        if policy_data is not None:
            self.update_kernels(policy_data)
        else:
            pass