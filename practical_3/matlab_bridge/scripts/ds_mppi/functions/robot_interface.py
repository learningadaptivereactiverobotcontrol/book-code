import zmq

from network_interfaces.zmq import network


class RobotInterface(object):
    def __init__(self, state_uri, command_uri):
        self.__context = zmq.Context(1)
        self.__subscriber = network.configure_subscriber(self.__context, state_uri, True)
        self.__publisher = network.configure_publisher(self.__context, command_uri, True)
        self.state = network.StateMessage()

    def get_state(self):
        return network.receive_state(self.__subscriber)

    def send_command(self, command: network.CommandMessage()):
        network.send_command(command, self.__publisher)
