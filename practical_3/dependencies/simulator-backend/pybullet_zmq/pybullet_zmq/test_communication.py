import time

import zmq
from network_interfaces.zmq import network
from pybullet_simulation import Robot
from pybullet_simulation import Simulation


def main():
    context = zmq.Context(1)
    subscriber_address = "0.0.0.0:1602"
    publisher_address = "0.0.0.0:1601"
    command_subscriber, state_publisher = network.configure_sockets(context, subscriber_address, publisher_address)

    desired_frequency = 500.0

    simulation = Simulation(gui=False)

    robot = Robot(simulation.uid, "panda", "/home/ros2/robot_descriptions/franka_panda_description/urdf/panda_arm.urdf")

    start = time.time()
    k = 0
    while simulation.is_alive():
        now = time.time()
        simulation.step()

        ee_state = robot.get_ee_link_state()
        joint_state = robot.get_joint_state()
        state = network.StateMessage(ee_state, joint_state)
        network.send_state(state, state_publisher)

        command = network.receive_command(command_subscriber)
        if command:
            print("received command")
            print(command)

        elapsed = time.time() - now
        sleep_time = (1. / desired_frequency) - elapsed
        if sleep_time > 0.0:
            time.sleep(sleep_time)
        k = k + 1
        print("Average rate: ", k / (time.time() - start))


if __name__ == "__main__":
    main()
