import argparse
import importlib
import os
import time

import yaml
import zmq
from pybullet_simulation import FuncExecManager
from pybullet_simulation import Robot
from pybullet_simulation import Simulation

class PyBulletZmqWrapper:
    """ZMQ wrapper class for pybullet simulator"""

    def __init__(self, config_file="franka_config.yaml"):
        """
        Constructor of the PyBulletZmqWrapper class.

        :param config_file: Relative or absolute path to the configuration file
        :type config_file: str
        """
        assert isinstance(config_file, str), "[PyBulletZmqWrapper::init] Argument 'config_file' has an incorrect type."
        if not os.path.isfile(config_file):
            script_dir = os.path.dirname(os.path.realpath(__file__))
            config_file = os.path.join(script_dir, "config", config_file)
        if not os.path.isfile(config_file):
            raise ValueError("[PyBulletZmqWrapper::init] Configuration file not found!")

        self._pb = importlib.import_module("pybullet")
        self._zmq_context = zmq.Context(1)
        self._simulation = Simulation(log_info=print, log_warn=self.print_warn, log_err=self.print_err)

        with open(config_file, "r") as stream:
            robot_config = yaml.safe_load(stream)
        self._parallel_plugin_execution = robot_config["parallel_plugin_execution"]
        self._loop_rate = robot_config["loop_rate"]

        robot_names = robot_config["robots"].keys()
        self._robots = {}
        self._plugins = []
        for robot_name in robot_names:
            robot = Robot(sim_uid=self._simulation.uid, name=robot_name,
                          urdf_path=robot_config["robots"][robot_name]["urdf_path"],
                          fixed_base=robot_config["robots"][robot_name]["fixed_base"],
                          use_inertia_from_file=robot_config["robots"][robot_name]["use_inertia_from_file"],
                          log_info=print, log_warn=self.print_warn, log_err=self.print_err)
            self._robots[robot_name] = robot
            
            # import plugins dynamically
            for plugin in robot_config["robots"][robot_name]["plugins"]:
                plugin_ = plugin.copy()
                module_ = plugin_.pop("module")
                class_ = plugin_.pop("class")
                params_ = plugin_.copy()
                print("[PyBulletZmqWrapper::init] Loading plugin: {} class from {} for robot {}".format(class_, module_,
                                                                                                        robot_name))
                # create object of the imported file class
                obj = getattr(importlib.import_module(module_), class_)(self._zmq_context, self._pb, robot, **params_)
                # store objects in member variable for future use
                self._plugins.append(obj)

        self._pb.setGravity(0, 0, -9.81)
        self._pb.loadURDF('plane.urdf')

        print("[PyBulletZmqWrapper::init] PyBullet ZMQ wrapper initialized.")

    def _start_pybullet_zmq_wrapper_sequential(self, loop_rate):
        """
        This function is deprecated, the use of parallel plugin execution is recommended.
        """
        print("[PyBulletZmqWrapper::start_pybullet_zmq_wrapper_sequential] Starting sequential execution of plugins.")
        while self._simulation.is_alive():
            if not self._simulation.is_paused():
                now = time.time()
                for task in self._plugins:
                    task.execute()
                # perform all the actions in a single forward dynamics simulation step
                self._simulation.step()
                elapsed = time.time() - now
                sleep_time = (1. / loop_rate) - elapsed
                if sleep_time > 0.0:
                    time.sleep(sleep_time)

    def _start_pybullet_zmq_wrapper_parallel(self, loop_rate):
        """
        Execute plugins in parallel, however watch their execution time and warn if they exceed the loop rate.

        :param loop_rate: Loop rate of parallel execution
        :type loop_rate: float
        """
        exec_manager_obj = FuncExecManager(self._plugins, lambda: not self._simulation.is_alive(),
                                           self._simulation.step, self._simulation.is_paused, log_info=print,
                                           log_warn=self.print_warn, log_debug=lambda x: None)
        print("[PyBulletZmqWrapper::start_pybullet_zmq_wrapper_parallel] Starting parallel execution of plugins.")
        # start parallel execution of all "execute" class methods in a synchronous way
        exec_manager_obj.start_synchronous_execution(loop_rate=loop_rate)

    def start_pybullet_zmq_wrapper(self):
        """
        Start execution of plugins.
        """
        if self._parallel_plugin_execution:
            self._start_pybullet_zmq_wrapper_parallel(self._loop_rate)
        else:
            self._start_pybullet_zmq_wrapper_sequential(self._loop_rate)

    @staticmethod
    def print_warn(msg):
        """
        Print message in yellow.

        :param msg: Message to print
        :type msg: str
        """
        print("\033[33m" + msg + "\033[0m")

    @staticmethod
    def print_err(msg):
        """
        Print message in red.

        :param msg: Message to print
        :type msg: str
        """
        print("\033[31m" + msg + "\033[0m")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Start PyBullet simulation with ZMQ interface.")
    parser.add_argument("-c", "--config-file", type=str, default="franka_config.yaml",
                        help="Configuration file for the simulation (default: franka_config.yaml)")
    args = parser.parse_args()

    wrapper = PyBulletZmqWrapper(args.config_file)
    wrapper.start_pybullet_zmq_wrapper()
