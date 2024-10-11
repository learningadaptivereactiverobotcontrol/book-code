import os

import pybullet as pb
import pybullet_data as pb_data


class Simulation(object):
    """
    The Simulation class creates the PyBullet physics server and optionally a GUI.
    Available methods (for usage, see documentation at function definition):
        - uid
        - is_alive
        - is_paused
        - step
        - add_search_path
        - add_pybullet_path
    """

    def __init__(self, gui=True, gui_options="", hide_menu=True, start_paused=False, log_info=print, log_warn=print, log_err=print):
        """
        Constructor of the Simulation class. This class creates the PyBullet server / GUI and steps the simulation.

        :param gui: Launch a PyBullet GUI
        :param gui_options: Additional options for the PyBullet GUI
        :param start_paused: Start simulation paused
        :param log_info: Function handle for info logging
        :param log_warn: Function handle for warning logging
        :param log_err: Function handle for error logging
        :type gui: bool
        :type gui_options: str
        :type start_paused: str
        :type log_info: T
        :type log_warn: T
        :type log_err: T
        """
        assert isinstance(gui, bool), "[Simulation::init] Argument 'gui' has an incorrect type."
        assert isinstance(gui_options, str), "[Simulation::init] Argument 'gui_options' has an incorrect type."
        assert isinstance(start_paused, bool), "[Simulation::init] Argument 'start_paused' has an incorrect type."

        self._log_info = log_info
        self._log_warn = log_warn
        self._log_err = log_err

        if gui:
            self._log_info("[Simulation::init] Running PyBullet with GUI")
            self._log_info("-------------------------")
            self._uid = pb.connect(pb.GUI, options=gui_options)
            if hide_menu: pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)  # Hides menu in GUI
            pb.resetDebugVisualizerCamera(2.5, 50.0, -35.0, [0.0, 0.0, 0.0]) ## Set camera view 
        else:
            self._log_info("[Simulation::init] Running PyBullet without GUI")
            self._log_info("-------------------------")
            self._uid = pb.connect(pb.DIRECT)

        self._simulation_paused = start_paused

        # add path to PyBullet models (for simple models like the plane and a table)
        self.add_pybullet_path()


    @property
    def uid(self):
        """
        Get UID of physics server.

        :rtype: int
        """
        return self._uid

    def is_alive(self):
        """
        Check if the physics server is still connected.

        :rtype: bool
        """
        return pb.isConnected(self._uid)

    def is_paused(self):
        """
        Check if the simulation if paused.

        :rtype: bool
        """
        return self._simulation_paused

    def step(self):
        """
        Step the simulation.
        """
        pb.stepSimulation(self._uid)

    def add_search_path(self, path):
        """
        Add the specified directory (absolute path) to PyBullet's search path for adding models from the path.

        :param path: The absolute path to the desired directory
        :type path: str
        :return: Boolean if action was successful
        :rtype: bool
        """
        assert isinstance(path, str), "[Simulation::add_search_path] Parameter 'path' has an incorrect type."
        if os.path.isdir(path):
            pb.setAdditionalSearchPath(path)
            self._log_info("[Simulation::add_search_path] Added {} to PyBullet path.".format(path))
            return True
        else:
            self._log_err(
                "[Simulation::add_search_path] Error adding to PyBullet path! {} not a directory.".format(path))
            return False

    @staticmethod
    def add_pybullet_path():
        """
        Adds PyBullets in-built models path to the PyBullet path for easily retrieving the models.
        """
        pb.setAdditionalSearchPath(pb_data.getDataPath())
