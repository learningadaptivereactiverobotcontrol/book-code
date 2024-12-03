from network_interfaces.control_type import ControlType
from network_interfaces.zmq import network
import math
import zmq
# import torch
import numpy as np


class ObstacleManager:
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

        self.obstacles_id = []
        self.obstacles_scale = []
        self.obstacles_color = []
        self.matlab_id = []
        
        # params for predefined obstacles
        self.spheres_id = []
        # self.color = [.63, .07, .185, 1] ## red
        self.color = [.18, .545, .341, 1] ## seagreen
        self.predefined_obstacles()

    def zmq_try_recv(self):
        try:
            msg_dict = self._subscriber.recv_json(flags=zmq.DONTWAIT)
            # print("Received ZMQ message")
            return msg_dict
        except zmq.Again as e:
            # No message received
            return None
        except Exception as e:
            print(f"Error receiving ZMQ message: {e}")
            return None
    
    def predefined_obstacles(self):
        ########################################
        ### I-Shape centered in front of franka
        ########################################
        x_dist = 0.5
        y_width = 0.4
        z_0 = 0.1
        height = 0.75
        r = 0.05
        n_vertical = 20
        n_horizontal = 20

        # # small one
        # z_0 = 0.35
        # height = 0.4
        # y_width = 0.2
        # x_dist = 0.35

        top_left = np.array([x_dist, -y_width, z_0 + height, r])
        top_right = np.array([x_dist, y_width, z_0 + height, r])
        top_bar = top_left + np.linspace(0, 1, n_horizontal).reshape(-1, 1) * (top_right - top_left)
        bottom_bar = top_bar - np.array([0, 0, height, 0])

        top_mid = top_left + 0.5 * (top_right - top_left)
        bottom_mid = top_mid - np.array([0, 0, height, 0])
        middle_bar = bottom_mid + np.linspace(0, 1, n_vertical).reshape(-1, 1) * (top_mid - bottom_mid)
        tshape = np.vstack((top_bar, middle_bar, bottom_bar))

        ########################################
        ### Ring constrained
        ########################################
        center = np.array([0.55, 0, 0.6])
        radius = 0.2
        n_ring = 21
        ring = np.zeros((n_ring, 4))
        ring[:, 0] = center[0]
        ring[:, 1] = center[1] + radius * np.cos(np.linspace(0, 2 * np.pi, n_ring))
        ring[:, 2] = center[2] + radius * np.sin(np.linspace(0, 2 * np.pi, n_ring))
        ring[:, 3] = 0.03

        ########################################
        ### Line/Wall
        ########################################
        r = 0.05
        n_pts = 2
        length = max(1, 2 * n_pts - 2) * r
        z0 = 0.5
        x0 = 0.6
        y0 = 0
        posA = np.array([x0, y0, z0 + length, r])
        posB = posA + np.array([length, 0.0, 0.0, 0.0])
        line = posA + np.linspace(0, 1, n_pts).reshape(-1, 1) * (posB - posA)
        wall = line
        n_down = n_pts
        for sphere in line:
            sphere_down = sphere - np.array([0, 0, length, 0])
            line_down = sphere + np.linspace(0, 1, n_down).reshape(-1, 1) * (sphere_down - sphere)
            wall = np.vstack((wall, line_down))

        ########################################
        ### Dummy obstacle (commented out)
        ########################################
        # n_dummy = 1
        # dummy_obs = np.hstack((np.zeros((n_dummy, 3)) + 10, np.zeros((n_dummy, 1)) + 0.1))
        # obs = np.vstack((obs, dummy_obs))

        self.ring = ring
        self.tshape = tshape
        self.wall = wall
        self.line = line


    def create_sphere(self, position, radius, color):
        sphere = self._pb.createVisualShape(self._pb.GEOM_SPHERE,
                                           radius=radius,
                                           rgbaColor=color, specularColor=[0, 0, 0, 1])
        sphere = self._pb.createMultiBody(baseVisualShapeIndex=sphere,
                                         basePosition=position)
        self.spheres_id.append(sphere)

    def initialize_spheres(self, obstacle_array):
        for obstacle in obstacle_array:
            self.create_sphere(obstacle[0:3], obstacle[3], self.color)

    def delete_spheres(self):
        for sphere in self.spheres_id:
            self._pb.removeBody(sphere)
        self.spheres_id = []

    def update_spheres(self, obstacle_array):
        if (obstacle_array is not None) and (len(self.spheres_id) == len(obstacle_array)):
            for i, sphere in enumerate(self.spheres_id):
                self._pb.resetBasePositionAndOrientation(sphere,
                                                        obstacle_array[i, 0:3],
                                                        [1, 0, 0, 0])
        else:
            print("Number of spheres do not match")
            self.delete_spheres()
            self.initialize_spheres(obstacle_array)

    def delete_obstacle(self, idx):
        self._pb.removeBody(self.obstacles_id[idx])
        del self.obstacles_id[idx]
        del self.obstacles_scale[idx]
        del self.obstacles_color[idx]
        del self.matlab_id[idx]

    def add_obstacle(self, msg_dict):

        matlab_id = msg_dict['id']
        position = [msg_dict['pose']['position']['x'],  msg_dict['pose']['position']['y'],  msg_dict['pose']['position']['z']]
        radius = msg_dict['scale']['x']/2
        color =[msg_dict['color']['r'], msg_dict['color']['g'], msg_dict['color']['b'], msg_dict['color']['a']]
        object_type = msg_dict['type']

        if object_type == 2:
            # Add a simple sphere obstacle
            sphere_collision_shape = self._pb.createCollisionShape(shapeType=self._pb.GEOM_SPHERE, radius=radius)
            sphere_visual_shape = self._pb.createVisualShape(shapeType=self._pb.GEOM_SPHERE, radius=radius, rgbaColor=color)
            new_id = self._pb.createMultiBody(baseMass=0, baseCollisionShapeIndex=sphere_collision_shape, 
                                                baseVisualShapeIndex=sphere_visual_shape, basePosition=position)
        elif object_type == 3:
            # Create a cylinder
            visual_shape_id = self._pb.createVisualShape(shapeType=self._pb.GEOM_CYLINDER, radius=radius, length=2.5, 
                                                         rgbaColor=color)
            #collision_shape_id = self._pb.createCollisionShape(shapeType=self._pb.GEOM_CYLINDER, radius=radius, height=2.5)
            new_id = self._pb.createMultiBody(baseMass=1, baseCollisionShapeIndex=-1 , #collision_shape_id, 
                                        baseVisualShapeIndex=visual_shape_id, basePosition=position)

        elif object_type == 1:
            # Create a visual plane using a large flat box
            scale = [msg_dict['scale']['x'], msg_dict['scale']['y'], msg_dict['scale']['z']]
            visual_plane_shape = self._pb.createVisualShape(shapeType=self._pb.GEOM_BOX, 
                                                            halfExtents=scale,
                                                            rgbaColor=color)
            # Add the visual plane as a multibody with no mass
            new_id = self._pb.createMultiBody(baseMass=0, baseVisualShapeIndex=visual_plane_shape, basePosition=position)

        self.obstacles_id.append(new_id)
        self.obstacles_scale.append(radius)
        self.obstacles_color.append(color)
        self.matlab_id.append(matlab_id)

    def update_obstacle(self, msg_dict):

        matlab_id = msg_dict['id']
        new_position = [msg_dict['pose']['position']['x'],  msg_dict['pose']['position']['y'],  msg_dict['pose']['position']['z']]
        new_radius = msg_dict['scale']['x']/2
        new_color = [msg_dict['color']['r'], msg_dict['color']['g'], msg_dict['color']['b'], msg_dict['color']['a']]

        idx = self.matlab_id.index(matlab_id)
        obstacle_id = self.obstacles_id[idx]
        obstacle_scale = self.obstacles_scale[idx]
        obstacle_color = self.obstacles_color[idx]

        pos, _ = self._pb.getBasePositionAndOrientation(obstacle_id)
        
        if pos != new_position: ## update new_position
            self._pb.resetBasePositionAndOrientation(obstacle_id, new_position, [1, 0, 0, 0])

        if obstacle_scale != new_radius: ## update new size
            print("UPDATING SIZE")
            self.delete_obstacle(idx)
            self.add_obstacle(msg_dict)

        if new_color != obstacle_color:
            if new_color == [0,0,0,0]: ##deleted in matlab, remove here too
                print( "Removing Sphere")
                self.delete_obstacle(idx)
            else : ## selected obstacle
                print(f"updating color! {new_color}")
                self._pb.changeVisualShape(obstacle_id, linkIndex=-1, rgbaColor=new_color)
                self.obstacles_color[idx] = new_color

    def execute(self):
        """
        Execution function of the plugin.
        """

        ##TODO : make thing to toggle collision OR remove it entirely ?

        msg_dict = self.zmq_try_recv()

        if msg_dict is not None:

            new_id = msg_dict['id']
            new_type = msg_dict['ns']

            if new_type == 'world' : ## single obstacle
                if new_id in self.matlab_id :  # Object already exists
                    self.update_obstacle(msg_dict)
                else :  # Object does not exist and must be created
                    print("Number of obstacles do not match")
                    self.add_obstacle(msg_dict)

            else : ## predefined obstacle
                
                ## Check if we removed obstacle
                new_color = [msg_dict['color']['r'], msg_dict['color']['g'], msg_dict['color']['b'], msg_dict['color']['a']]
                
                if new_color == [0,0,0,0]:
                    self.delete_spheres()
                
                else: 
                    if new_type == 'ring':
                        obs = self.ring
                    elif new_type == 'tshape':
                        obs = self.tshape
                    elif new_type == 'wall':
                        obs = self.wall
                    elif new_type == 'line':
                        obs = self.line
                    else :
                        print("INCORRECT OBSTACLE TYPE !")
                    
                    self.update_spheres(obs)



