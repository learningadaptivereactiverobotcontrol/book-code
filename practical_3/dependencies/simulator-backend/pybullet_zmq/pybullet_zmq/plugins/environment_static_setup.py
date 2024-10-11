from network_interfaces.zmq import network
from state_representation import Parameter, ParameterType


class EnvironmentSetup:
    """
    Set up static environment around robot for Practical purposes
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
        
        self.executed_once = False

        # Create concentric red and white circles to form a target on top of the box
        self.box_position = [0.5, 0, 0.05]  # Position slightly above the static box
        self.center_position = [0.5, 0, 0.1]  # Position slightly above the static box

        # Set the scaling factor
        self.scale_factor = 0.3

        # Center letters on X-axis (adjust start_y_offset dynamically)
        self.extra_space = 0.3
        self.letter_spacing = 4 * self.scale_factor
        total_width = 8 * self.letter_spacing  # 4 letters with spacing between them
        self.start_y_offset = -(total_width / 2) + self.scale_factor # Center the letters

        # Define the letter grids for E, P, F, L with more cubes to fill the shape
        self.letter_grids = {
            'E': [(0,0), (0,1), (0,3), (0,4), (1,0), (1,2), (1,4), (2,0), (2,4)],
            'P': [(0,0), (0,1), (0,2), (0,3), (0,4), (1,4), (1,2), (2,4), (2,2), (2,3)],
            'F': [(0,0), (0,1), (0,3), (0,4), (1,4), (1,2), (2,2), (2,4)],
            'L': [(0,0), (0,1), (0,2), (0,3), (0,4), (1,0), (2,0)],
            'A': [(0, 0), (0, 1), (0, 2), (0, 3), (0, 4), (1, 2), (1, 4), (2, 0), (2, 1), (2, 2), (2, 3), (2, 4),],  
            'S': [(0, 0),  (0, 2), (0, 3), (0, 4), (1, 0), (1, 2), (1, 4), (2, 0), (2, 1), (2,2), (2, 4)]
        }
        
    # Function to create a static red cube at a specified position
    def create_red_cube(self, position):
        half_extents = [0.5* self.scale_factor, 0.5* self.scale_factor, 0.5* self.scale_factor]   # Small cube size
        collision_shape_id = self._pb.createCollisionShape(self._pb.GEOM_BOX, halfExtents=half_extents)
        visual_shape_id = self._pb.createVisualShape(self._pb.GEOM_BOX, halfExtents=half_extents, rgbaColor=[1, 0, 0, 1])  # Red color
        self._pb.createMultiBody(baseMass=0, baseCollisionShapeIndex=collision_shape_id, 
                        baseVisualShapeIndex=visual_shape_id, basePosition=position)

    # Function to create letter out of cubes on the "wall" (constant X = -3)
    def create_letter(self, letter, offset_y):
        for (y, z) in self.letter_grids[letter]:
            # Create a cube for each point in the grid
            self.create_red_cube([-3, y*self.scale_factor + offset_y, z*self.scale_factor + 0.5* self.scale_factor])  # Positioned 3m behind the origin on a vertical plane


    def create_flat_circle(self, position, radius, color):
        # circle_shape = self._pb.createCollisionShape(self._pb.GEOM_CYLINDER, radius=radius, height=0.01)  # Very thin cylinder
        visual_shape = self._pb.createVisualShape(self._pb.GEOM_CYLINDER, radius=radius, length=0.01, rgbaColor=color)
        self._pb.createMultiBody(
            baseMass=0,
            # baseCollisionShapeIndex=circle_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=position
        )

    def create_target_support(self):
        # Create a static box by setting mass to zero
        # collision_shape_id = self._pb.createCollisionShape(self._pb.GEOM_BOX, halfExtents=[0.1, 0.1, 0.05])
        visual_shape_id = self._pb.createVisualShape(self._pb.GEOM_BOX, halfExtents=[0.1, 0.1, 0.05], rgbaColor=[0, 0, 1, 1])

        # Add the static box at a desired position (e.g., at (0, 0, 1))
        static_box_id = self._pb.createMultiBody(
            baseMass=0,  # Set mass to 0 to make it static
            # baseCollisionShapeIndex=collision_shape_id,
            baseVisualShapeIndex=visual_shape_id,
            basePosition=self.box_position
        )

    def execute(self):
        """
        Execution function of the plugin.
        """

        if not self.executed_once :
   
            # Create the letters E, P, F, L
            self.create_letter('E', self.start_y_offset - self.extra_space)
            self.create_letter('P', self.start_y_offset + self.letter_spacing - self.extra_space)
            self.create_letter('F', self.start_y_offset + 2 * self.letter_spacing - self.extra_space)
            self.create_letter('L', self.start_y_offset + 3 * self.letter_spacing - self.extra_space)

            self.create_letter('L', self.start_y_offset + 4 * self.letter_spacing + self.extra_space)
            self.create_letter('A', self.start_y_offset + 5 * self.letter_spacing + self.extra_space)
            self.create_letter('S', self.start_y_offset + 6 * self.letter_spacing + self.extra_space)
            self.create_letter('A', self.start_y_offset + 7 * self.letter_spacing + self.extra_space)


            # Create red and white rings (larger radius first for the outer ring)
            self.create_target_support()
            self.create_flat_circle(self.center_position, radius=0.03, color=[1, 0, 0, 1])  # Outer red circle
            # self.create_flat_circle(self.center_position, radius=0.07, color=[1, 1, 1, 1])  # Middle white circle
            # self.create_flat_circle(self.center_position, radius=0.04, color=[1, 0, 0, 1])  # Inner red circle
            # # self.create_flat_circle(self.center_position, radius=0.02, color=[1, 1, 1, 1])  # Center white circle

            self.executed_once = True

        