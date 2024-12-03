#!/usr/bin/env python3

#############################################################################
##  Copyright (C) 2024 Learning Algorithms and Systems Laboratory, EPFL,
##    Switzerland
##   Author: Aude Billard
##   email: aude.billard@epfl.ch
##   website: lasa.epfl.ch
##    
##   Permission is granted to copy, distribute, and/or modify this program
##   under the terms of the GNU General Public License, version 3 or any
##  later version published by the Free Software Foundation.
##
##   This program is distributed in the hope that it will be useful, but
##   WITHOUT ANY WARRANTY; without even the implied warranty of
##   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
##   Public License for more details
#############################################################################

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from visualization_msgs.msg import Marker
import tkinter as tk
# from tkinter import Tk, Listbox, Button, Scale, Label, SINGLE, HORIZONTAL


class ObstacleManager(Node):
    def __init__(self):
        super().__init__('obstacle_manager_gui')

        self.publisher_ = self.create_publisher(Marker, '/visualization_marker', 10)

        self.obstacles = []
        self.obstacles_count = 1
        self.selected_index = None

        self.colors = [
            '#FFB3BA',  # Light Pink
            '#FFDFBA',  # Light Orange
            '#FFFFBA',  # Light Yellow
            '#B3E5FC',  # Light Blue
            '#B3FFBA',  # Light Green
            '#E1BAFF',  # Light Purple
            '#FFCCE5',  # Light Rose
            '#FFFFCC',  # Light Cream
            '#E6FFCC',  # Light Lime
            '#FFE4CC',  # Light Peach
            '#FFCCF2',  # Light Magenta
            '#CCFFE5',  # Light Aqua
            '#CCE5FF',  # Light Cornflower Blue
            '#FFEDCC',  # Light Apricot
            '#D6EFFF',  # Light Powder Blue
            '#FFD6FF',  # Light Lavender Pink
            '#FFF0D6',  # Light Beige
            '#CCF5FF',  # Light Cyan
            '#E0FFCC',  # Light Mint
            '#BAE1FF'  # Light Sky Blue
        ]

        # Initialize the Tkinter window
        self.root = tk.Tk()
        self.root.title("Obstacle Manager")

        # Ensure the window stays on top
        self.root.attributes('-topmost', True)

        # Bind the window close event to a function
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        # Configure the grid to be resizable
        for i in range(10):
            self.root.grid_rowconfigure(i, weight=1)
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, weight=1)
        
        # Button to add a new obstacle
        self.add_button = tk.Button(self.root, text="Add Obstacle", command=self.add_obstacle, bg='lightblue', fg='black')
        self.add_button.grid(row=0, column=0, columnspan=2, padx=10, pady=10)

        # Listbox to display all obstacles
        self.obstacle_list = tk.Listbox(self.root, selectmode=tk.SINGLE)
        self.obstacle_list.grid(row=1, column=0, columnspan=3, padx=10, pady=0, sticky="nsew")
        self.obstacle_list.bind('<<ListboxSelect>>', self.on_select)

        # Labels and sliders for obstacle parameters
        self.position_label = tk.Label(self.root, text="Position")
        self.position_label.grid(row=2, column=0, columnspan=2, pady=(5,0))

        # X Slider
        self.x_label = tk.Label(self.root, text="X")
        self.x_label.grid(row=3, column=0, padx=20, pady=(20, 0), sticky='e')  # 'e' aligns the label to the east (right)
        self.x_slider = tk.Scale(self.root, from_=-1.5, to=1.5, resolution=0.05, orient=tk.HORIZONTAL, command=self.update_obstacle)
        self.x_slider.grid(row=3, column=1, padx=5, pady=0, sticky='w')  # 'w' aligns the slider to the west (left)

        # Y Slider
        self.y_label = tk.Label(self.root, text="Y")
        self.y_label.grid(row=4, column=0, padx=20, pady=(20, 0), sticky='e')
        self.y_slider = tk.Scale(self.root, from_=-1.5, to=1.5, resolution=0.05,orient=tk.HORIZONTAL, command=self.update_obstacle)
        self.y_slider.grid(row=4, column=1, padx=5, pady=0, sticky='w')

        # Z Slider
        self.z_label = tk.Label(self.root, text="Z")
        self.z_label.grid(row=5, column=0, padx=20, pady=(20, 0), sticky='e')
        self.z_slider = tk.Scale(self.root, from_=0, to=1.5, resolution=0.05, orient=tk.HORIZONTAL, command=self.update_obstacle)
        self.z_slider.grid(row=5, column=1, padx=5, pady=0, sticky='w')

        # Radius Label and Slider
        self.radius_label = tk.Label(self.root, text="Radius")
        self.radius_label.grid(row=6, column=0, padx=5, pady=(20, 0), sticky='e')
        self.radius_slider = tk.Scale(self.root, from_=0.05, to=0.5, resolution=0.05, orient=tk.HORIZONTAL, command=self.update_obstacle)
        self.radius_slider.grid(row=6, column=1, padx=5, pady=5, sticky='w')

        # Sensitivity Label and Slider
        self.sensitivity_label = tk.Label(self.root, text="Sensitivity")
        self.sensitivity_label.grid(row=7, column=0, padx=5, pady=(20, 0), sticky='e')
        self.sensitivity_slider = tk.Scale(self.root, from_=0.00, to=1.0, resolution=0.05, orient=tk.HORIZONTAL, command=self.update_obstacle)
        self.sensitivity_slider.grid(row=7, column=1, padx=5, pady=5, sticky='w')

        # Remove button
        self.update_button = tk.Button(self.root, text="Remove Obstacle", command=self.remove_obstacle, bg='red3', fg='black')
        self.update_button.grid(row=8, column=0, columnspan=2, padx=10, pady=10)
      
        # Labels and sliders for predef obstacle
        self.preded_obs_label_1 = tk.Label(self.root, text="Add predefined obstacle:")
        self.preded_obs_label_1.grid(row=9, column=0, columnspan=2, padx=10, pady=(15,0))

        # Dropdown menu variable
        self.selected_obstacle = tk.StringVar(self.root)
        self.selected_obstacle.set("Select Obstacle")  # default value

        # Dropdown menu
        self.dropdown_menu = tk.OptionMenu(self.root, self.selected_obstacle, "line", "wall", "ring", "tshape", command=self.add_predef_obstacle)
        self.dropdown_menu.grid(row=10, column=0, columnspan=2, padx=0, pady=(5,10))

        # Change the background and foreground of the button part of the OptionMenudark
        self.dropdown_menu.config(bg="seagreen", fg="black", activebackground="darkgreen", activeforeground="white")


    def add_predef_obstacle(self, selection):

        ## There can only be one (check all obs and remove predef)
        for index, item in enumerate( self.obstacle_list.get(0, tk.END)):
            if "Predef" in item:
                ## Remove previous predef
                obstacle = self.obstacles[index]
                self.publish_marker(obstacle, action="remove")
                del self.obstacles[index]
                self.obstacle_list.delete(index)

        new_obstacle = {
            'id': self.obstacles_count,
            'position': [0.0, 0.0, 0.0],
            'radius': 0.0,
            'sensitivity' : 1.0,
            'type' : str(selection),
            'color' : '#2E8B57' ##seagreen
        }      
        self.obstacles_count+=1
        self.obstacles.append(new_obstacle)
        self.obstacle_list.insert('end', f"Predefined obstacle")
        self.obstacle_list.itemconfig(len(self.obstacles)-1, {'bg': '#2E8B57'})
        self.publish_marker(new_obstacle)

    def add_obstacle(self):
        """Add a new obstacle to the list and publish it."""

        index = self.obstacles_count % len(self.colors)  # Cycle through the color list
        color = self.colors[index]

        new_obstacle = {
            'id': self.obstacles_count,
            'position': [1.0, 1.0, 1.0],
            'radius': 0.15,
            'sensitivity' : 1.0,
            'type': "world",
            'color': color
        }
        self.obstacles_count+=1
        self.obstacles.append(new_obstacle)
        self.obstacle_list.insert('end', f"Obstacle {new_obstacle['id']}")
        self.obstacle_list.itemconfig(len(self.obstacles)-1, {'bg': color})
        self.publish_marker(new_obstacle)

    def remove_obstacle(self):
        if self.selected_index is not None:
            obstacle = self.obstacles[self.selected_index]
            self.publish_marker(obstacle, action="remove")
            del self.obstacles[self.selected_index]
            self.obstacle_list.delete(self.selected_index)
            self.selected_index = None


    def on_select(self, event):
        """Handle obstacle selection from the list."""

        selection = event.widget.curselection()
        if selection:
    
            self.selected_index = selection[0]
            selected_obstacle = self.obstacles[self.selected_index]
            self.x_slider.set(selected_obstacle['position'][0])
            self.y_slider.set(selected_obstacle['position'][1])
            self.z_slider.set(selected_obstacle['position'][2])
            self.radius_slider.set(selected_obstacle['radius'])
            self.sensitivity_slider.set(selected_obstacle['sensitivity'])
            self.publish_marker(selected_obstacle)

    def update_obstacle(self, value=None):
        """Update the selected obstacle's position and radius."""
        if self.selected_index is not None:
            obstacle = self.obstacles[self.selected_index]
            obstacle['position'] = [float(self.x_slider.get()), float(self.y_slider.get()), float(self.z_slider.get())]
            obstacle['radius'] = float(self.radius_slider.get())
            obstacle['sensitivity'] = float(self.sensitivity_slider.get())
            self.publish_marker(obstacle)

    def publish_marker(self, obstacle, action="update"):
        """Publish the obstacle as a Marker message to the /visualization_marker topic."""
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = obstacle['type']
        marker.id = obstacle['id']
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = obstacle['position'][0]
        marker.pose.position.y = obstacle['position'][1]
        marker.pose.position.z = obstacle['position'][2]
        marker.scale.x = marker.scale.y =  obstacle['radius'] * 2  # Radius to Diameter
        
        ## use scale in z as sensitivity
        marker.scale.z = obstacle['sensitivity']

        # Set the color (parse the hex color)
        r, g, b = self.hex_to_rgb(obstacle['color'])

        if action == "update":
            marker.color.r = r / 255.0
            marker.color.g = g / 255.0
            marker.color.b = b / 255.0
            marker.color.a = 1.0
        elif action == "remove":
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.0
            
        self.publisher_.publish(marker)

    def hex_to_rgb(self, hex_color):
        """Convert a hex color (e.g., '#FF00FF') to an RGB tuple."""
        hex_color = hex_color.lstrip('#')
        return tuple(int(hex_color[i:i+2], 16) for i in (0, 2, 4))
    
    def on_close(self):
        """Remove all obstacles when the window is closed."""
        for i in range(len(self.obstacle_list.get(0, tk.END))):
            # Delete all obstacles in the list
            obstacle = self.obstacles[0]
            self.publish_marker(obstacle, action="remove")
            del self.obstacles[0]
            self.obstacle_list.delete(0)
            
        self.root.destroy()  # Close the window

    def run_gui(self):
        """Run the Tkinter GUI loop."""
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    obstacle_manager = ObstacleManager()

    # Tkinter runs its own main loop, so it doesn't need rclpy.spin()
    obstacle_manager.run_gui()

    # Shutdown ROS 2 when the GUI is closed
    rclpy.shutdown()


if __name__ == '__main__':
    main()
