#!/usr/bin/env python

import rospy
from includes.text_manipulation import *
from includes.traj_tracking import *
from includes.frankapy.frankapy import *
import matplotlib.patches as patches
import numpy as np
import time
from transforms3d.quaternions import mat2quat

# Set the seed to ensure the same quaternion is generated every time
np.random.seed(1234)

# Define the state machine
class WritingBot:
    def __init__(self, font):
        
        print("Resetting the arm...")
        self.fa = FrankaArm()
        self.fa.reset_joints()
        print("Reset arm!")

        self.font_path = font
        self.board_height = 0.17493918 + 0.003 # 0.17147485 #0.12694734 #0.11158911
        self.lift_height = self.board_height + 0.01

        self.pen_down_HTM = [
        [1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 0.0, -1.0]]

        # Generate the quaternion from the reset pose and hold on to it
        starting_pose = self.fa.get_pose()
        self.pen_down_quat = starting_pose.quaternion

        self.buddah_length = 0.2413
        self.buddah_width = 0.3048

        self.bowl_centroid =  [0.51859339, 0.22058808, 0.18415329]#[0.52842954, 0.21425549, 0.18259661] #[0.51017637, 0.2247803,  0.14424434] #[0.52138842, 0.23314998, 0.11760377]
        self.above_bowl = [0.52842954, 0.21425549, 0.18259661 + 0.1] #[0.51017637, 0.2247803,  0.14424434 + 0.1] #[0.52138842, 0.23314998, 0.11760377 + 0.1]

        self.bowl_side = [0.43740308-0.02, 0.22452381, 0.21196003 + 0.01] #[0.44670231-0.01, 0.23171216, 0.20904718]

    def find_go2pose(self, trans, rot=None, pen_down=True):

        if pen_down:
            des_pose= RigidTransform(rotation=np.array(self.pen_down_HTM),
                    translation=np.array(trans),
                    from_frame='franka_tool', to_frame='world')
        else:
            des_pose= RigidTransform(rotation=np.array(rot),
                    translation=np.array(trans),
                    from_frame='franka_tool', to_frame='world')
        
        return des_pose

    def text2Write(self, text, display=False, debug=False):
        # get the coordinates 

        vertices, codes, points_to_plot = get_coords_word(text, self.font_path, plot_points=display, debug=debug, remove_close_path=True)

        if display:
            plot_xlim, plot_ylim = render(vertices, codes, points_to_plot)
            
        
        return vertices, codes, points_to_plot, plot_xlim, plot_ylim
    
    def create_waypoints(self, vertices, codes):
        waypoints = list()

        for i in range(len(vertices)-1):
            pt1_x, pt1_y = vertices[i][0], vertices[i][1]
            pt2_x, pt2_y = vertices[i+1][0], vertices[i+1][1]
            x_vals, y_vals = self.linear_interpolation(pt1_x, pt1_y, pt2_x, pt2_y)

            two_list = []
            one_list = []
            for _ in range(len(x_vals)):
                one_list.append(1)
                two_list.append(2)

            if codes[i+1] == 2 or codes[i+1] == 4: # go to or curve
                waypoints += zip(x_vals, y_vals, two_list)
            else:
                waypoints += zip(x_vals, y_vals, one_list)

        return waypoints

    def linear_interpolation(self, x1, y1, x2, y2, num_points=30):
         # calculate the distance between the two points
        distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        distance *= 100 # convert the distance to cm
        # adjust the number of points based on the distance
        num_points = int(num_points * distance)

        if x2 - x1 == 0:  # check if the line is vertical
            # create an array of y values spaced according to number of points
            y_values = np.linspace(y1, y2, num_points)
            # create an array of x values with the same length as y_values
            x_values = np.full_like(y_values, x1)
        else:
            # create an array of x values spaced according to number of points
            x_values = np.linspace(x1, x2, num_points)
            # calculate the slope and y-intercept of the line
            slope = (y2 - y1) / (x2 - x1)
            y_intercept = y1 - slope * x1
            # create an array of y values interpolated between the points
            y_values = slope * x_values + y_intercept
            # return the interpolated x and y values

        return x_values, y_values
            
    
    def transformCoords2World(self, vertices, codes, points_to_plot, scale_x, scale_y, shift_x, shift_y, plot=False):
        

        vertices_scaled = scale_vertices(vertices, scale_x, scale_y)
        vertices_shifted = shift_vertices(vertices_scaled, shift_x, shift_y)

        waypoints = self.create_waypoints(vertices_shifted, codes)
        
        wp_x_up, wp_y_up = list(), list()
        wp_x_down, wp_y_down = list(), list()
        vert_x, vert_y = list(), list()
        for wp in waypoints:
            # print("Shifted Vertices: ", vertice)
            if wp[2] == 1: # move to 
                wp_x_up.append(wp[0])
                wp_y_up.append(wp[1])
            else: # line to 
                wp_x_down.append(wp[0])
                wp_y_down.append(wp[1])

        for vert in vertices_shifted:
            vert_x.append(vert[0])
            vert_y.append(vert[1])

        if plot:
            fig, ax = plt.subplots()
            # plot the buddah board
            # Create a rectangle patch
            rect = patches.Rectangle((shift_x, shift_y), self.buddah_width, self.buddah_length, linewidth=2, edgecolor='black', facecolor='gray', zorder=0)

            # add the text
            plt.scatter(wp_x_up, wp_y_up, s = 3)
            plt.scatter(wp_x_down, wp_y_down, color="purple", s = 3)
            plt.scatter(vert_x, vert_y, color="r",s=6)
            
            # add the robot frame
            # x axis
            plt.annotate("", xy=(0.0, 0.1), xytext=(0.0, 0.0),
             arrowprops=dict(arrowstyle="->", color="r"))
            # y axis 
            # x axis
            ax.annotate("", xy=(0.1, 0.0), xytext=(0.0, 0.0),
             arrowprops=dict(arrowstyle="->", color="b"))
            
            ax.set_aspect('equal')

            # Add the patch to the Axes
            ax.add_patch(rect)

            ax.set_title('World Frame')
            ax.set_ylabel("Robot Frame X-Axis (m)")
            ax.set_xlabel("Robot Frame Y-Axis (m)")
            # 0 ft to 3 ft 
            ax.set_ylim([0, 0.8128])
            # -2 ft to 2 ft
            ax.set_xlim([-0.6096, 0.6096])
            ax.legend(["Buddah Board", "Waypoints Up", "Waypoints Down", "Vertices"])
            plt.show()

        # NEED TO CHANGE OUT X, Y IN THE ROBOT FRAME
        waypoints_flipped = list()
        for wp in waypoints:
            waypoints_flipped.append((wp[1], -1*wp[0], wp[2]))
                        
        return waypoints_flipped
    
    def scaleBoard2World(self, plot_xlim, plot_ylim, board_length, board_height):

        orig_x_min = plot_xlim[0]
        orig_x_max = plot_xlim[1]

        orig_y_min = plot_ylim[0]
        orig_y_max = plot_ylim[1]


        scale_x = (board_height) / (orig_x_max - orig_x_min)
        scale_y = (board_length) / (orig_y_max - orig_y_min)

        return scale_x * 0.6, scale_y * 0.6
    
    def runWaypoints(self, wps, time, dt):
        
        traj = []


        i = 0
        for wp in wps:
            trans = list()

            if wp[2] == 1: # move to 
                trans += [wp[0], wp[1], self.lift_height] 
            else: # curve or go to 
                trans += [wp[0], wp[1], self.board_height-i] 

            traj_pt = (trans, self.pen_down_quat)
            
            traj.append(traj_pt)

            i += 0.000001
        
        # for i in wps:
        #     with open("log.txt", "a") as log:
        #         log.write(str(i))
        #         log.write("\n")

        run_traj(self.fa, traj, time, dt)
        


