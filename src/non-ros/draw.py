import numpy as np
from autolab_core import RigidTransform
from frankapy import FrankaArm
import time
import math

import copy

def dip_brush(starting_pose, ending_pose):
    curr_top = copy.deepcopy(starting_pose)
    curr_top[2] += 0.1
    des_pose= RigidTransform(rotation=np.array([
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0]]),
            translation=np.array(curr_top),
            from_frame='franka_tool', to_frame='world')
    fa.goto_pose(des_pose, duration=3.0, use_impedance=False)

    bowl_top = copy.deepcopy(bowl_centroid)
    bowl_top[2] += 0.1
    des_pose= RigidTransform(rotation=np.array([
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0]]),
            translation=np.array(bowl_top),
            from_frame='franka_tool', to_frame='world')
    fa.goto_pose(des_pose, duration=5.0, use_impedance=False)

    des_pose= RigidTransform(rotation=np.array([
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0]]),
            translation=np.array(bowl_centroid),
            from_frame='franka_tool', to_frame='world')

    fa.goto_pose(des_pose, duration=2.0, use_impedance=False)
    time.sleep(5)
    #############################################################
    des_pose= RigidTransform(rotation=np.array([
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0]]),
            translation=np.array(fancy_step),
            from_frame='franka_tool', to_frame='world')
    fa.goto_pose(des_pose, duration=1.0, use_impedance=False)

    fancy_step2 = copy.deepcopy(fancy_step)
    fancy_step2[0] += 0.06
    des_pose= RigidTransform(rotation=np.array([
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0]]),
            translation=np.array(fancy_step2),
            from_frame='franka_tool', to_frame='world')
    fa.goto_pose(des_pose, duration=1.0, use_impedance=False)

    # des_pose= RigidTransform(rotation=np.array([
    #         [1.0, 0.0, 0.0],
    #         [0.0, -1.0, 0.0],
    #         [0.0, 0.0, -1.0]]),
    #         translation=np.array(fancy_step),
    #         from_frame='franka_tool', to_frame='world')
    # fa.goto_pose(des_pose, duration=1.0, use_impedance=False)

    # fancy_step3 = copy.deepcopy(fancy_step)
    # fancy_step3[2] += 0.1
    # des_pose= RigidTransform(rotation=np.array([
    #         [1.0, 0.0, 0.0],
    #         [0.0, -1.0, 0.0],
    #         [0.0, 0.0, -1.0]]),
    #         translation=np.array(fancy_step3),
    #         from_frame='franka_tool', to_frame='world')
    # fa.goto_pose(des_pose, duration=1.0, use_impedance=False)

    ####################################################################
    # des_pose= RigidTransform(rotation=np.array([
    #         [1.0, 0.0, 0.0],
    #         [0.0, -1.0, 0.0],
    #         [0.0, 0.0, -1.0]]),
    #         translation=np.array(bowl_top),
    #         from_frame='franka_tool', to_frame='world')

    # fa.goto_pose(des_pose, duration=2.0, use_impedance=False)

    # des_pose= RigidTransform(rotation=np.array([
    #         [1.0, 0.0, 0.0],
    #         [0.0, -1.0, 0.0],
    #         [0.0, 0.0, -1.0]]),
    #         translation=np.array(curr_pos),
    #         from_frame='franka_tool', to_frame='world')
    # fa.goto_pose(des_pose, duration=2.0, use_impedance=False)

def draw_circle(coords):
  
    for i in coords:
        
        des_pose= RigidTransform(rotation=np.array([
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0]]),
            translation=np.array(i),
            from_frame='franka_tool', to_frame='world')
        fa.goto_pose(des_pose, duration=0.3, use_impedance=False)

def draw_D(coords):
    des_pose= RigidTransform(rotation=np.array([
        [1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 0.0, -1.0]]),
        translation=np.array(coords[-1]),
        from_frame='franka_tool', to_frame='world')
    fa.goto_pose(des_pose, duration=2.0, use_impedance=False)
    des_pose= RigidTransform(rotation=np.array([
        [1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 0.0, -1.0]]),
        translation=np.array(coords[0]),
        from_frame='franka_tool', to_frame='world')
    fa.goto_pose(des_pose, duration=2.0, use_impedance=False)
    draw_circle(coords)
    
def shift_vertices(vertices, shift_x=0, shift_y=0):
    vertices = [(x + shift_x, y + shift_y) for x, y in vertices]
    return vertices

def scale_vertices(vertices, scale_x=1.0, scale_y=1.0):
    """
    This function scales the vertices of a polygon by a given scale factor.    
    """   
    
    scaled_vertices = []
    for vertex in vertices:
        scaled_vertices.append((vertex[0] * scale_x, vertex[1] * scale_y))
    return scaled_vertices


top_left = [0.37823196, -0.1788297, 0.11158911]
bottom_left = [0.59704681, -0.1788297, 0.11158911]
bottom_right = [0.59704681, 0.09648339, 0.11298614]
top_right = [0.37823196, 0.09648339, 0.11298614]
bowl_centroid = [0.52138842, 0.23314998, 0.11760377]
fancy_step =  [0.42724328, 0.23033206, 0.14543322]
center_board = [(top_left[0]+bottom_right[0])/2, (top_left[1]+bottom_right[1])/2, 0.11]
# print(center_board)
radius = 0.05
x_cords = np.linspace((center_board[0]-radius), (center_board[0]+radius), 20)
# x_value = (x_cords -center_board[0])**2
# print((x_cords -center_board[0])**2 - radius**2)
coords = []
for i in range(x_cords.shape[0]):
    xvalue = radius**2 - (x_cords[i]-center_board[0])**2 
    if xvalue  < 0:
        xvalue = 0

    x2 = math.sqrt((xvalue ))+center_board[1]
    
    y = [x_cords[i], x2, 0.11]
    coords.append(y)
# print(coords)
# time.sleep(60)

default_rot_matrix = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])

# vertices_a = [[0.081, 0.0926, 0.11], [0.1544, 0.0926, 0.11], [0.17120000000000002, 0.05, 0.11], [0.1184, 0.185, 0.11], [0.11760000000000001, 0.185, 0.11], [0.064, 0.05]]
vertices_a = np.array([[0.44456249999999997, -0.12554678490890342, 0.11], [0.44456249999999997, -0.12554678490890342, 0.11], [0.44456249999999997, 0.0006134075971123976, 0.11], [0.378, 0.029489309969061545, 0.11], [0.5889375, -0.06126352605706428, 0.11], [0.5889375, -0.0626385690271571, 0.11], [0.378, -0.15476644802337575, 0.11]])
# print(vertices_a)
# print("\n")
# vertices_a = [[vertice[0] + 0.22, vertice[1], vertice[2] ] for vertice in vertices_a]
# print(vertices_a)
# print("\n")
# angle = math.pi
# mat_z = np.array([[math.cos(math.pi), -math.sin(math.pi), 0], \
#                   [math.sin(math.pi), math.cos(math.pi), 0], \
#                   [0, 0, 1]])
# vertices_a = vertices_a @ mat_z
# print(vertices_a)
# import sys; sys.exit()
# min_x_vertice_a = 
# vertices_a_not_working = [[0.44456249999999997, 0.1497629150910966, 0.11], [0.44456249999999997, 0.1497629150910966, 0.11], [0.44456249999999997, 0.2759231075971124, 0.11], [0.378, 0.30479900996906156, 0.11], [0.5889375, 0.21404617394293574, 0.11], [0.5889375, 0.2126711309728429, 0.11], [0.378, 0.12054325197662427, 0.11]]
commands_a = [1, 2, 2, 1, 2, 2, 2]

def line_to(vertice):
    des_pose= RigidTransform(rotation=default_rot_matrix,
                             translation=np.array(vertice),
                             from_frame='franka_tool', to_frame='world')
    fa.goto_pose(des_pose, duration=3.0, use_impedance=False)

def move_to(past_vertice, vertice):
    past_vertice[2] += 0.05
    des_pose= RigidTransform(rotation=default_rot_matrix,
                             translation=np.array(past_vertice),
                             from_frame='franka_tool', to_frame='world')
    fa.goto_pose(des_pose, duration=3.0, use_impedance=False)

    des_pose= RigidTransform(rotation=default_rot_matrix,
                             translation=np.array(vertice),
                             from_frame='franka_tool', to_frame='world')
    fa.goto_pose(des_pose, duration=3.0, use_impedance=False)

def draw_word(vertices, commands):
    past_vertice = vertices[0]
    for (vertice, command) in zip(vertices, commands):
        print(vertice, command)
        if command == 2:
            line_to(vertice)
        elif command == 1:
            move_to(past_vertice, vertice)
        past_vertice = vertice




fa = FrankaArm()
joints = fa.get_joints()

# dip_brush(vertices_a[0])
# draw_word(vertices_a, commands_a)

# fa.reset_joints()
# print("Finish reset!")

# import sys; sys.exit()




# point_a = [0.47823196, -0.1788297, 0.11158911]

# fa.opeFranka.FrankArm() a desired endeffector pose
# des_pose= RigidTransform(rotation=np.array([
#         [1.0, 0.0, 0.0],
#         [0.0, -1.0, 0.0],
#         [0.0, 0.0, -1.0]]),
#         translation=np.array(top_left),
#         from_frame='franka_tool', to_frame='world')

# fa.goto_pose(des_pose, duration=10.0, use_impedance=False)

dip_brush(coords[-1])
draw_D(coords)

# des_pose= RigidTransform(rotation=np.array([
#         [1.0, 0.0, 0.0],
#         [0.0, -1.0, 0.0],
#         [0.0, 0.0, -1.0]]),
#         translation=np.array(bottom_right),
#         from_frame='franka_tool', to_frame='world')

# fa.goto_pose(des_pose, duration=5.0, use_impedance=False)

fa.reset_joints()
print("Finish reset!")

# fa.goto_pose(des_pose,duration=10.0, use_impedance=False, cartesian_impedances=[3000,
# 3000, 100, 300, 300, 300])





# mesh = np.meshgrid()