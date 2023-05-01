from random import sample, seed
from re import A
import time
import pickle
import numpy as np

# import vrep_interface as vpi
import RobotUtil as rt
import Franka
import time
import frankapy

#Seed the random object
seed(10)

np.random.seed(10)

thresh=0.1
iters = 20000

deg_to_rad = np.pi/180.

#Initialize robot object
mybot = Franka.FrankArm()

#Initialize real Franka
real_franka = frankapy.FrankaArm()

# Initialize some variables related to the simulation
joint_counter = 0

# Initializing planner variables as global for access between planner and simulator
plan=[]
interpolated_plan = []
plan_length = len(plan)
inc = 1

# Add obstacle descriptions into pointsObs and axesObs
pointsObs=[]
axesObs=[]

'''What we did:
- Tap the end effector to the vertices that define the box
- subtract values in a logical way (if x vals change sub those)
- Get dim in this way for three seperate points
- from this obtain xyz center by dividing the dim by 2 and adding this to the lowest raw value 
- Still have to define the walls as obstacles

'''

xyz = []
dim = []

## =============================== init obs & goals ===============================

# box obstacle

dim.append(0.61918278 - 0.34031181 - 0.1) # length 
dim.append(abs(-0.16467692 - 0.20047515) - 0.35) # width
dim.append(0.2273642) # height

xyz.append(dim[0]/2 + 0.34031181) # xpos
xyz.append(dim[1]/2 - 0.16467692 +0.9 ) # ypos
xyz.append(dim[2]/2 + 0) # zpos

envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.],xyz),dim)
pointsObs.append(envpoints), axesObs.append(envaxes)

# ## left wall 
# xyz = [0.15, 0.46, 0.5]
# dim = [1.2, 0.01, 1.1]

# envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.],xyz),dim)
# pointsObs.append(envpoints), axesObs.append(envaxes)


# ## right wall 
# xyz = [0.15, -0.46, 0.5]
# dim = [1.2, 0.01, 1.1]

# envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.],xyz),dim)
# pointsObs.append(envpoints), axesObs.append(envaxes)

# ## top wall 
# xyz = [0.2, 0, 1]
# dim = [1.2, 1, 0.01]

# envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.],xyz),dim)
# pointsObs.append(envpoints), axesObs.append(envaxes)

# ## back wall 
# xyz = [-0.41, 0, 0.5]
# dim = [0.01, 1, 1.1]

# envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.],xyz),dim)
# pointsObs.append(envpoints), axesObs.append(envaxes)

# ## front
# xyz = [0.75, 0, 0.5]
# dim = [0.01, 1, 1.1]

# envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.],xyz),dim)
# pointsObs.append(envpoints), axesObs.append(envaxes)

# # bottom 
# dim = [1.2, 1, 0.01]

# envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.],xyz),dim)
# pointsObs.append(envpoints), axesObs.append(envaxes)
# xyz = [0.2, 0, -0.05]
# dim = [1.2, 1, 0.01]

# envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.],xyz),dim)
# pointsObs.append(envpoints), axesObs.append(envaxes)


# define start and goal
deg_to_rad = np.pi/180.

# set the initial and goal joint configurations
# qInit = [ 0.02553083, -0.13707075, -0.48147371, -2.4088776,  -0.04392068,  2.33436979,
#   0.45919799]

qInit = [ 0.02553083, -0.13707075, -0.48147371, -2.4088776,  -0.04392068,  2.33436979,
  0.45919799]

qGoal1 = np.array([5.66854856e-04 ,-3.77525289e-01 ,-1.43185836e-01 ,-2.34214534e+00
 , 4.10799853e-04 , 1.99624480e+00 , 7.86346298e-01])

qGoal2 = np.array([ 0.17462477,  0.08674834,  0.17746724, -2.227393 ,   0.15537398,  2.32222484,
  1.0864026 ])

# qGoal = np.array([
	
#   [ 0.01232109, -0.49310141, -0.61045661, -2.3324406,  -0.36860691,  2.03737528,
#   0.71397186],
#   [ 0.14432697, -0.72605193, -0.09714947, -2.37137992, -0.16611002,  1.96172093,
#   0.95548059],
#   [ 0.21039833, -0.46555444,  0.38370584, -2.25732931, -0.07590572,  1.97539554,
#   1.24093726],
#   [ 0.17462477,  0.08674834,  0.17746724, -2.227393 ,   0.15537398,  2.32222484,
#   1.0864026 ]
# ])

## =============================== init obs & goals ===============================

# Initialize some data containers for the RRT planner
rrtVertices=[]
rrtEdges=[]

rrtVertices.append(qInit)
rrtEdges.append(0)


FoundSolution=False
SolutionInterpolated = False

# Utility function to find the index of the nearset neighbor in an array of neighbors in prevPoints
def FindNearest(prevPoints,newPoint):
	D=np.array([np.linalg.norm(np.array(point)-np.array(newPoint)) for point in prevPoints])
	return D.argmin()

# Utility function for smooth linear interpolation of RRT plan, used by the controller
def naive_interpolation(plan):

	angle_resolution = 0.01

	global interpolated_plan 
	global SolutionInterpolated
	interpolated_plan = np.empty((1,7))
	SolutionInterpolated = False
	np_plan = np.array(plan)
	interpolated_plan[0] = np_plan[0]
	
	for i in range(np_plan.shape[0]-1):
		max_joint_val = np.max(np_plan[i+1] - np_plan[i])
		number_of_steps = int(np.ceil(max_joint_val/angle_resolution))
		inc = (np_plan[i+1] - np_plan[i])/number_of_steps

		for j in range(1,number_of_steps+1):
			step = np_plan[i] + j*inc
			interpolated_plan = np.append(interpolated_plan, step.reshape(1,7), axis=0)


	SolutionInterpolated = True
	print("Plan has been interpolated successfully!")



#TODO: - Create RRT to find path to a goal configuration by completing the function
# below. Use the global rrtVertices, rrtEdges, plan and FoundSolution variables in 
# your algorithm

def RRTQuery(qInit_, qGoal_):

	# global FoundSolution
	# global plan
	# global rrtVertices
	# global rrtEdges


	FoundSolution=False
	qInit = qInit_
	qGoal = qGoal_

	rrtVertices=[]
	rrtEdges=[]
	plan=[]

	rrtVertices.append(qInit)
	rrtEdges.append(0)
	
	while len(rrtVertices)<iters and not FoundSolution:

		# Fill in the algorithm here

		print(len(rrtVertices))

		# sample the joint space: our joint space is between -pi -> pi with 7 joints #! verify this
		qRand = mybot.SampleRobotConfig()

		#! Goal Bias: for a small prob, switch qRand with qGoal
		if np.random.uniform(0, 1) < thresh:
			qRand = qGoal#[np.random.randint(0, 3)]

		# find out which node this same is nearest to
		idNear = FindNearest(rrtVertices, qRand)
		qNear = rrtVertices[idNear]

		# convert to np arrays
		qNear, qRand = np.asarray(qNear), np.asarray(qRand)

		# if the length between this random point and neighbor is too long, 
		# keep extending towards qRand and stop if there is a collision
		while np.linalg.norm(qRand - qNear) > thresh:
			
			# find incremental step length
			# delta_q is 
			delta_q = (thresh * (qRand - qNear) / np.linalg.norm(qRand - qNear)  ) #! how is this delta_q
			qConnect = qNear + delta_q

			# if there is no collision, update the edge & vertices with qConnect
			if not mybot.DetectCollisionEdge(qConnect, qNear, pointsObs, axesObs):
				rrtVertices.append(qConnect)
				rrtEdges.append(idNear)
				qNear = qConnect
			# if there is a collision, break
			else:
				break 

		#! ???
		qConnect = qRand
		if not mybot.DetectCollisionEdge(qConnect, qNear, pointsObs, axesObs):
			rrtVertices.append(qConnect)
			rrtEdges.append(idNear)
		
		# See if we are close to the goal
		# make make the connection if we are and break - we have found the goal
		idNear = FindNearest(rrtVertices, qGoal)
		if np.linalg.norm(qGoal - rrtVertices[idNear]) < 0.025:
			rrtVertices.append(qGoal)
			rrtEdges.append(idNear)
			FoundSolution = True
			break

	print(FoundSolution, len(rrtVertices))

	### if a solution was found
	if FoundSolution:
		# Extract path
		c=-1 #Assume last added vertex is at goal 
		plan.insert(0, rrtVertices[c])

		while True:
			c=rrtEdges[c]
			plan.insert(0, rrtVertices[c])
			if c==0:
				break

		# TODO - Path shortening
		for i in range(150):
			# get a random value along the path (excluding the goal)
			anchorA = np.random.randint(0, len(plan)-2)
			# find another point between the first and the goal
			anchorB = np.random.randint(anchorA+1, len(plan)-1)

			# #! why??
			shiftA = np.random.uniform(0, 1)
			shiftB = np.random.uniform(0, 1)
			
			# print("a", plan[anchorA], type(plan[anchorA]))
			# print("A+1", plan[anchorA+1], type(plan[anchorA+1]))
			# print("B", plan[anchorB], type(plan[anchorB]))
			# print("B+1", plan[anchorB+1], type(plan[anchorB+1]))
			# print("==================================")

			# candidate A & B are vertices on the path
			candidateA = (1-shiftA)*np.asarray(plan[anchorA])+shiftA*np.asarray(plan[anchorA+1])
			candidateB = (1-shiftB)*np.asarray(plan[anchorB])+shiftB*np.asarray(plan[anchorB+1])


			if not mybot.DetectCollisionEdge(candidateA, candidateB, pointsObs, axesObs):
				# remove the unnecessary points along the path between anchorA and anchorB
				while anchorB>anchorA:
					plan.pop(anchorB)
					anchorB = anchorB-1
				# replace the vertices on the path
				plan.insert(anchorA+1, candidateB)
				plan.insert(anchorA+1, candidateA)
	
		
		for (i, q) in enumerate(plan):
			print("Plan step: ", i, "and joint: ", q)
		plan_length = len(plan)
		
		naive_interpolation(plan)

		return plan

	else:
		print("No solution found")



################################# YOU DO NOT NEED TO EDIT ANYTHING BELOW THIS ##############################

if __name__ == "__main__":

	mybot.PlotCollisionBlockPoints(qInit, pointsObs)
	mybot.PlotCollisionBlockPoints(qGoal1, pointsObs)

	# Compute the RRT solution
	path1 = RRTQuery(qInit, qGoal1)
	path2 = RRTQuery(qGoal1, qGoal2)
	print("\n\n\npath: ", path1, path2)
	print("\n\n\npath: ", len(path1), len(path2))
	path1.extend(path2)
	if type(path1) is not None:
		print("Beginning path...")
		for i, pos in enumerate(path1):
			print(f"Executing position {i+1}")
			real_franka.goto_joints(pos)
		print("Reached goal position!")
		print("Resetting Joints...")
		real_franka.reset_joints()