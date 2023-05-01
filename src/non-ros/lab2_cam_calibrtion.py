from time import sleep
import numpy as np
import rospy
from geometry_msgs.msg import Pose
from autolab_core import RigidTransform
from frankapy import FrankaArm


if __name__ == "__main__":
    fa = FrankaArm()
    print("Resetting arm...")
    # fa.open_gripper()
    # sleep(10)
    
    fa.close_gripper()
    fa.reset_joints()
    print("Finish reset!")

    fa.run_guide_mode(40)
    print("Out of run guide mode\n\n\n")

    robot_pose = fa.get_pose()
    print(robot_pose)

    offset_pose = RigidTransform(translation=np.array([0.0425, 0, -0.01]), \
                                 rotation=np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]]), \
                                 from_frame="aruco_pose", to_frame="franka_tool")
    aruco_pose = rospy.wait_for_message('/aruco_simple/pose', Pose, timeout=5)
    aruco_rt_pose = RigidTransform.from_pose_msg(aruco_pose, from_frame="aruco_pose", to_frame="camera")
    camera_offset = robot_pose*offset_pose*aruco_rt_pose.inverse()

    aruco_global = camera_offset*aruco_rt_pose
    camera_offset.save("myCalibFilename.tf")
