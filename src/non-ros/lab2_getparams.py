import numpy as np
# from autolab_core import RigidTransform
from frankapy import FrankaArm
import time


if __name__ == "__main__":

    fa = FrankaArm(with_gripper=False)

    fa.reset_joints()

    user_input = int(input("Please Enter a Command:\n1 - Open Gripper\n2 - Close Gripper\n3 - Both\n4 - Neither\n"))

    if user_input == 1:
        fa.open_gripper()
    elif user_input == 2:
        fa.close_gripper()
    elif user_input == 3:
        fa.open_gripper()
        time.sleep(3)
        fa.close_gripper()
    else:
        pass

    # run the franka arm in guide mode for 1000 seconds
    # block = False will allow us to be able to run other code in parallel
    fa.run_guide_mode(1000, block=False)
    print("Starting Guide Mode...\n")
    

    while True:

        # take input from the user
        user_input = int(input("Please Enter a Command:\n1 - Print EE Pose\n2 - Print Joint Angle\n3 - End Guide Mode\n"))

        if user_input == 1:
            # printing pose...
            ee_world_pose = fa.get_pose()
            print(ee_world_pose)

        elif user_input == 2:
            # printing joints...
            joint_world = fa.get_joints()
            print(joint_world)

        elif user_input == 3:
            # kills the user guide mode...
            fa.stop_skill()

            reset = int(input("Reset the arm? [0 = No, 1 = Yes]: "))
            if reset:
                fa.reset_joints()

            break




    

    

        





