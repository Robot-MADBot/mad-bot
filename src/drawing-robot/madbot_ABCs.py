from includes.writing_bot import *
import time

if __name__ == "__main__":

    # 1. Designate the font we will use
    font_path = "includes/fonts/" + 'ReliefSingleLine-Regular.ttf'

    # 2. Designate the writing bot class and tell it what font it will use
    madbot = WritingBot(font_path)

    alphabet = "YZ"

    for letter in alphabet:
    
        # 3. show what the text will look like untransformed
        vertices, codes, points_to_plot, plot_xlim, plot_ylim = madbot.text2Write(letter, display=True)

        # 4. show what the text will look like transformed
        """
        change board length & height with vision
        """       
        scale_x, scale_y = madbot.scaleBoard2World(plot_xlim, plot_ylim, board_height=madbot.buddah_width, board_length=madbot.buddah_length)

        # 5. Transform the points to the world and obtain the waypoints needed to draw
        """
        change the shifting with vision -> the bottom left corner 
        """
        shift_x, shift_y= -0.08735142, 0.39408099 #-0.1788297, 0.378 
        waypoints = madbot.transformCoords2World(vertices, codes, points_to_plot, scale_x, scale_y, shift_x, shift_y, plot=True)
        
        # 6. Wet the brush
        print("Going above the bowl...")
        above_bowl = madbot.find_go2pose(trans=madbot.above_bowl)
        madbot.fa.goto_pose(above_bowl, duration=8, use_impedance=False)

        
        print("Dipping the brush...")
        dip_brush = madbot.find_go2pose(trans=madbot.bowl_centroid)
        madbot.fa.goto_pose(dip_brush, duration=4, use_impedance=False)
        # time.sleep(5)

        # print("Going above the bowl...")
        # above_bowl = madbot.find_go2pose(trans=madbot.above_bowl)
        # madbot.fa.goto_pose(above_bowl, duration=4, use_impedance=False)

        print("Brush over side...")
        bowl_side = madbot.find_go2pose(trans=madbot.bowl_side)
        madbot.fa.goto_pose(bowl_side, duration=2, use_impedance=False)

        # # 7. go to the first waypoint

        # we have waypoints of writing procedure, need to go to the board position then the run the trajectory
        starting_point = [waypoints[0][0], waypoints[0][1], madbot.board_height]

        # # go to the first point
        start_pose = madbot.find_go2pose(trans=starting_point)
        print("Going to the staring point...")
        madbot.fa.goto_pose(start_pose, duration=6, use_impedance=False)

        # 8. run the waypoints that define the letters
        # # run the trajectory of the letters
        print("Attempting to run the waypoints...")
        try:
            print(f"waypoints : {len(waypoints)} | wp/time {len(waypoints)/20}")
            madbot.runWaypoints(waypoints, time=140, dt = 0.008)#dt=0.005)
        except Exception as e:
            print(e)
            madbot.fa.stop_skill()
            madbot.fa.reset_joints()

        # 9. Reset the joints 
        madbot.fa.stop_skill()
        print("Finished the word...resetting joints...")
        madbot.fa.reset_joints()

"""
- robot writing backwards?
- trajectory tracking happening really slow
- is the board offset correct???
- need to add board wetting condition
"""