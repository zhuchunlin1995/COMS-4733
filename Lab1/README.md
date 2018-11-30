Instruction:

1. open a terminal and run command: roscore

2. open a new terminal and run command: roslaunch rbx1_bringup fake_turtlebot.launch

3. open the third terminal and run command: rosrun rviz rviz -d `rospack find rbx1_nav`/sim.rviz to launch RViz

4. open the forth terminal and run command: rosrun rbx1_nav timed_out_and_back.py to run the python script.

5. Instruction message will be prompted as "Enter T(translation), R(rotation) or Q(quit)". Then enter T, R or Q correspondingly.

6. If enter T, R, terminal will prompt for the distance or angle. If enter Q, the program will be terminated.


Methods:

1. command:
	Handle the terminal messgage prompt and input.
2. translate:
	handle robot's positive and negative translations.
3. rotate:
	handle robot's positive and negative rotations.
4. quit:
	Quit the program.
