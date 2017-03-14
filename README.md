# JARVIS: Joke Adaptive Robot with VISual Sensing
To test the system, run the following commands on ROS in separate prompts in order.
	roscore
	roslaunch turtlebot_bringup minimal.launch
	roslaunch astra_launch astra_pro.launch
	roslaunch pocketsphinx robocup.launch
	rosrun sound_play soundplay_node.py
	rosrun face_evaluator face_evaluator_node /home/turtlebot/shape_predictor_68_face_landmarks.dat
	rosrun sound_play state_machine2.py

The robot will ask the user if they want to hear a joke. During this time, the user should maintain a neutral expression and look straight at the screen. If ''yes'' or ''ok'', it will tell a joke, evaluate user's expression, and respond with virtual avatar and motor movements accordingly. Expression prediction scores are stored in /home/turtlebot/interactions, where scores for each frame are separated by white space. This process then repeats.
