#!/usr/bin/env python

import roslib
#; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
import speak
from std_msgs.msg import Int8, String
from geometry_msgs.msg import Twist
#import cv2
#import Image
import webbrowser
from sensor_msgs.msg import Image
from scipy.stats import mode

# store the last 10 response scores when a user is hearing a joke
aveResponse = [None]*10
responseCounter = 0
ans = ''
# publish 1 when there's a new user, otherwise 0
new_pub = rospy.Publisher("new_person", Int8, queue_size=10)
msg = Int8()

# camera callback
def cam_callback(data):
    new_pub.publish(msg)

# callback function for speech recognition
def listener_callback(data):
    global ans
    rospy.loginfo("enter listener_callback: %s", data.data)
    ans = data.data

#callback function for face evaluator
def callback(data):
    global responseCounter
    global aveResponse
    rospy.loginfo('face_evaluator: %d', data.data)
    #aveResponse.append(data.data)
    aveResponse[responseCounter] = data.data
    responseCounter = (responseCounter + 1) % 10

# Asking the target if he/she want to hear a joke
class Start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
	global ans,msg

        rospy.loginfo('Executing state Start')
	rospy.loginfo('ans before asking: %s', ans)
#	rospy.loginfo('would you like to hear a joke?')
	speak.welcome()
	rospy.sleep(3)
	#subscribe to speech recognizer
	listener = rospy.Subscriber("/recognizer/output", String, listener_callback)
	# when no answer is received, stay in this while loop
	while ans == '':
	    rospy.sleep(2)
	# Once get the answer from speech recognizer, no need to subscribe
        listener.unregister()
        if ans == 'yes' or ans == 'ok':
	    # When the target answers yes or ok, face evaluator will know there's a person
	    msg.data = 1
	    new_pub.publish(msg) # just publish once
	    rospy.loginfo('%d\n', msg.data)
	    ans = ''
	    while new_pub.get_num_connections() < 1 :
		rospy.sleep(1)
		rospy.loginfo('num_connections: %d', new_pub.get_num_connections())
            return 'outcome1'
        else:
	    ans = ''
            return 'outcome2'

# define state Bar
class TellJokes(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['happy','sad'])

    def execute(self, userdata):
	global aveResponse,msg
	#all states should send zero to face evaluator except start
	msg.data = 0
	new_pub.publish(msg)
	#get the length of joke from speak_func()
	joke_length = speak.speak_func()
	#subscribe to face evaluator
	sub = rospy.Subscriber("face_evaluator/expression", Int8, callback)
	#wait until the joke is finished
	rospy.sleep(joke_length*0.8)
	#no need to subscribe anymore
	sub.unregister()
        rospy.loginfo('Executing state TellJokes')
        rospy.loginfo(aveResponse)
        m = mode(aveResponse)
	rospy.loginfo(m)
	#if sum(aveResponse)*1.0/len(aveResponse) > 2:
        if m.mode[0] > 2:
	    aveResponse = [None]*10
            return 'happy'
	else:
	    aveResponse = [None]*10
	    return 'sad'

class Happy(smach.State):
    count = 0
    move_cmd = Twist()
    cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist)
    THRES = 5000
    def __init__(self):
	smach.State.__init__(self, outcomes=['outcome1'])
    def execute(self, userdata):
	#all states hould send 0 to face evaluator except start
	global msg
	msg.data = 0
	rospy.loginfo('Executing state Happy')
	speak.play_wav('/home/turtlebot/Downloads/woohoo.wav')
        # Load happy image on screen for avatar.
        webbrowser.open('/home/turtlebot/turtlebot_ws/src/hri-w17-turtlebot/audio_common/sound_play/scripts/happy.png')
	speak.play_wav('/home/turtlebot/Downloads/september.mp3')

        while self.count < self.THRES:
	    rospy.loginfo('count = %d',self.count)
	    self.count += 1
	    self.move_cmd.angular.z = 2
	    self.cmd_vel_pub.publish(self.move_cmd)

        self.count = 0
	self.move_cmd.angular.z = 0
	self.cmd_vel_pub.publish(self.move_cmd)

	return 'outcome1'

class Sad(smach.State):
    count = 0
    move_cmd = Twist()
    cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist)
    THRES = 5000
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])
    def execute(self, userdata):
	#all states hould send 0 to face evaluator except start
	global msg
	msg.data = 0
	speak.play_wav('/home/turtlebot/Downloads/what-2.wav')
        rospy.loginfo('Executing state Sad')

        # Load sad image on screen for avatar.
        webbrowser.open('/home/turtlebot/turtlebot_ws/src/hri-w17-turtlebot/audio_common/sound_play/scripts/sad.png')

	speak.play_wav('/home/turtlebot/Downloads/naruto_cutted.mp3')

        while self.count < self.THRES:
            rospy.loginfo('count = %d',self.count)
            self.count += 1
	    if (self.count / 1000)%2 == 0:
                self.move_cmd.angular.z = 0.5
                self.cmd_vel_pub.publish(self.move_cmd)
	    else:
		self.move_cmd.angular.z = -0.5
		self.cmd_vel_pub.publish(self.move_cmd)

        self.count = 0
        self.move_cmd.angular.z = 0
        self.cmd_vel_pub.publish(self.move_cmd)

	return 'outcome1'


def main():
    rospy.init_node('smach_example_state_machine')

    # the message for face_evaluator to identify a new user
    msg.data = 0
    #subscribe to camera
    #cam_sub = rospy.Subscriber("/camera/rgb/image_raw",Image, cam_callback)
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Start', Start(),
                               transitions={'outcome1':'TellJokes', 'outcome2':'outcome4'})
        smach.StateMachine.add('TellJokes', TellJokes(),
                               transitions={'happy':'Happy','sad':'Sad'})
	smach.StateMachine.add('Happy', Happy(),
			       transitions={'outcome1':'Start'})
	smach.StateMachine.add('Sad', Sad(),
                               transitions={'outcome1':'Start'})


    # Execute SMACH plan
    outcome = sm.execute()



if __name__ == '__main__':
    main()
