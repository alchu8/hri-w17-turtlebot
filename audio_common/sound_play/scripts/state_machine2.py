#!/usr/bin/env python

import roslib
#; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
import speak
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

aveResponse = []

def callback(data):
    rospy.loginfo('%d', data.data)
    aveResponse.append(data.data)

# define state Foo
class Start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
	rospy.loginfo('Executing state Start')
	rospy.loginfo('would you like to hear a joke?')
	ans = raw_input()
        if ans == 'yes':
            return 'outcome1'
        else:
            return 'outcome2'

# define state Bar
class TellJokes(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['happy','sad'])

    def execute(self, userdata):
	global aveResponse
	joke_length = speak.speak_func()
	sub = rospy.Subscriber("chatter", Int32, callback)
	rospy.sleep(joke_length*0.5)
	sub.unregister()
        rospy.loginfo('Executing state TellJokes')
	rospy.loginfo('length: %d', len(aveResponse))
	if sum(aveResponse)*1.0/len(aveResponse) > 3:
	    aveResponse = []
            return 'happy'
	else:
	    aveResponse = []
	    return 'sad'

class Happy(smach.State):
    count = 0
    move_cmd = Twist()
    cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist)
    THRES = 5000
    def __init__(self):
	smach.State.__init__(self, outcomes=['outcome1'])
    def execute(self, userdata):
	rospy.loginfo('Executing state Happy')
	while self.count < self.THRES:
	    rospy.loginfo('count = %d',self.count)
	    self.count += 1
	    self.move_cmd.angular.z = 1
	    self.cmd_vel_pub.publish(self.move_cmd)
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
        rospy.loginfo('Executing state Sad')
        while self.count < self.THRES:
            rospy.loginfo('count = %d',self.count)
            self.count += 1
            self.move_cmd.angular.z = 0.5
            self.cmd_vel_pub.publish(self.move_cmd)
        self.move_cmd.angular.z = 0
        self.cmd_vel_pub.publish(self.move_cmd)
        return 'outcome1'


def main():
    rospy.init_node('smach_example_state_machine')

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
