#!/usr/bin/env python

import roslib
#; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
import speak
from std_msgs.msg import Int32

def callback(data):
    rospy.loginfo("I heard %d", data.data)

# define state Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
	rospy.loginfo('Executing state FOO')
	rospy.loginfo('would you like to hear a joke?')
	ans = raw_input()
        if ans == 'yes':
            return 'outcome1'
        else:
            return 'outcome2'


# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
	joke_length = speak.speak_func()
	sub = rospy.Subscriber("chatter", Int32, callback)
	rospy.sleep(joke_length*0.5)
	sub.unregister()
        rospy.loginfo('Executing state BAR')
        return 'outcome1'

def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FOO', Foo(), 
                               transitions={'outcome1':'BAR', 'outcome2':'outcome4'})
        smach.StateMachine.add('BAR', Bar(), 
                               transitions={'outcome1':'FOO'})

    # Execute SMACH plan
    outcome = sm.execute()



if __name__ == '__main__':
    main()
