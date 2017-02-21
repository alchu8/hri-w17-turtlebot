#!/usr/bin/python
import rospy
from geometry_msgs.msg import Twist
from cmvision.msg import Blobs, Blob

turn = 0.0
blob_position = 0

def callback(data):
    rospy.loginfo("enter callback")
    global turn
    global blob_position

    if(len(data.blobs)):

        for obj in data.blobs:
            blob_position = blob_position + obj.x
            blob_position = blob_position/len(data.blobs)
            rospy.loginfo(blob_position)

def run():
    #global blob_position
    # publish twist messages to /cmd_vel
    #pub = rospy.Publisher('/mobile_base/commands/velocity', Twist)

    #subscribe to the robot sensor state
    rospy.init_node("color_tracker")
    rospy.Subscriber('/blobs', Blobs, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException: pass
