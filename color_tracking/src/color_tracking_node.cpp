#include <cmvision/Blob.h>
#include <cmvision/Blobs.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

ros::Publisher pub_msg;
geometry_msgs::Twist cmd_msg;

/************************************************************
* Function Name: blobsCallBack
* Parameters: const cmvision::Blobs
* Returns: void
* Description: This is the callback function of the /blobs topic
***********************************************************/
void blobsCallBack (const cmvision::Blobs& blobsIn)
{
/************************************************************
* These blobsIn.blobs[i].red, blobsIn.blobs[i].green, and blobsIn.blobs[i].blue values depend on the
* values those are provided in the colos.txt file.
* For example, the color file is like:
*
* [Colors]
* (255, 0, 0) 0.000000 10 RED
* (255, 255, 0) 0.000000 10 YELLOW
* [Thresholds]
* ( 127:187, 142:161, 175:197 )
* ( 47:99, 96:118, 162:175 )
*
* Now, if a red blob is found, then the blobsIn.blobs[i].red will be 255, and the others will be 0.
* Similarly, for yellow blob, blobsIn.blobs[i].red and blobsIn.blobs[i].green will be 255, and
blobsIn.blobs[i].blue will be 0.
************************************************************/
    ros::Rate rate(10);
    for (int i = 0; i < blobsIn.blob_count; i++) {
        ROS_INFO("blob found");
	cmd_msg.linear.x = 1;
	pub_msg.publish(cmd_msg);
    }
}
int main(int argc, char **argv)
{
ros::init(argc, argv, "color_tracking_node");
ros::NodeHandle n ("~");
 //subscribe to /blobs topic
ros::Subscriber blobsSubscriber = n.subscribe("/blobs", 100, blobsCallBack);
pub_msg = n.advertise<geometry_msgs::Twist>("follower_velocity",1);
ros::spin();
}
