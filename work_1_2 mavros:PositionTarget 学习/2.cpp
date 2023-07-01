#include "ros/ros.h"
#include "mavros_msgs/PositionTarget.h"

void callback(const mavros_msgs::PositionTargetConstPtr &msg)
{
    ROS_INFO("received positison x: %f ,y: %f ,z: %f \n",msg->position.x, msg->position.y,msg->position.z);
}



int main(int argc, char **argv)
{
    ros::init( argc, argv, "sub_node");

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe<mavros_msgs::PositionTarget>("position1",10, callback);
    ros::spin();
    return 0;
}
