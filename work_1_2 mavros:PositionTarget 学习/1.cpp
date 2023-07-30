#include "ros/ros.h"
#include "mavros_msgs/PositionTarget.h"


int main(int argc, char **argv)
{
    ros::init( argc, argv, "publish_node");

    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<mavros_msgs::PositionTarget>("position1",10);
    ros::Rate loop_race(10);

    while(ros::ok())
    {
         mavros_msgs::PositionTarget msg;

        // 设置位置信息
        msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        msg.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                        mavros_msgs::PositionTarget::IGNORE_VY |
                        mavros_msgs::PositionTarget::IGNORE_VZ |
                        mavros_msgs::PositionTarget::IGNORE_AFX |
                        mavros_msgs::PositionTarget::IGNORE_AFY |
                        mavros_msgs::PositionTarget::IGNORE_AFZ |
                        mavros_msgs::PositionTarget::IGNORE_YAW_RATE; //设置掩码
        msg.position.x = 1.0;
        msg.position.y = 2.0;
        msg.position.z = 3.0;
        ROS_INFO("running \n");
        pub.publish(msg);

        ros::spinOnce();
        loop_race.sleep();

    }

    return 0;

}