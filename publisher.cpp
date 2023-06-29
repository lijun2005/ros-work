#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>
//创建一个publisher


int main(int argc,char **argv)
{
    ros::init(argc,argv,"publisher");
    cv::Mat cv_image = cv::imread("/home/lijun/图片/Wallpapers/2.png");

    if(cv_image.empty())
    {
        ROS_INFO("failed\n");
        return 1;
    }

    //创建节点句柄进行转化
    ros::NodeHandle n;
    image_transport::ImageTransport transport(n);
    image_transport::Publisher image_pub;
    image_pub = transport.advertise("outimage",1);

    cv_bridge::CvImage cvi;//存储图片信息
    cvi.header.frame_id = "image";
    cvi.encoding = "bgr8";
    cvi.image = cv_image;
    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        sensor_msgs::Image im;
        cvi.toImageMsg(im);
        image_pub.publish(im);
        ROS_INFO("sucess!");

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}