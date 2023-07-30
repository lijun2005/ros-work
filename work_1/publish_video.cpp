#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>


int main(int argc,char **argv)
{
    ros::init(argc,argv,"publisher_video");
    cv::VideoCapture cap(0);


    if(!cap.isOpened())
    {
        ROS_INFO("error!\n");
        return -1;
    }
    

    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_transport::Publisher image_pub;
    image_pub = it.advertise("out_image",1);

    cv::Mat frame;
    sensor_msgs::ImagePtr msg;
    while(ros::ok()){
        cap >> frame;
        if(!frame.empty())
        {
            msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8",frame).toImageMsg();
            image_pub.publish(msg);
            ROS_INFO("running!\n");
        }
        
        ros::spinOnce();
    }
    cap.release();
    return 0;
}






