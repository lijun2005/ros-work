#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>

void callback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    ROS_INFO("sucess!\n");
	cv::Mat img = cv_ptr -> image;
    cv::namedWindow ("test" , cv::WINDOW_AUTOSIZE);
	cv::imshow("image", img);
	cv::waitKey(10);
    cv::destroyWindow("test");
}



int main(int argc,char **argv)
{
    ros::init(argc,argv,"listener");

    //创建节点句柄
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub = it.subscribe("outimage",10,callback);

    ros::spin();
    return 0;
    
}