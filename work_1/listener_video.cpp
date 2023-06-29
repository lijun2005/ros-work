#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>


void callback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat frame = cv_ptr->image;
    ROS_INFO("sucess\n");
    cv::imshow("frame",frame);
    cv::waitKey(10);

}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"listener_video");

    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub;
    cv::namedWindow("frame",cv::WINDOW_AUTOSIZE);
    sub = it.subscribe("out_image",1,callback);
    ros::spin();
    cv::destroyWindow("frame");
    return 0;
}