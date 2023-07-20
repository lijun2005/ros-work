#include <iostream>
#include <random>
#include <vector>
#include <ros/ros.h>
#include <mavros_msgs/RCIn.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <thread>

int transport (float m);
void pic_hist( std::vector<float> arry);

int transport (int m)
{
    return (int)((m-1500)/4.63);
}

void callback(const mavros_msgs::RCIn::ConstPtr& msg)
{

    cv::Mat image(600, 800, CV_8UC3, cv::Scalar(255, 255, 255));

    std::vector<int> arry;
    for(int i=0;i <=4;i++)
    {
        arry.push_back(msg->channels[i]);
    }

    // 设置绘制直方图的参数
    int bar_width = 80;
    int gap = 30;
    int x = 50;
    int y = 300;
    cv::Scalar color(0, 0, 255);

    // 绘制直方柱形图
    for (int i = 0; i < arry.size(); ++i) {
        int height =transport(arry[i]);

        cv::Point pt1(x, y);
        cv::Point pt2(x + bar_width, y - height);
        cv::Point pt3(x,100);
        cv::Scalar color1(0,0,0);
        std::string text = "num: " + std::to_string(height);
        cv::rectangle(image, pt1, pt2, color, -1); // -1表示实心矩形
        cv::putText(image,text,pt3,1,1.0,color1);
        x += bar_width + gap;
    }
    cv::namedWindow("image");
    // 显示图像
    cv::imshow("image", image);
    cv::waitKey(30);
    arry.clear();
    //cv::destroyWindow("image");
}

int main(int argc , char** argv)
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nn;
    ros::Subscriber sub = nn.subscribe("/mavros/rc/in",10,callback);
    ros::spin();
    return 0;

}