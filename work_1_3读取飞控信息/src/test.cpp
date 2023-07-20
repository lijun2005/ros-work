#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <mavros_msgs/RCIn.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

int transport (float m);
void pic_hist( std::vector<float> arry);
// void callback(const mavros_msgs::RCIn::ConstPtr& msg)
// {


//     打印油门通道
//      int i=0;
//      ROS_INFO("channel %d : %d ", i, msg->channels[i]);
// }

// void callback()
// {

// 
void pic_hist( std::vector<float> arry)
{
    cv::Mat image(600, 800, CV_8UC3, cv::Scalar(255, 255, 255));


    // 计算直方图的最大值
    int max_value = transport(*std::max_element(arry.begin(), arry.end()));

    // 设置绘制直方图的参数
    int bar_width = 80;
    int gap = 30;
    int x = 50;
    int y = 300;
    cv::Scalar color(0, 0, 255);

    // 绘制直方柱形图
    for (int i = 0; i < arry.size(); ++i) {
        int height = cvRound(static_cast<double>(transport(arry[i])));

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
    //cv::destroyWindow("image");
}
int transport (float m)
{
    return (int)((m-1500)/4.63);
}
int main(int argc , char** argv)
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nn;
    //ros::Subscriber sub = nn.subscribe("/mavros/rc/in",10,callback);
    std::random_device rd;
    std::mt19937 generator(rd());

    float lowerBound = 1037; // 下界（范围的最小值）
    float upperBound = 1963; // 上界（范围的最大值）

    // 定义一个随机数分布，指定范围为 [lowerBound, upperBound]
    std::uniform_real_distribution<float> distribution(lowerBound, upperBound);
    std::vector<float> arr;
    // 生成随机数
    int j=0;
    while(j<=100)
    {
        for(int i=0;i <=4;i++)
        {
            float randomNum = distribution(generator);
            arr.push_back(randomNum);
        }
        for( auto a: arr)
        {
            std::cout<< transport(a) << " " <<std::endl;
        }
        pic_hist(arr);
        j++;
        std::chrono::seconds duration(1);
        std::this_thread::sleep_for(duration);
        arr.clear();
    }
    //ros::spin();
    return 0;

}
