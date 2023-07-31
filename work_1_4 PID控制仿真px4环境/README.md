## 第四周任务介绍
1. 配置px4仿真环境
2. px4与仿真环境之间实现通信
3. 遥控器实现无人机解锁和进入offboard模式
4. 发布姿态命令，实现控制
5. 使用PID，实现仿真环境中无人机的姿态控制
6. 了解无人机的基本姿态

## 基础知识
### px4与仿真环境比如gazebo之间如何进行通信
在ROS和C++中，可以使用MavROS（Mavlink for ROS）来实现PX4与Gazebo仿真环境之间的通信。MavROS提供了一个ROS接口，用于将PX4飞控系统与ROS进行集成，并允许在ROS中发布和接收来自PX4的消息。

简单来说就是通过mavros
### 无人机姿态介绍
[]()
### PID控制算法
参考[https://zhuanlan.zhihu.com/p/39573490](https://zhuanlan.zhihu.com/p/39573490)

PID控制算法是一种经典的反馈控制算法，用于调节系统的输出，使其接近期望值或目标值。PID控制器根据系统当前的误差和误差变化率，计算出控制输出，以实现系统的稳定性和准确性。

PID控制器由三个部分组成：比例（Proportional）、积分（Integral）和微分（Derivative），分别用P、I和D表示。控制输出可以通过以下公式计算：

控制输出 = Kp * P + Ki * I + Kd * D

其中：

- P代表比例项，是当前误差的乘以一个比例常数Kp，用于产生直接与误差成正比的控制输出。
- I代表积分项，是误差的累积值乘以一个积分常数Ki，用于消除系统的静态误差。
- D代表微分项，是误差变化率的负数乘以一个微分常数Kd，用于预测系统未来的误差变化趋势。

PID控制器的工作原理如下：

1. 首先，计算当前误差，即期望值与实际值之间的差异。
2. 然后，根据比例项、积分项和微分项的权重，计算出控制输出。
3. 将控制输出应用到系统中，使其产生反馈作用。
4. 反复执行上述步骤，使得系统逐渐稳定在期望值附近。
可以利用高中生物所学的反馈调节的原理来解释，也可以用电路里面的放大电路里面的选取合适静态工作点的思想。

视频推荐：
1. [https://www.bilibili.com/video/BV1B54y1V7hp/?p=2&spm_id_from=pageDriver&vd_source=797201f7e4343269e3696caf52edcbaa](https://www.bilibili.com/video/BV1B54y1V7hp/?p=2&spm_id_from=pageDriver&vd_source=797201f7e4343269e3696caf52edcbaa)
2. [https://www.bilibili.com/video/BV1zM4y157pk/?spm_id_from=333.999.0.0&vd_source=797201f7e4343269e3696caf52edcbaa](https://www.bilibili.com/video/BV1zM4y157pk/?spm_id_from=333.999.0.0&vd_source=797201f7e4343269e3696caf52edcbaa)
## 代码实现
- 手动实现无人机解锁并进入offboard模式
  - 官方文档:[https://docs.px4.io/main/en/ros/mavros_offboard_cpp.html](https://docs.px4.io/main/en/ros/mavros_offboard_cpp.html)
```c++
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

```
- 使用PID对飞机的横滚角和俯仰角进行控制
```c++
```
