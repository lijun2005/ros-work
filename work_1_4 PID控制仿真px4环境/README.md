## 第四周任务介绍
1. 配置px4仿真环境
2. px4与仿真环境之间实现通信
3. 遥控器实现无人机解锁和进入offboard模式
4. 发布姿态命令，实现控制
5. 使用PID，实现仿真环境中无人机的姿态控制
6. 了解无人机的基本姿态

## 基础知识
### 安装px4

### px4与仿真环境比如gazebo之间如何进行通信
在ROS和C++中，可以使用MavROS（Mavlink for ROS）来实现PX4与Gazebo仿真环境之间的通信。MavROS提供了一个ROS接口，用于将PX4飞控系统与ROS进行集成，并允许在ROS中发布和接收来自PX4的消息。

简单来说就是通过mavros
### 无人机基本姿态知识介绍
参考：[无人机姿态介绍](https://github.com/lijun1234567/ros-work/blob/work_1/work_1_2%20mavros%3APositionTarget%20%E5%AD%A6%E4%B9%A0/%E5%9F%BA%E7%A1%80%E7%9F%A5%E8%AF%86)
大概内容如下：
- 坐标系，位置，速度，加速度
- 三个角度：roll(横滚角）翻滚，pitch(俯仰角）点头，yaw(航向角）摇头
  - 绕向前的轴旋转就是横滚角；
  - 绕向右的轴旋转就是俯仰角；
  - 绕向上的轴旋转就是航向角。
### PID控制 
1. [https://blog.csdn.net/Nirvana_Tai/article/details/105409311?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522169080930016800186564344%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=169080930016800186564344&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-2-105409311-null-null.142^v91^koosearch_v1,239^v12^control2&utm_term=PID%E6%8E%A7%E5%88%B6C%E8%AF%AD%E8%A8%80&spm=1018.2226.3001.4187](https://blog.csdn.net/Nirvana_Tai/article/details/105409311?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522169080930016800186564344%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=169080930016800186564344&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-2-105409311-null-null.142^v91^koosearch_v1,239^v12^control2&utm_term=PID%E6%8E%A7%E5%88%B6C%E8%AF%AD%E8%A8%80&spm=1018.2226.3001.4187)
2. [https://www.bilibili.com/video/BV1B54y1V7hp/?spm_id_from=333.337.search-card.all.click&vd_source=797201f7e4343269e3696caf52edcbaa](https://www.bilibili.com/video/BV1B54y1V7hp/?spm_id_from=333.337.search-card.all.click&vd_source=797201f7e4343269e3696caf52edcbaa)
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
- 简单控制
参考于[https://www.guyuehome.com/18778](https://www.guyuehome.com/18778)
```c++
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped local_position;
void position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_position = *msg;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pub_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, position_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
 
    ros::Rate rate(20.0);
 
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
	geometry_msgs::PoseStamped target;
	target.pose.position.x = 2;
	target.pose.position.y = 5;
	target.pose.position.z =6;
 
    geometry_msgs::PoseStamped pose;//姿态控制
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;
    
    geometry_msgs::TwistStamped vel;//速度控制
 
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
 	
 	//起飞
    while(ros::ok())
    {
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
           	last_request = ros::Time::now();
       	}
        else if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }
        else if(ros::Time::now() - last_request > ros::Duration(5.0))
        	break;
 
        local_pos_pub.publish(pose);
		ROS_INFO("1");
        ros::spinOnce();
        rate.sleep();
    }
    
    //逛一圈
	last_request = ros::Time::now();
	while(ros::ok())
	{
		if(ros::Time::now() - last_request > ros::Duration(5.0))
			break;
	    vel.twist.linear.x = 1;
		vel.twist.linear.y = 0;
		vel.twist.linear.z = 0;
		local_vel_pub.publish(vel);
		ROS_INFO("2");
		ros::spinOnce();
    	rate.sleep();
	}
	last_request = ros::Time::now();
	while(ros::ok())
	{
		if(ros::Time::now() - last_request > ros::Duration(5.0))
			break;
	    vel.twist.linear.x = 0;
		vel.twist.linear.y = 1;
		vel.twist.linear.z = 0;
		local_vel_pub.publish(vel);
		ROS_INFO("3");
		ros::spinOnce();
    	rate.sleep();
	}
	last_request = ros::Time::now();
	while(ros::ok())
	{
		if(ros::Time::now() - last_request > ros::Duration(5.0))
			break;
	    vel.twist.linear.x = -1;
		vel.twist.linear.y = 0;
		vel.twist.linear.z = 0;
		local_vel_pub.publish(vel);
		ROS_INFO("4");
		ros::spinOnce();
    	rate.sleep();
	}
	last_request = ros::Time::now();
	while(ros::ok())
	{
		if(ros::Time::now() - last_request > ros::Duration(5.0))
			break;
	    vel.twist.linear.x = 0;
		vel.twist.linear.y = -1;
		vel.twist.linear.z = 0;
		local_vel_pub.publish(vel);
		ros::spinOnce();
    	rate.sleep();
	}
	
    
    offb_set_mode.request.custom_mode = "AUTO.LAND";
    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
    {
        ROS_INFO("Offboard enabled");
        last_request = ros::Time::now();
    }
 
    return 0;
}



```
