/**
 * @file chassis.hpp
 * @author lp
 * @brief
 * @version 0.1
 * @date 2024-04-06
 * @copyright Copyright (c) 2024
 */
#ifndef CHASSIS_EXECUTOR_H
#define CHASSIS_EXECUTOR_H

#include <ros/ros.h>
#include <cmath>

#include <nav_msgs/Odometry.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <robot_msg/RobotStateMsg.h>

#include "robot_msg/CmdGimbal.h"
#include "tools/log.hpp"
namespace sp_decision
{
    // 给MoveBaseAction定义一个别名，方便创建对象
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client;
    class ChassisExecutor
    {
    public:
        std::mutex nav_cmd_vel_cbk_mutex;
        std::mutex localization_cbk_mutex;
        enum RobotState
        {
            MOVE,
            CRUISR,
            IDLE,
            FAST,
            STOP,
            ROTATE,
            PURSUIT,
            SlOW,
        };
        ChassisExecutor(const tools::logger::Ptr &logger_ptr);
        typedef std::shared_ptr<ChassisExecutor> Ptr;
        int send_goal(double pos_x, double pos_y, double w = 1); // 0为失败，1为进行中，2为成功到达
        void directly_send_goal(double pos_x, double pos_y, double w = 1); // 直接发布目标点
        void robot_state_pub(RobotState robot_state);
        void cmd_vel_callback(const geometry_msgs::Twist &msg);             // 接收move_base规划的速度
        void localization_callback(const nav_msgs::Odometry::ConstPtr msg); // 接受定位信息
    private:
        geometry_msgs::Twist cmd_vel_;    // move_base规划的速度
        geometry_msgs::Twist last_cmd_vel_;    // move_base规划的速度
        nav_msgs::Odometry localization_; // 定位信息
        ros::Time last_judge_time_;       // 判断路径是否存在的计时器
        int nav_status_;                  // 导航状态
        tools::logger::Ptr logger_ptr_;
        ros::NodeHandle nh_;
        ros::Publisher set_goal_pub_;      // 导航目标发布
        ros::Publisher robot_state_pub_;   // 导航模式发布
        ros::Subscriber cmd_vel_sub_;      // move_base速度规划器
        ros::Subscriber localization_sub_; // 接受定位信息
        geometry_msgs::PoseStamped target_pose_;
        move_base_msgs::MoveBaseGoal goal_;
    };
}
#endif