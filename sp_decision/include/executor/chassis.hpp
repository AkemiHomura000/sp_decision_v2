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

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <robot_msg/RobotStateMsg.h>

#include "robot_msg/CmdGimbal.h"
#include "tools/log.hpp"
namespace sp_decision
{ // 给MoveBaseAction定义一个别名，方便创建对象
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client;
    class ChassisExecutor
    {
    public:
        ChassisExecutor(const tools::logger::Ptr &logger_ptr);
        typedef std::shared_ptr<ChassisExecutor> Ptr;
        void sendGoal(double x, double y, double w = 1);

    private:
        tools::logger::Ptr logger_ptr_;
        ros::NodeHandle nh_;
        move_base_msgs::MoveBaseGoal goal_;
        move_base_client move_base_ac_;
    };
}
#endif