#include "executor/chassis.hpp"
namespace sp_decision
{
    ChassisExecutor::ChassisExecutor(const tools::logger::Ptr &logger_ptr) : move_base_ac_("move_base", true)
    {
        logger_ptr_ = logger_ptr; // 获取日志器
        ROS_INFO("等待 move_base 动作服务器");
        move_base_ac_.waitForServer(ros::Duration(60));
        ROS_INFO("连接到 move_base 服务器");
    }
    void ChassisExecutor::sendGoal(double x, double y, double w)
    {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;
        goal.target_pose.pose.orientation.w = w;

        // 发送目标
        ROS_INFO("发送目标");
        move_base_ac_.sendGoal(goal);

        while (move_base_ac_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        { // 打印执行结果
            ROS_INFO("动作状态：%s", move_base_ac_.getState().toString().c_str());
            std::stringstream ss;
            ss << move_base_ac_.getState().toString().c_str();
            logger_ptr_->logInfo(ss);
        }
    }
}