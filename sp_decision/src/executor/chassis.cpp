#include "executor/chassis.hpp"
namespace sp_decision
{
    ChassisExecutor::ChassisExecutor(const tools::logger::Ptr &logger_ptr)
    {
        logger_ptr_ = logger_ptr; // 获取日志器
        set_goal_pub_ =
            nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
        robot_state_pub_ =
            nh_.advertise<robot_msg::RobotStateMsg>("/robot_state", 1);
        cmd_vel_sub_ =
            nh_.subscribe("cmd_vel", 10, &ChassisExecutor::cmd_vel_callback, this);

        target_pose_.pose.position.x = 100000;
        target_pose_.pose.position.y = 100000; // 确保初值不会和第一个点冲突--------待优化
    }
    void ChassisExecutor::cmd_vel_callback(const geometry_msgs::Twist &msg)
    {
        nav_cmd_vel_cbk_mutex.lock();
        cmd_vel_.linear.x = msg.linear.x;
        cmd_vel_.linear.y = msg.linear.y;
        cmd_vel_.angular.z = msg.angular.z;
        nav_cmd_vel_cbk_mutex.unlock();
    }
    void ChassisExecutor::localization_callback(const nav_msgs::Odometry::ConstPtr msg)
    {
        localization_cbk_mutex.lock();
        localization_ = *msg;
        localization_cbk_mutex.unlock();
    }
    void ChassisExecutor::robot_state_pub(RobotState robot_state)
    {
        robot_msg::RobotStateMsg robot_state_msg;
        robot_state_msg.robot_state = static_cast<int8_t>(robot_state);
        robot_state_pub_.publish(robot_state_msg);
    }
    int ChassisExecutor::send_goal(double pos_x, double pos_y, double w)
    {
        // 距离小于0.1m认为到达
        if ((sqrt(pow(pos_x - localization_.pose.pose.position.x, 2) + pow(pos_y - localization_.pose.pose.position.y, 2))) < 0.1)
        {
            return 2;
        }
        else
        {
            if (target_pose_.pose.position.x == pos_x && target_pose_.pose.position.y == pos_y)
            {
                ros::Duration time_interval = ros::Time::now() - last_judge_time_;
                double time_interval_ms = time_interval.toSec() * 1000.0; // 计算时间间隔
                if (time_interval_ms > 100)                               // 每100ms进行一次判断
                {
                    if ((cmd_vel_.linear.x == last_cmd_vel_.linear.x && cmd_vel_.linear.y == last_cmd_vel_.linear.y) ||
                        (cmd_vel_.linear.x == 0 && cmd_vel_.linear.y == 0))
                    {
                        last_judge_time_ = ros::Time::now();
                        last_cmd_vel_.linear.x = cmd_vel_.linear.x;
                        last_cmd_vel_.linear.y = cmd_vel_.linear.y;
                        directly_send_goal(pos_x, pos_y, w);
                        return 0;
                    }
                    else
                    {
                        last_judge_time_ = ros::Time::now();
                        last_cmd_vel_.linear.x = cmd_vel_.linear.x;
                        last_cmd_vel_.linear.y = cmd_vel_.linear.y;
                        directly_send_goal(pos_x, pos_y, w);
                        return 1;
                    }
                }
                else
                {
                    if (nav_status_ == 0)
                    {
                        nav_status_ = 0;
                        return 0;
                    }
                    else
                    {
                        return 1;
                    }
                }
            }
            else
            {
                last_judge_time_ = ros::Time::now();
                last_cmd_vel_.linear.x = cmd_vel_.linear.x;
                last_cmd_vel_.linear.y = cmd_vel_.linear.y;
                directly_send_goal(pos_x, pos_y, w);
                nav_status_ = 1;
                return 1;
            }
        }
    }
    void ChassisExecutor::directly_send_goal(double pos_x, double pos_y, double w)
    {
        target_pose_.header.frame_id = "map";
        target_pose_.header.stamp = ros::Time::now();
        target_pose_.pose.position.x = pos_x;
        target_pose_.pose.position.y = pos_y;
        target_pose_.pose.orientation.x = 0.0;
        target_pose_.pose.orientation.y = 0.0;
        target_pose_.pose.orientation.z = 0.0;
        target_pose_.pose.orientation.w = w;
        goal_.target_pose = target_pose_;
        set_goal_pub_.publish(goal_);
    }
}