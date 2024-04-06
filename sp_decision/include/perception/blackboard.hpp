/**
 * @file blackboard.hpp
 * @author lp
 * @brief 感知信息汇总，包含自身状态：导航状态，其他机器人状态：血量+位置
 * @version 0.1
 * @date 2024-04-06
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef BLACKBOARD_H
#define BLACKBOARD_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <actionlib_msgs/GoalStatusArray.h>

#include <Eigen/Geometry>
#include <cmath>

#include <robot_msg/RobotHP.h>
#include <robot_msg/Log.h>
#include <robot_msg/MatchMsg.h>
#include <robot_msg/EnemyStage.h>
#include <robot_msg/RefereeInfoMsg.h>
#include <robot_msg/Armor.h>

#include <tools/log.hpp>
namespace sp_decision
{
    class Blackboard
    {
    public:
        typedef std::shared_ptr<Blackboard> Ptr;
        Blackboard(const tools::logger::Ptr &logger_ptr);
        ~Blackboard() {}
        std::mutex match_status_cbk_mutex;
        std::mutex sentry_status_cbk_mutex;
        std::mutex referee_data_cbk_mutex;
        std::mutex move_base_status_cbk_mutex;
        std::mutex nav_cmd_vel_cbk_mutex;
        std::mutex goal_status_mutex;
        std::mutex referee_info_mutex;
        std::mutex enemy_status_cbk_mutex;
        void ResetFlag();

        // 云台手发布坐标
        struct Point
        {
            double x;
            double y;
        };
        /**
         * @brief 机器人状态结构体
         * TODO 两种复活状态的区分
         */
        enum ROBOT_STATUS
        {
            dead,
            revive_3s,
            revive_10s,
            live,
        };
        struct RobotStatus
        {
            int robot_id;               // 机器人编号，从1～8为英雄、工程、步兵*3、哨兵、前哨站、基地
            Eigen::Vector2d robot_pos;  // 机器人当前位置
            ros::Time pos_update_time;  // 机器人位置更新时间
            uint16_t robot_hp;          // 机器人血量
            ROBOT_STATUS robot_status;  // 机器人状态:0为死亡，1为复活无敌，2为存活
            ros::Time last_revive_time; // 上次复活时间
        };
        // 友方状态
        std::vector<RobotStatus> teammate_status;
        // 敌方状态
        std::vector<RobotStatus> enemy_status;
        // 比赛状态
        struct match_status
        {
            uint8_t game_progress;      // 比赛进程
            uint16_t stage_remain_time; // 当前阶段剩余时间
            bool rfid_remedy_state;     // rfid激活状态
            bool rfid_centerpoint_state;
        };
        match_status match_status;
        // 比赛阶段判定
        enum class MatchSatuts
        {
            TO_BEGIN,
            AT_MATCH,
            AFTER_MATCH
        };
        MatchSatuts game_status = MatchSatuts::TO_BEGIN;
        // 自身状态,便于调用,TODO:导航状态
        struct sentry_status
        {
            int sentry_hp;                 // 机器人血量
            nav_msgs::Odometry robot_pose; // 机器人位置
        };
        sentry_status sentry_status;
        //--------------------------------------------------------------------------------------------------------------------------------//

        ros::Time time_received_armor_;
        robot_msg::Armor armor_;

        /**
         * @brief 对方血量,联盟赛使用
         */
        int enemy_hp_[4];                         // 依次是基地，哨兵，单位一，单位二
        int enemy_stage_[4];                      // 机器人状态，0--死亡，1--复活无敌状态，2--正常存活
        int enemy_number[2] = {-1, -1};           // 英雄为1，步兵3，4，5
        std::vector<ros::Time> enemy_revive_time; // 用于复活时间倒计时
        /**
         * @brief
         */
        float posx_x_;
        float posy_y_;
        float key_z_;
        /**
         * @brief move_base
         */
        geometry_msgs::Twist vel_msg_sub_;

        // 动作状态
        enum class Action_Lock
        {
            ADD_BLOOD,
            BACKWARD_DEFENCE,
            RETREAT,
            PURSUIT,
            ATTACK,
            PATROL,
            JUDGING,
        };
        Action_Lock action_status_ = Action_Lock::JUDGING;

        /**
         *@brief 决策所需变量
         */
        int available_hp_ = 600; // 剩余可加血量?
        bool plan_get_ = 0;      // 是否规划出路径规划出路径
        bool base_attacked_ = 0; // 基地受击状态?
        ros::Time current_time;  // 用于基地受击状态更新?
        int current_hp;
        bool status_init = 0; // 状态初始化
        ros::Time time_1;
        bool sentry_attacked_ = 0;          // 烧饼受击状态
        bool attacked_violently_ = 0;       // 掉血速度过快
        bool armor_received_ = 0;           // 装甲板接收状态
        geometry_msgs::Point last_position; // 记录位置
        bool attack = 0;                    // 对方死一人后更新为1
        ros::Publisher log_pub_;
        void LogPub(std::string str);

    private:
        tools::logger::Ptr logger_ptr_;
        
        ros::NodeHandle nh_;
        ros::Subscriber referee_info_sub_;
        ros::Subscriber enemy_hp_sub_;
        ros::Subscriber sentry_odom_sub_;
        ros::Subscriber enemy_pos_sub_; // 接收敌方坐标
        ros::Publisher enemy_status_pub_;

        ros::Subscriber referee_data_sub_;
        ros::Subscriber move_base_status_sub_;
        ros::Subscriber cmd_vel_sub_;
        ros::Subscriber goal_status_sub_;
        ros::Subscriber armor_sub_;

        void enemy_status_advertise(); // 发布无敌状态的目标
        void robot_init();             // 初始化机器人列表
        void referee_info_callback(const robot_msg::RefereeInfoMsg::ConstPtr &msg);
        void enemy_hp_callback(const robot_msg::RobotHP::ConstPtr &msg);
        void sentry_pose_callback(const nav_msgs::Odometry::ConstPtr msg);
        void enemy_pose_callback(const robot_msg::Armor::ConstPtr &msg);
    };
} // namespace sp_decision
#endif