#include "perception/blackboard.hpp"

namespace sp_decision
{
    Blackboard::Blackboard(const tools::logger::Ptr &logger_ptr)
    {
        logger_ptr_ = logger_ptr; // 获取日志器
        robot_init();
        referee_info_sub_ =
            nh_.subscribe("referee_info", 1, &Blackboard::referee_info_callback, this);
        enemy_hp_sub_ =
            nh_.subscribe("Enemy_robot_HP", 10, &Blackboard::enemy_hp_callback, this);
        sentry_odom_sub_ =
            nh_.subscribe("localization", 1, &Blackboard::sentry_pose_callback, this);
        enemy_pos_sub_ =
            nh_.subscribe("/enemy_pose", 1, &Blackboard::enemy_pose_callback, this);
        enemy_status_pub_ =
            nh_.advertise<robot_msg::EnemyStage>("/enemy_stage", 1);
        log_pub_ =
            nh_.advertise<robot_msg::EnemyStage>("/sentry/log", 1);
    }

    // void Blackboard::GoalStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg)
    // {
    //     goal_status_mutex.lock();
    //     plan_get_ = false;
    //     for (const auto &status : msg->status_list)
    //     {
    //         if (status.status == 1)
    //         {
    //             // ROS_INFO("move_base ok");
    //             plan_get_ = true;
    //         }
    //     }
    //     goal_status_mutex.unlock();
    // }

    void Blackboard::referee_info_callback(const robot_msg::RefereeInfoMsg::ConstPtr &msg)
    {
        referee_info_mutex.lock();
        // 更新消息
        sp_decision::Blackboard::match_status.game_progress = msg->game_progress;
        sp_decision::Blackboard::match_status.stage_remain_time = msg->stage_remain_time;
        sp_decision::Blackboard::match_status.rfid_remedy_state = msg->rfid_remedy_state;
        sp_decision::Blackboard::match_status.rfid_centerpoint_state = msg->rfid_centerpoint_state;
        // 判断比赛进程
        if (match_status.game_progress < 4)
            game_status = MatchSatuts::TO_BEGIN;
        if (match_status.game_progress == 5)
            game_status = MatchSatuts::AFTER_MATCH;
        if (match_status.game_progress == 4 && match_status.stage_remain_time <= 419)
            game_status = MatchSatuts::AT_MATCH;
        // // 更新基地受击状态?
        // if (base_HP_ > msg->base_HP || base_attacked_)
        // {
        //     base_attacked_ = true;
        //     // ROS_INFO("base -----------------------------------------------------:%hu",  msg->base_HP);
        //     if (base_HP_ == msg->base_HP)
        //     {
        //         ros::Time time = ros::Time::now();
        //         if ((time.sec - current_time.sec) > 15) // 频率待确定?
        //         {
        //             base_attacked_ = false;
        //             current_time = ros::Time::now();
        //         }
        //     }
        //     if (base_HP_ > msg->base_HP)
        //     {
        //         current_time = ros::Time::now();
        //     }
        // }
        // // 更新可用血量?
        // if (robot_hp_ < msg->robot_HP)
        // {
        //     available_hp_ -= (msg->robot_HP - robot_hp_);
        // }
        // // 更新烧饼受击状态?
        // if (robot_hp_ > msg->robot_HP || status_init || attacked_violently_)
        // {
        //     if (!status_init)
        //     {
        //         time_1 = ros::Time::now();
        //         current_hp = msg->robot_HP;
        //         status_init = 1;
        //     }
        //     if (ros::Time::now().sec - time_1.sec > 3) // 3s更新一次?
        //     {
        //         if (current_hp - msg->robot_HP > 90)
        //         {
        //             attacked_violently_ = true;
        //         }
        //         else
        //         {
        //             attacked_violently_ = false;
        //         }
        //         status_init = 0;
        //     }
        // }
        // robot_hp_ = msg->robot_HP;
        referee_info_mutex.unlock();
    }
    void Blackboard::robot_init()
    {
        for (int i = 0; i < 8; i++)
        {
            sp_decision::Blackboard::RobotStatus status;
            status.robot_id = i + 1;
            status.robot_hp = 400; // 初始化血量为400
            status.robot_status = sp_decision::Blackboard::ROBOT_STATUS::live;
            status.robot_pos(0) = 0;
            status.robot_pos(1) = 0;
            sp_decision::Blackboard::teammate_status.push_back(status);
        }
        for (int i = 0; i < 8; i++)
        {
            sp_decision::Blackboard::RobotStatus status;
            status.robot_id = i + 1;
            status.robot_hp = 400; // 初始化血量为400
            status.robot_status = sp_decision::Blackboard::ROBOT_STATUS::live;
            status.robot_pos(0) = 0;
            status.robot_pos(1) = 0;
            sp_decision::Blackboard::enemy_status.push_back(status);
        }
    }
    void Blackboard::sentry_pose_callback(const nav_msgs::Odometry::ConstPtr msg)
    {
        sentry_status_cbk_mutex.lock();
        Blackboard::sentry_status.robot_pose = *msg;
        sentry_status_cbk_mutex.unlock();
    }

    void Blackboard::enemy_pose_callback(const robot_msg::Armor::ConstPtr &msg)
    {
        enemy_status_cbk_mutex.lock();
        int num = std::stoi(msg->number);
        Blackboard::enemy_status[num - 1].robot_pos(0) = msg->pose.position.x;
        Blackboard::enemy_status[num - 1].robot_pos(1) = msg->pose.position.y;
        enemy_status[num - 1].pos_update_time = ros::Time::now();
        enemy_status_cbk_mutex.unlock();
    }
    // void Blackboard::CmdVelDataCallback(const geometry_msgs::Twist &msg)
    // {
    //     nav_cmd_vel_cbk_mutex.lock();
    //     vel_msg_sub_.linear.x = msg.linear.x;
    //     vel_msg_sub_.linear.y = msg.linear.y;
    //     vel_msg_sub_.angular.z = msg.angular.z;
    //     nav_cmd_vel_cbk_mutex.unlock();
    // }
    void Blackboard::enemy_hp_callback(const robot_msg::RobotHP::ConstPtr &msg)
    {
        enemy_status_cbk_mutex.lock();
        enemy_status[0].robot_hp = msg->Hero_HP;
        enemy_status[1].robot_hp = msg->Engineer_HP;
        enemy_status[2].robot_hp = msg->Infantry_3_HP;
        enemy_status[3].robot_hp = msg->Infantry_4_HP;
        enemy_status[4].robot_hp = msg->Infantry_5_HP;
        enemy_status[5].robot_hp = msg->Sentry_HP;
        enemy_status[6].robot_hp = msg->OutPose_HP;
        enemy_status[7].robot_hp = msg->Base_HP;
        if (game_status == MatchSatuts::AT_MATCH) // 比赛开始后再更新敌方复活状态,TODO:哨兵无敌时间的判定
        {
            for (int i = 0; i < 5; i++)
            {
                switch (enemy_status[i].robot_status) // 判定无敌状态的fsm,仅适用超级对抗赛！！！
                {
                case ROBOT_STATUS::dead:
                    if (enemy_status[i].robot_hp == 0)
                        enemy_status[i].robot_status = ROBOT_STATUS::dead;
                    if (enemy_status[i].robot_hp > 100)
                    {
                        enemy_status[i].last_revive_time = ros::Time::now();
                        enemy_status[i].robot_status = ROBOT_STATUS::revive_3s;
                    }
                    if (enemy_status[i].robot_hp < 100 && enemy_status[i].robot_hp > 0)
                    {
                        enemy_status[i].last_revive_time = ros::Time::now();
                        enemy_status[i].robot_status = ROBOT_STATUS::revive_10s;
                    }
                    break;
                case ROBOT_STATUS::revive_3s:
                    if (enemy_status[i].robot_hp == 0)
                        enemy_status[i].robot_status = ROBOT_STATUS::dead;
                    if (-(enemy_status[i].last_revive_time.sec - ros::Time::now().sec) < 3)
                        enemy_status[i].robot_status = ROBOT_STATUS::revive_3s;
                    if (-(enemy_status[i].last_revive_time.sec - ros::Time::now().sec) > 3)
                        enemy_status[i].robot_status = ROBOT_STATUS::live;
                    break;
                case ROBOT_STATUS::revive_10s:
                    if (enemy_status[i].robot_hp == 0)
                        enemy_status[i].robot_status = ROBOT_STATUS::dead;
                    if (-(enemy_status[i].last_revive_time.sec - ros::Time::now().sec) < 10)
                        enemy_status[i].robot_status = ROBOT_STATUS::revive_10s;
                    if (-(enemy_status[i].last_revive_time.sec - ros::Time::now().sec) > 10)
                        enemy_status[i].robot_status = ROBOT_STATUS::live;
                    break;
                case ROBOT_STATUS::live:
                    if (enemy_status[i].robot_hp == 0)
                        enemy_status[i].robot_status = ROBOT_STATUS::dead;
                    else
                        enemy_status[i].robot_status = ROBOT_STATUS::live;
                    break;
                default:
                    break;
                }
            }
            switch (enemy_status[5].robot_status) // 判定无敌状态的fsm,哨兵均视为3s无敌
            {
            case ROBOT_STATUS::dead:
                if (enemy_status[5].robot_hp == 0)
                    enemy_status[5].robot_status = ROBOT_STATUS::dead;
                if (enemy_status[5].robot_hp > 100)
                {
                    enemy_status[5].last_revive_time = ros::Time::now();
                    enemy_status[5].robot_status = ROBOT_STATUS::revive_3s;
                }
                break;
            case ROBOT_STATUS::revive_3s:
                if (enemy_status[5].robot_hp == 0)
                    enemy_status[5].robot_status = ROBOT_STATUS::dead;
                if (-(enemy_status[5].last_revive_time.sec - ros::Time::now().sec) < 3)
                    enemy_status[5].robot_status = ROBOT_STATUS::revive_3s;
                if (-(enemy_status[5].last_revive_time.sec - ros::Time::now().sec) > 3)
                    enemy_status[5].robot_status = ROBOT_STATUS::live;
                break;
            case ROBOT_STATUS::live:
                if (enemy_status[5].robot_hp == 0)
                    enemy_status[5].robot_status = ROBOT_STATUS::dead;
                else
                    enemy_status[5].robot_status = ROBOT_STATUS::live;
                break;
            default:
                break;
            }
        }
        enemy_status_cbk_mutex.unlock();
        enemy_status_advertise();
    }
    void Blackboard::enemy_status_advertise()
    {
        std::stringstream ss; // 向自瞄发送无敌目标
        ss << "0";
        for (int i = 0; i < 6; i++)
        {
            if (enemy_status[i].robot_status == ROBOT_STATUS::revive_10s || enemy_status[i].robot_status == ROBOT_STATUS::revive_3s)
            {
                ss << "," << enemy_status[i].robot_id;
            }
        }
        robot_msg::EnemyStage enemy__;
        std::string binary_string = ss.str();
        enemy__.ss = binary_string;
        enemy_status_pub_.publish(enemy__);
        logger_ptr_->logInfo(ss);
    }
} // namespace sp_decision