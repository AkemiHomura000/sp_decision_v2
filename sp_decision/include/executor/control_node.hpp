#ifndef CONTROL_NODE_H
#define CONTROL_NODE_H
#include "executor/chassis.hpp"
#include "executor/gimbal.hpp"
#include "robot_msg/DecisionMsg.h"
#include "perception/blackboard.hpp"
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include "tools/yaml_read.hpp"
namespace sp_decision
{
    class ControlNode
    {
    public:
        typedef std::shared_ptr<ControlNode> Ptr;
        std::mutex decision_cbk_mutex;
        ControlNode(const Blackboard::Ptr &blackboard_ptr, const tools::logger::Ptr &logger_ptr);
        ~ControlNode();
        void decision_sub(const robot_msg::DecisionMsg &decision);
        void execute_decision(); // 执行决策
        void points_init();      // 读取点集列表
        void run();              // 动作状态跟踪及更新
        void run_start();        // 启动决策执行器
        /**
         * @brief 决策一：冲锋
         *
         */
        void charge();

    private:
        std::vector<Eigen::Vector2d> points_;
        bool control_thread_running;
        std::thread control_thread_;
        std::string decision_;      // 决策类别
        std::string last_decision_; // 决策类别
        ros::NodeHandle nh_;
        ros::Subscriber decision_sub_;                  // 决策信息订阅
        int loop_rate;                                  // 状态维护频率
        sp_decision::ChassisExecutor::Ptr chassis_ptr_; // 底盘控制
        sp_decision::GimbalExecutor::Ptr gimbal_ptr_;   // 云台控制
        sp_decision::Blackboard::Ptr blackboard_ptr_;   
        tools::yaml_reader::Ptr yaml_reader_ptr_;
        tools::logger::Ptr logger_ptr_;
    };
}
#endif