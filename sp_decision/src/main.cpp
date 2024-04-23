#include "iostream"
#include "ros/ros.h"
#include "perception/blackboard.hpp"
#include "tools/log.hpp"
#include "tools/yaml_read.hpp"
#include "executor/chassis.hpp"
#include "executor/gimbal.hpp"
#include "decision_node/decision_tree.hpp"
#include "executor/control_node.hpp"
int main(int argc, char **argv)
{
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "sp_decision_node");
    tools::logger::Ptr logger = std::make_shared<tools::logger>();
    sp_decision::Blackboard::Ptr blackboard = std::make_shared<sp_decision::Blackboard>(logger);
    sp_decision::ChassisExecutor::Ptr chassis = std::make_shared<sp_decision::ChassisExecutor>(logger, blackboard);
    // sp_decision::GimbalExecutor::Ptr gimbal = std::make_shared<sp_decision::GimbalExecutor>(logger);
    // sp_decision::ControlNode::Ptr control_node = std::make_shared<sp_decision::ControlNode>(blackboard,chassis, gimbal, logger);
    // sp_decision::decision_tree decison_tree_0( blackboard);
    // decison_tree_0.print_tree();
    // decison_tree_0.run_start();
    ros::Rate loop_rate(10);
    while(ros::ok)
    {
        Eigen::Vector2d point_1,point_2;
        point_1<<1,2;
        point_2<<0,0.5;
        chassis->single_point_move(point_1,point_2);
    }
    ros::spin();
    return 0;
}