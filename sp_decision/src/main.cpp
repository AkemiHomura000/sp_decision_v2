#include "iostream"
#include "ros/ros.h"
#include "perception/blackboard.hpp"
#include "tools/log.hpp"
#include "executor/chassis.hpp"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sp_decision_node");
    tools::logger::Ptr logger= std::make_shared<tools::logger>();
    sp_decision::Blackboard blackboard(logger);
    sp_decision::ChassisExecutor chassis(logger);
    chassis.sendGoal(1,1);
    ros::spin();
    return 0;
}