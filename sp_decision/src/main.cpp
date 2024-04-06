#include "iostream"
#include "ros/ros.h"
#include "perception/blackboard.hpp"
#include "tools/log.hpp"
#include "executor/chassis.hpp"
int main(int argc, char **argv)
{
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "sp_decision_node");
    tools::logger::Ptr logger= std::make_shared<tools::logger>();
    sp_decision::Blackboard blackboard(logger);
    sp_decision::ChassisExecutor chassis(logger);
    ros::spin();
    return 0;
}