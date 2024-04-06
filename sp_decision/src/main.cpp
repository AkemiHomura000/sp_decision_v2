#include "iostream"
#include "ros/ros.h"
#include "perception/blackboard.hpp"
#include "tools/log.hpp"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sp_decision_node");
    tools::logger log;
    for (int i = 0; i < 100; i++)
    {
        std::stringstream ss;
        ss << "hello " << i;
        log.logDebug(ss);
        log.logError(ss);
    }
    sp_decision::Blackboard blackboard;
    ros::spin();
    return 0;
}