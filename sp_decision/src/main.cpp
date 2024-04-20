#include "iostream"
#include "ros/ros.h"
#include "perception/blackboard.hpp"
#include "tools/log.hpp"
#include "tools/yaml_read.hpp"
#include "executor/chassis.hpp"
#include "decision_node/decision_tree.hpp"
int main(int argc, char **argv)
{
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "sp_decision_node");
    tools::logger::Ptr logger = std::make_shared<tools::logger>();
    tools::yaml_reader::Ptr yaml_read = std::make_shared<tools::yaml_reader>("/home/lp1/sp_nav_ws/src/sp_nav/sp_decision/config/test.yaml");
    sp_decision::Blackboard::Ptr blackboard = std::make_shared<sp_decision::Blackboard>(logger);
    sp_decision::ChassisExecutor::Ptr chassis = std::make_shared<sp_decision::ChassisExecutor>(logger);
    std::vector<Eigen::Vector2d> points;

    // 添加点到容器中
    points.push_back(Eigen::Vector2d(0, 0));
    points.push_back(Eigen::Vector2d(5, 0));
    points.push_back(Eigen::Vector2d(5, 5));
    points.push_back(Eigen::Vector2d(3, 5));
    while (ros::ok)
    {
        Eigen::Vector2d point = chassis->point_generate(points);
        std::cout << "point:" << point[0] << "," << point[1] << std::endl;
        //ros::Duration(0.1).sleep();
    }
    // sp_decision::decision_tree decison_tree_0(yaml_read, blackboard);
    //  decison_tree_0.print_tree();
    //  decison_tree_0.run_start();
    //  sp_decision::ChassisExecutor chassis(logger);
    ros::spin();
    return 0;
}