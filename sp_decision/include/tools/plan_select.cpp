/**
 * @file plan_select.cpp
 * @author lp
 * @brief 获取策略id，在参数服务器中设置后开启决策节点并关闭本程序
 * @version 0.1
 * @date 2024-04-23
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>
#include <vector>
#include "tools/yaml_read.hpp"

int main(int argc, char **argv)
{
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "plan_select");
    ros::NodeHandle nh;
    int param_plan_id=0;
    ros::param::set("plan_id",param_plan_id);
    return 0;
}