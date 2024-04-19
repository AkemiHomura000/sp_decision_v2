// /**
//  * @file pcl2process.cpp
//  * @author lp
//  * @brief 转发/cloud_registered_body为/pointcloud2_out。实现点云与先验地图的匹配，根据先验地图过滤掉坡面(先判定xy范围，再根据角度确定是否属于坡面)
//  * @version 0.1
//  * @date 2024-04-17
//  *
//  * @copyright Copyright (c) 2024
//  *
//  */
// #include <algorithm>
// #include <vector>
// #include <cmath>
// #include <ros/ros.h>
// #include <sensor_msgs/LaserScan.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <image_transport/image_transport.h>
// #include <opencv2/highgui/highgui.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <pcl/point_types.h>
// #include <pcl/point_cloud.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/visualization/cloud_viewer.h>
// #include <pcl/features/normal_3d.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Matrix3x3.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <Eigen/Geometry>
// #include <geometry_msgs/PoseStamped.h>
// // #include <pcl/features/normal_3d.h>
// // #include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/surface/mls.h>
// #include <yaml-cpp/yaml.h>
// #include "math.hpp"
// tf2_ros::Buffer tfBuffer;
// ros::Publisher pcl_publisher;
// // 定义一个结构体slope_body，用于存储斜坡的四个角点，底边中心点，法向量
// struct slope_body
// {
//     std::vector<Eigen::Vector3f> points; // 四个角点
//     Eigen::Vector3f point_center;        // 底边中心点
//     Eigen::Vector3f normal_vector;       // 平面法向量
//     float h;                             // 斜坡高度
// };
// std::vector<slope_body> slopes;
// slope_body slope1;
// // 定义一个结构体PointInt，用于存储点的x,y,z坐标
// struct PointInt
// {
//     int p_x, p_y, p_z;
// };
// // 计算平面的法向量,确保指向上方
// void calculate_normal(slope_body &slope)
// {
//     Eigen::Vector3f vector1 = slope.points[0] - slope.points[1];
//     Eigen::Vector3f vector2 = slope.points[1] - slope.points[2];
//     Eigen::Vector3f normal = vector1.cross(vector2);
//     if (normal[2] < 0)
//         normal = -normal;
//     slope.normal_vector = normal / normal.norm();
// }
// // 读取yaml文件，进行初始化
// void init_slopes()
// {
//     YAML::Node config = YAML::LoadFile("slopes.yaml");
//     for (const auto &slope_node : config["slopes"])
//     {
//         slope_body slope;
//         for (int i = 0; i < 4; ++i)
//         {
//             Eigen::Vector3f point;
//             point[0] = slope_node["points"][i]["x"].as<float>();
//             point[1] = slope_node["points"][i]["y"].as<float>();
//             point[2] = slope_node["points"][i]["z"].as<float>();
//             slope.points.push_back(point);
//         }
//         slope.point_center[0] = slope_node["points"][4]["x"].as<float>();
//         slope.point_center[1] = slope_node["points"][4]["y"].as<float>();
//         slope.point_center[2] = slope_node["points"][4]["z"].as<float>();
//         calculate_normal(slope);
//         slopes.push_back(slope);
//     }
// }
// /**
//  * @brief  检查点是否在平面区域内
//  *
//  * @param testx 检查点x坐标
//  * @param testy 检查点  y
//  * @param slope
//  * @return int 0为不在区域内
//  */
// int pnpoly(float testx, float testy, const slope_body &slope)
// {
//     std::vector<float> pos_x = {slope.points[0][0], slope.points[1][0], slope.points[2][0], slope.points[3][0]};
//     auto max_X = std::max_element(pos_x.begin(), pos_x.end());
//     auto min_X = std::min_element(pos_x.begin(), pos_x.end());
//     float maxX = *max_X; // 解引用迭代器得到最大值
//     float minX = *min_X;
//     std::vector<float> pos_y = {slope.points[0][1], slope.points[1][1], slope.points[2][1], slope.points[3][1]};
//     auto max_Y = std::max_element(pos_x.begin(), pos_x.end());
//     auto min_Y = std::min_element(pos_x.begin(), pos_x.end());
//     float maxY = *max_Y; // 解引用迭代器得到最大值
//     float minY = *min_Y;
//     if (testx < minX || testx > maxX || testy < minY || testy > maxY)
//         return 0;
//     int i, j, c = 0;
//     for (i = 0, j = 4 - 1; i < 4; j = i++)
//     {
//         if (((slope.points[i][1] > testy) != (slope.points[j][1] > testy)) &&
//             (testx < (slope.points[j][0] - slope.points[i][0]) * (testy - slope.points[i][1]) / (slope.points[j][1] - slope.points[i][1]) + slope.points[i][0]))
//             c = !c;
//     }
//     return c;
// }
// /**
//  * @brief 根据测试点与中心点组成的向量与法向量夹角判断点是否在斜坡内部
//  *
//  * @param testx 测试点坐标x
//  * @param testy 测试点坐标y
//  * @param testz 测试点坐标z
//  * @param slope
//  * @return int 0为在斜坡内部，1为在斜坡外部
//  */
// int normal_judge(float testx, float testy, float testz, const slope_body &slope)
// {
//     Eigen::Vector3f line;
//     line << testx - slope.point_center[0], testy - slope.point_center[1], testz - slope.point_center[2];
//     float angle = acos(line.dot(slope.normal_vector) / (line.norm() * slope.normal_vector.norm()));
//     if (angle > 0)
//         return 1;
//     else
//         return 0;
// }
// /**
//  * @brief 更新斜坡的位置到body坐标系下
//  *
//  * @param slope 斜坡
//  * @param transform map2body
//  */
// void transform_slope(slope_body &slope, const geometry_msgs::TransformStamped &transform)
// {
//     geometry_msgs::PoseStamped in;
//     geometry_msgs::PoseStamped out;
//     for (int i = 0; i < slope.points.size(); i++) // 更新角点坐标
//     {
//         in.pose.position.x = slope.points[i][0];
//         in.pose.position.y = slope.points[i][1];
//         in.pose.position.z = slope.points[i][2];
//         out = tools::trans(in, transform);
//         slope.points[i][0] = out.pose.position.x;
//         slope.points[i][1] = out.pose.position.y;
//         slope.points[i][2] = out.pose.position.z;
//     }
//     // 更新中心点
//     in.pose.position.x = slope.point_center[0];
//     in.pose.position.y = slope.point_center[1];
//     in.pose.position.z = slope.point_center[2];
//     out = tools::trans(in, transform);
//     slope.point_center[0] = out.pose.position.x;
//     slope.point_center[1] = out.pose.position.y;
//     slope.point_center[2] = out.pose.position.z;
//     // 更新法向量
//     in.pose.position.x = slope.normal_vector[0];
//     in.pose.position.y = slope.normal_vector[1];
//     in.pose.position.z = slope.normal_vector[2];
//     out = tools::trans(in, transform);
//     slope.normal_vector[0] = out.pose.position.x;
//     slope.normal_vector[1] = out.pose.position.y;
//     slope.normal_vector[2] = out.pose.position.z;
// }
// float current_x = 0.0, current_y = 0.0;

// void getcloud_vec(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
// { // 过滤点云
//     pcl::PointCloud<pcl::PointXYZ>::Ptr pcl2cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     sensor_msgs::PointCloud2 ROSPCL_output;
//     pcl::fromROSMsg(*laserCloudMsg, *pcl2cloud);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr pcl2cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

//     geometry_msgs::TransformStamped map2body;
//     try
//     {
//         map2body = tfBuffer.lookupTransform("body", "map", ros::Time(0));
//     }
//     catch (tf2::TransformException &ex)
//     {
//         ROS_WARN("Pcl2process Get TF ERROR!");
//         return;
//     }
//     slope_body slope_1 = slope1;
//     transform_slope(slope_1, map2body);
//     if ((pcl2cloud->points.size()) == 0)
//     {
//         return;
//     }
//     else
//     {
//         long point_num = 0;
//         for (long i = 0; i <= pcl2cloud->points.size(); i = i + 1)
//         {
//             int num = 0;
//             for (num; num < slopes.size(); num++)
//             {
//                 if (pnpoly(pcl2cloud_out->points[i].x, pcl2cloud_out->points[i].y, slopes[num]))
//                     break;
//             }
//             if (num<slopes.size()&&(!normal_judge(pcl2cloud_out->points[i].x, pcl2cloud_out->points[i].y, pcl2cloud_out->points[i].z, slopes[num])))
//                 continue;
//             pcl2cloud_out->points.push_back(pcl2cloud->points[i]);
//             point_num = point_num + 1;
//         }

//         pcl::PointXYZ point4push;
//         for (float x = -7.00; x < 22.0; x = x + 0.05)
//         {
//             point4push.x = x;
//             point4push.y = -7.85f;
//             point4push.z = 0.2f;
//             pcl2cloud_out->points.push_back(point4push);
//             point4push.y = 7.85f;
//             pcl2cloud_out->points.push_back(point4push);
//             point_num = point_num + 2;
//         }
//         for (float y = -7.85; y < 7.85; y = y + 0.05)
//         {
//             point4push.x = -7.00f;
//             point4push.y = y;
//             point4push.z = 0.2f;
//             pcl2cloud_out->points.push_back(point4push);
//             point4push.x = 22.0f;
//             pcl2cloud_out->points.push_back(point4push);
//             point_num = point_num + 2;
//         }

//         pcl2cloud_out->width = point_num;
//         pcl2cloud_out->height = 1;
//         pcl2cloud_out->points.resize(pcl2cloud_out->width * pcl2cloud_out->height);
//         pcl::toROSMsg(*pcl2cloud_out, ROSPCL_output);
//         ROSPCL_output.header.frame_id = "body";
//         pcl_publisher.publish(ROSPCL_output);
//     }
// }

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "pointcloud_process");
//     ros::NodeHandle pnh("~");
//     init_slopes();
//     tf2_ros::TransformListener tfListener(tfBuffer);
//     auto subCloud = pnh.subscribe<sensor_msgs::PointCloud2>("/cloud_registered_body", 1, getcloud_vec);
//     pcl_publisher = pnh.advertise<sensor_msgs::PointCloud2>("/pointcloud2_out", 1);
//     ros::spin();
//     return 0;
// }
//
// Created by KevinTC.
//
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

//#include <pcl/features/normal_3d.h>
//#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

tf2_ros::Buffer tfBuffer;
ros::Publisher pcl_publisher;

inline float float_abs(float x) {
    if (x > 0) {
        return x;
    } else {
        return -x;
    }
}



// 定义一个结构体PointInt，用于存储点的x,y,z坐标
struct PointInt {
    int p_x, p_y, p_z;
};

float current_x = 0.0, current_y = 0.0;

void getcloud_vec(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {//法向量法投影转平面
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl2cloud(new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2 ROSPCL_output;
    pcl::fromROSMsg(*laserCloudMsg, *pcl2cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl2cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

    geometry_msgs::TransformStamped base2map;
    try {
        base2map = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));

    } catch (tf2::TransformException &ex) {
        ROS_WARN("Pcl2process Get TF ERROR!");
        return;
    }
    current_x = base2map.transform.translation.x;
    current_y = base2map.transform.translation.y;

    if((pcl2cloud->points.size()) == 0){
        return;
    }
    else{
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        pcl::VoxelGrid<pcl::PointXYZ> filter;
        filter.setInputCloud(pcl2cloud);
        filter.setLeafSize(0.01f, 0.01f, 0.01f);
        filter.filter(*pcl2cloud);
        ne.setInputCloud(pcl2cloud);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        ne.setSearchMethod(tree);
        //存储输出数据
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        //ne.setRadiusSearch(0.1); //使用半径在查询点周围3厘米范围内的所有临近元素
        ne.setKSearch(10); //使用最近的10个点
        ne.compute(*cloud_normals);
        long point_num = 0;
        for (long i = 0; i <= pcl2cloud->points.size(); i = i + 1) {
            float gradient = (pow(cloud_normals->points[i].normal_x, 2) + pow(cloud_normals->points[i].normal_y, 2)) / pow(cloud_normals->points[i].normal_z, 2);
            if(gradient > 0.7f){
                if(pcl2cloud->points[i].y > 6.6 or pcl2cloud->points[i].y < -6.6){
                    continue;
                }
                if(pow(pcl2cloud->points[i].x - current_x, 2) + pow(pcl2cloud->points[i].y - current_y, 2) > 0.09){
                    pcl2cloud->points[i].z = 0.25;
                    pcl2cloud_out->points.push_back(pcl2cloud->points[i]);
                    point_num = point_num + 1;
                }
            }
        }

        pcl::PointXYZ point4push;
        for (float x = -7.00; x < 22.0; x = x + 0.05) {
            point4push.x = x;
            point4push.y = -7.85f;
            point4push.z = 0.2f;
            pcl2cloud_out->points.push_back(point4push);
            point4push.y = 7.85f;
            pcl2cloud_out->points.push_back(point4push);
            point_num = point_num + 2;
        }
        for (float y = -7.85; y < 7.85; y = y + 0.05) {
            point4push.x = -7.00f;
            point4push.y = y;
            point4push.z = 0.2f;
            pcl2cloud_out->points.push_back(point4push);
            point4push.x = 22.0f;
            pcl2cloud_out->points.push_back(point4push);
            point_num = point_num + 2;
        }

        pcl2cloud_out->width = point_num;
        pcl2cloud_out->height = 1;
        pcl2cloud_out->points.resize(pcl2cloud_out->width * pcl2cloud_out->height);
        pcl::toROSMsg(*pcl2cloud_out, ROSPCL_output);
        ROSPCL_output.header.frame_id = "body";
        pcl_publisher.publish(ROSPCL_output);
    }
}

void getcloud(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {//使用体素+梯度法进行点云分割
    std::vector<std::vector<int>> point_list1(2800);
    std::vector<PointInt> point_list2;
    geometry_msgs::TransformStamped base2map;
    try {
        base2map = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));

    } catch (tf2::TransformException &ex) {
        ROS_WARN("Pcl2process Get TF ERROR!");
        return;
    }
    current_x = base2map.transform.translation.x;
    current_y = base2map.transform.translation.y;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl2cloud(new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2 ROSPCL_output;
    pcl::fromROSMsg(*laserCloudMsg, *pcl2cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl2cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
    unsigned int point_num = 0;
    cv::Mat hight_map(2801, 2801, CV_8UC1, cv::Scalar(0));
    cv::Mat gradient_map(2801, 2801, CV_8UC1, cv::Scalar(0));
    for (auto point: (pcl2cloud->points)) {
        point.z = point.z + 0.52;
        if (point.x < 22.4) {   //判断该点是否属于车身
            if (point.x > -5.4) {
                if (point.y < 13.8) {
                    if (point.y > -13.8) {
                        if (point.z > -1.2) {
                            if (point.z < 1.2) {
                                if (pow((current_x - point.x), 2) + pow((current_y - point.y), 2) < 0.067) {
                                    continue;
                                }
                                /*
                                float point_distance = sqrtf(point.x * point.x + point.y * point.y);
                                if(point_distance > 0.4f){
                                    if(float_abs(point.z) > 0.7f * (point_distance - 0.3f)){
                                        pcl::PointXYZ point4push;
                                        /*
                                        point4push.x = point.x;
                                        point4push.y = point.y;
                                        point4push.z = 0.0f;

                                        pcl2cloud_out->points.push_back(point);
                                        point_num = point_num + 1;
                                        continue;
                                    }
                                }
                                */
                                int px = 100 * point.x + 550;
                                int py = 100 * point.y + 1400;
                                unsigned char pz = 100 * point.z + 130;
                                if (!(hight_map.at<uchar>(px, py))) {
                                    hight_map.at<uchar>(px, py) = pz;
                                    point_list1[px].push_back(py);
                                    point_list2.push_back({px, py, pz});
                                } else {
                                    if (pz > hight_map.at<uchar>(px, py)) {
                                        hight_map.at<uchar>(px, py) = pz;
                                        point_list1[px].push_back(py);
                                        point_list2.push_back({px, py, pz});
                                    }
                                }
                            }
                            //continue;
                        }
                    }
                }
            }
        }
        //pcl2cloud_out->points.push_back(point);
    }

    for (auto point: point_list2) {
        unsigned char max = 0;
        unsigned char min = 255;
        if (point.p_x < 1435) {
            if (point.p_x > 1365) {
                if (point.p_y < 1435) {
                    if (point.p_y > 1365) {
                        continue;
                    }
                }
            }
        }
        int surround_point_x = 0;
        for (surround_point_x = ((point.p_x - 10) > 0 ? (point.p_x - 10) : 0);//在附近20*20cm范围内搜索高度差异最大的点
             surround_point_x < ((point.p_x + 10) < 2799 ? (point.p_x + 10) : 2799);
             surround_point_x = surround_point_x + 1) {
            for (auto surround_point_y: point_list1[surround_point_x]) {
                if ((surround_point_y < point.p_y + 10) and (surround_point_y > point.p_y - 10)) {
                    if (hight_map.at<uchar>(surround_point_x, surround_point_y) < min) {
                        min = hight_map.at<uchar>(surround_point_x, surround_point_y);
                    }
                    if (hight_map.at<uchar>(surround_point_x, surround_point_y) > max) {
                        max = hight_map.at<uchar>(surround_point_x, surround_point_y);
                    }
                }
                if (surround_point_y > point.p_y + 10) {
                    break;
                }
            }
        }
        unsigned char point_gradient = max - min;
        gradient_map.at<uchar>(point.p_x, point.p_y) = point_gradient;
        if (point_gradient > 14) {
            pcl::PointXYZ point4push;
            point4push.x = (float) (point.p_x - 550) / 100;
            point4push.y = (float) (point.p_y - 1400) / 100;
            point4push.z = 0.15f;
            pcl2cloud_out->points.push_back(point4push);
            point_num = point_num + 1;
        } else {
            unsigned char max_5cm = 0;
            unsigned char min_5cm = 255;
            int surround_point_x = 0;
            for (surround_point_x = ((point.p_x - 5) > 0 ? (point.p_x - 5) : 0);
                 surround_point_x < ((point.p_x + 5) < 2799 ? (point.p_x + 5) : 2799);
                 surround_point_x = surround_point_x + 1) {
                for (auto surround_point_y: point_list1[surround_point_x]) {
                    if ((surround_point_y < point.p_y + 5) and (surround_point_y > point.p_y - 5)) {
                        if (hight_map.at<uchar>(surround_point_x, surround_point_y) < min) {
                            min_5cm = hight_map.at<uchar>(surround_point_x, surround_point_y);
                        }
                        if (hight_map.at<uchar>(surround_point_x, surround_point_y) > max) {
                            max_5cm = hight_map.at<uchar>(surround_point_x, surround_point_y);
                        }
                    }
                    if (surround_point_y > point.p_y + 5) {
                        break;
                    }
                }
            }
            unsigned char point_gradient_5cm = max_5cm - min_5cm;
            //gradient_map.at<uchar>(point.p_x, point.p_y) = point_gradient;
            if (point_gradient > 10) {
                pcl::PointXYZ point4push;
                point4push.x = (float) (point.p_x - 550) / 100;
                point4push.y = (float) (point.p_y - 1400) / 100;
                point4push.z = 0.12f;
                pcl2cloud_out->points.push_back(point4push);
                point_num = point_num + 1;
            }
        }
    }
//    pcl::PointXYZ point4push;
//    for (float x = -5.88; x < 21.8; x = x + 0.05) {
//        point4push.x = x;
//        point4push.y = -7.51f;
//        point4push.z = 0.2f;
//        pcl2cloud_out->points.push_back(point4push);
//        point4push.y = 7.24f;
//        pcl2cloud_out->points.push_back(point4push);
//        point_num = point_num + 2;
//    }
//    for (float y = -7.51; y < 7.24; y = y + 0.05) {
//        point4push.x = -5.88f;
//        point4push.y = y;
//        point4push.z = 0.2f;
//        pcl2cloud_out->points.push_back(point4push);
//        point4push.x = 21.8f;
//        pcl2cloud_out->points.push_back(point4push);
//        point_num = point_num + 2;
//    }
    pcl2cloud_out->width = point_num;
    pcl2cloud_out->height = 1;
    pcl2cloud_out->points.resize(pcl2cloud_out->width * pcl2cloud_out->height);
    cv::threshold(gradient_map, gradient_map, 10, 255, cv::THRESH_BINARY);
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(pcl2cloud_out);
    filter.setLeafSize(0.03f, 0.03f, 0.01f);//体素下采样
    filter.filter(*pcl2cloud_out);

    pcl::toROSMsg(*pcl2cloud_out, ROSPCL_output);
//    cv::imshow("gradient_map", hight_map);
//    cv::waitKey(1);
    ROSPCL_output.header.frame_id = "map";
    pcl_publisher.publish(ROSPCL_output);
    //ROS_INFO("%d",pcl2cloud->points.size());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pointcloud_process");
    ros::NodeHandle pnh("~");
    tf2_ros::TransformListener tfListener(tfBuffer);
    auto subCloud = pnh.subscribe<sensor_msgs::PointCloud2>("/cloud_registered_body", 1, getcloud_vec);
    pcl_publisher = pnh.advertise<sensor_msgs::PointCloud2>("/pointcloud2_out", 1);
    ros::spin();
    return 0;
    //auto sub1 = pnh.subscribe("/pointcloud2_in", 100, laserCallback);
    //auto sub2 = pnh.subscribe("/depth_image", 100, depth_img_callback);
    //pub = pnh.advertise<sensor_msgs::LaserScan>("/projected_scan", 1);
    //cv::destroyWindow("depth");
}
