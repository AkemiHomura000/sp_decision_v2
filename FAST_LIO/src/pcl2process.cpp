/**
 * @file pcl2process.cpp
 * @author lp
 * @brief 转发/cloud_registered_body为/pointcloud2_out。实现点云与先验地图的匹配，根据先验地图过滤掉坡面(先判定xy范围，再根据角度确定是否属于坡面)
 * @version 0.1
 * @date 2024-04-17
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <algorithm>
#include <vector>
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
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

#include "math.hpp"
tf2_ros::Buffer tfBuffer;
ros::Publisher pcl_publisher;
struct slope_body
{
    std::vector<Eigen::Vector2f> points; // 四个角点
    Eigen::Vector3f point_center;        // 底边中心点
    Eigen::Vector3f normal_vector;       // 平面法向量
};
slope_body slope1;
// 定义一个结构体PointInt，用于存储点的x,y,z坐标
struct PointInt
{
    int p_x, p_y, p_z;
};
// 定义一个结构体slope_body，用于存储斜坡的四个角点，底边中心点，法向量

/**
 * @brief  检查点是否在平面区域内
 *
 * @param testx 检查点x坐标
 * @param testy 检查点  y
 * @param slope
 * @return int 0为不在区域内
 */
int pnpoly(float testx, float testy, const slope_body &slope)
{
    std::vector<float> pos_x = {slope.points[0][0], slope.points[1][0], slope.points[2][0], slope.points[3][0]};
    auto max_X = std::max_element(pos_x.begin(), pos_x.end());
    auto min_X = std::min_element(pos_x.begin(), pos_x.end());
    float maxX = *max_X; // 解引用迭代器得到最大值
    float minX = *min_X;
    std::vector<float> pos_y = {slope.points[0][1], slope.points[1][1], slope.points[2][1], slope.points[3][1]};
    auto max_Y = std::max_element(pos_x.begin(), pos_x.end());
    auto min_Y = std::min_element(pos_x.begin(), pos_x.end());
    float maxY = *max_Y; // 解引用迭代器得到最大值
    float minY = *min_Y;
    if (testx < minX || testx > maxX || testy < minY || testy > maxY)
        return 0;
    int i, j, c = 0;
    for (i = 0, j = 4 - 1; i < 4; j = i++)
    {
        if (((slope.points[i][1] > testy) != (slope.points[j][1] > testy)) &&
            (testx < (slope.points[j][0] - slope.points[i][0]) * (testy - slope.points[i][1]) / (slope.points[j][1] - slope.points[i][1]) + slope.points[i][0]))
            c = !c;
    }
    return c;
}
/**
 * @brief 根据测试点与中心点组成的向量与法向量夹角判断点是否在斜坡内部
 *
 * @param testx 测试点坐标x
 * @param testy 测试点坐标y
 * @param testz 测试点坐标z
 * @param slope
 * @return int 0为在斜坡内部，1为在斜坡外部
 */
int normal_judge(float testx, float testy, float testz, const slope_body &slope)
{
    Eigen::Vector3f line;
    line << testx - slope.point_center[0], testy - slope.point_center[1], testz - slope.point_center[2];
    float angle = acos(line.dot(slope.normal_vector) / (line.norm() * slope.normal_vector.norm()));
    if (angle > 0)
        return 1;
    else
        return 0;
}
/**
 * @brief 更新斜坡的位置到body坐标系下
 *
 * @param slope 斜坡
 * @param transform map2body
 */
void transform_slope(slope_body &slope, const geometry_msgs::TransformStamped &transform)
{
    geometry_msgs::PoseStamped in;
    geometry_msgs::PoseStamped out;
    for (int i = 0; i < slope.points.size(); i++) // 更新角点坐标
    {
        in.pose.position.x = slope.points[i][0];
        in.pose.position.y = slope.points[i][1];
        in.pose.position.z = 0;
        out = tools::trans(in, transform);
        slope.points[i][0] = out.pose.position.x;
        slope.points[i][1] = out.pose.position.y;
    }
    // 更新中心点
    in.pose.position.x = slope.point_center[0];
    in.pose.position.y = slope.point_center[1];
    in.pose.position.z = slope.point_center[2];
    out = tools::trans(in, transform);
    slope.point_center[0] = out.pose.position.x;
    slope.point_center[1] = out.pose.position.y;
    slope.point_center[2] = out.pose.position.z;
    // 更新法向量
    in.pose.position.x = slope.normal_vector[0];
    in.pose.position.y = slope.normal_vector[1];
    in.pose.position.z = slope.normal_vector[2];
    out = tools::trans(in, transform);
    slope.normal_vector[0] = out.pose.position.x;
    slope.normal_vector[1] = out.pose.position.y;
    slope.normal_vector[2] = out.pose.position.z;
}
float current_x = 0.0, current_y = 0.0;

void getcloud_vec(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{ // 过滤点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl2cloud(new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2 ROSPCL_output;
    pcl::fromROSMsg(*laserCloudMsg, *pcl2cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl2cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

    geometry_msgs::TransformStamped map2body;
    try
    {
        map2body = tfBuffer.lookupTransform("body", "map", ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Pcl2process Get TF ERROR!");
        return;
    }
    slope_body slope_1 = slope1;
    transform_slope(slope_1, map2body);
    if ((pcl2cloud->points.size()) == 0)
    {
        return;
    }
    else
    {
        long point_num = 0;
        for (long i = 0; i <= pcl2cloud->points.size(); i = i + 1)
        {
            if (!pnpoly(pcl2cloud_out->points[i].x, pcl2cloud_out->points[i].y, slope_1))
                return;
            if (!normal_judge(pcl2cloud_out->points[i].x, pcl2cloud_out->points[i].y, pcl2cloud_out->points[i].z, slope_1))
                break;
            pcl2cloud_out->points.push_back(pcl2cloud->points[i]);
            point_num = point_num + 1;
        }

        pcl::PointXYZ point4push;
        for (float x = -7.00; x < 22.0; x = x + 0.05)
        {
            point4push.x = x;
            point4push.y = -7.85f;
            point4push.z = 0.2f;
            pcl2cloud_out->points.push_back(point4push);
            point4push.y = 7.85f;
            pcl2cloud_out->points.push_back(point4push);
            point_num = point_num + 2;
        }
        for (float y = -7.85; y < 7.85; y = y + 0.05)
        {
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_process");
    ros::NodeHandle pnh("~");
    slope1.points = {{0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}};
    slope1.point_center = {0.0f, 0.0f, 0.0f};
    slope1.normal_vector = {0.0f, 0.0f, 0.0f};
    tf2_ros::TransformListener tfListener(tfBuffer);
    auto subCloud = pnh.subscribe<sensor_msgs::PointCloud2>("/cloud_registered_body", 1, getcloud_vec);
    pcl_publisher = pnh.advertise<sensor_msgs::PointCloud2>("/pointcloud2_out", 1);
    ros::spin();
    return 0;
}
