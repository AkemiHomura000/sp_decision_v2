#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

cv::Mat vel_img;

void get_cmd_vel(const geometry_msgs::Twist &msg)
{
    ROS_INFO("linear_x: %f", msg.linear.x);
    ROS_INFO("linear_y: %f", msg.linear.y);
    ROS_INFO("angular_z: %f", msg.angular.z);

    // 计算速度矢量的大小
    double magnitude = sqrt(pow(msg.linear.x, 2) + pow(msg.linear.y, 2));

    // 计算速度矢量的方向（以弧度表示）
    double direction = atan2(msg.linear.y, msg.linear.x);

    // 将方向从弧度转换为度
    direction *= 180.0 / M_PI;

    // 在 vel_img 上绘制速度矢量
    int center_x = vel_img.cols / 2;
    int center_y = vel_img.rows / 2;
    int arrow_length = 50;

    int end_x = center_x + arrow_length * cos(direction);
    int end_y = center_y + arrow_length * sin(direction);

    cv::arrowedLine(vel_img, cv::Point(center_x, center_y), cv::Point(end_x, end_y), cv::Scalar(255), 2);
    cv::imshow("Velocity Image", vel_img);
    cv::waitKey(1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_vel_test");
    ros::NodeHandle nh("~");
    vel_img = cv::Mat(160, 160, CV_8UC1, cv::Scalar(0));

    ros::Subscriber enemy_sub_ = nh.subscribe("/cmd_vel", 1, get_cmd_vel);

    ros::spin();
}
