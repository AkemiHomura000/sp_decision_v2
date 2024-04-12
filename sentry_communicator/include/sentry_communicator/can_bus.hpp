//#pragma once
#include <string>
// ros
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <realtime_tools/realtime_buffer.h>
#include <nav_msgs/Odometry.h>
// superpower_hardware
#include <sentry_communicator/socketcan.h>
#include <tf/transform_broadcaster.h>

#include <robot_msg/RefereeInfoMsg.h>
#include <robot_msg/RobotHP.h>
#include <robot_msg/CmdGimbal.h>

namespace sentry_communicator
{
    struct ChassisCommand
    {
        geometry_msgs::Twist cmd_vel_;
        ros::Time stamp_;
    };

    struct GimbalCommand
    {
        robot_msg::CmdGimbal cmd_gimbal_;
        ros::Time stamp_;
    };

    struct CanFrameStamp
    {
        can_frame frame;
        ros::Time stamp;
    };

    class CanBus
    {
    public:
        CanBus(const std::string &bus_name, int thread_priority, ros::NodeHandle &root_nh);
        void read(ros::Time time);
        void write();

    private:
        /*
         * @brief This Function will be called when CAN_DATA is captured by the thread.
         */
        void frameCallback(const can_frame &frame);
        void cmdChassisCallback(const geometry_msgs::Twist::ConstPtr &msg);
        void cmdGimbalCallback(const robot_msg::CmdGimbal::ConstPtr &msg);

        const std::string bus_name_;
        can::SocketCAN socket_can_;
        ros::Subscriber cmd_chassis_sub_;
        ros::Subscriber cmd_gimbal_sub_;
        std::mutex mutex_;


        ros::Publisher referee_info_pub_;
        robot_msg::RefereeInfoMsg 
            referee_info_msg_;

        ros::Publisher robot_EnemyHP_pub_;
        robot_msg::RobotHP robot_EnemyHP_msg_;
        // 上半数据更新标记
        bool upper_Enemydata_updated_ = false;
        // 下半数据更新标记
        bool lower_Enemydata_updated_ = false;

        ros::Publisher robot_TeamHP_pub_;
        robot_msg::RobotHP robot_TeamHP_msg_;
        // 上半数据更新标记
        bool upper_Teamdata_updated_ = false;
        // 下半数据更新标记
        bool lower_Teamdata_updated_ = false;
        

        // Lithesh : use realtime buffer to keep the multi-thread safe.
        realtime_tools::RealtimeBuffer<ChassisCommand> chassis_buffer_;
        realtime_tools::RealtimeBuffer<GimbalCommand> gimbal_buffer_;
        
        // the int array used to contain data_frame, which has 8 byte
        uint8_t *can_data_;
        can_frame chassis_frame_;
        can_frame gimbal_frame_;
        
        uint16_t data;
        
    };

    // TODO(Lithesh) : the zero-drift should be concerned.
    /* @brief : convert flaot into uint16_t, used in pair with uint2float()
     * @param[in]  bits - number of bits used to transmit the data
     */
    static uint16_t float2uint(float x, float x_min, float x_max, uint8_t bits)
    {
        float span = x_max - x_min;
        float offset = x_min;
        return (uint16_t)((x - offset) * (float)((1 << bits) - 1) / span);
    }

    static float uint2float(uint16_t x_int, float x_min, float x_max, uint8_t bits)
    {
        float span = x_max - x_min;
        float offset = x_min;
        return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
    }
} //  namespace : sentry_communicator