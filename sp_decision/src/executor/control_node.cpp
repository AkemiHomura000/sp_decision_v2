#include "executor/control_node.hpp"
namespace sp_decision
{
    ControlNode::ControlNode(const Blackboard::Ptr &blackboard_ptr,
                             const ChassisExecutor::Ptr &chassis_ptr,
                             const GimbalExecutor::Ptr &gimbal_ptr,
                             const tools::logger::Ptr &logger_ptr)
    {
        blackboard_ptr_=blackboard_ptr;
        chassis_ptr_ = chassis_ptr;
        gimbal_ptr_ = gimbal_ptr;
        logger_ptr_ = logger_ptr;
        yaml_reader_ptr_ = std::make_shared<tools::yaml_reader>(ros::package::getPath("sp_decision") + "/config/points.yaml");
        loop_rate = 20.0;
        last_decision_ = "null";
        points_init();
        decision_sub_ = nh_.subscribe("/sentry/decision", 10, &ControlNode::decision_sub, this);
    }
    ControlNode::~ControlNode()
    {
        if (control_thread_.joinable())
        {
            control_thread_running = false;
            control_thread_.join();
        }
    }
    void ControlNode::points_init()
    {
        if (yaml_reader_ptr_->readYAML())
        {
            YAML::Node config = yaml_reader_ptr_->getConfig();
            for (const auto &node : config["points"])
            {
                Eigen::Vector2d point;
                point[0] = node["x"].as<double>();
                point[1] = node["y"].as<double>();
                points_.push_back(point);
            }
            for (const auto &point : points_)
            {
                std::cout << "x: " << point[0] << "  y:  " << point[1] << std::endl;
            }
        }
    }
    void ControlNode::decision_sub(const robot_msg::DecisionMsg &msg)
    {
        decision_cbk_mutex.lock();
        decision_ = msg.decision;
        decision_cbk_mutex.unlock();
    }
    void ControlNode::execute_decision()
    {
        if (decision_ == "null")
            return;
        else if (decision_ == "charge")
            charge();
    }
    void ControlNode::run()
    {
        ros::Rate rate(loop_rate);
        while (ros::ok() && control_thread_running)
        {
            if (last_decision_ != decision_)
            {
                chassis_ptr_->action_status_ = 0;
                last_decision_ = decision_;
            }
            execute_decision();
            rate.sleep();
        }
    }
    void ControlNode::run_start()
    {
        control_thread_running = true;
        control_thread_ = std::thread(&ControlNode::run, this);
    }
    void ControlNode::charge()
    {
        std::vector<Eigen::Vector2d> points_1;
        std::vector<float> wait_time_1;
        for (int i = 0; i < 12; i++)
        {
            points_1.push_back(points_[i]);
            if (i < 5)
                wait_time_1.push_back(0.2);
            else
                wait_time_1.push_back(5.0);
        }
        chassis_ptr_->cequence_move(points_1, wait_time_1, false);
    }
}