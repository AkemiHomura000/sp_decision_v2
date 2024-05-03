#include "executor/control_node.hpp"
namespace sp_decision
{
    ControlNode::ControlNode(const Blackboard::Ptr &blackboard_ptr, const tools::logger::Ptr &logger_ptr)
    {
        blackboard_ptr_ = blackboard_ptr;
        logger_ptr_ = logger_ptr;
        chassis_ptr_ = std::make_shared<sp_decision::ChassisExecutor>(logger_ptr_, blackboard_ptr);
        gimbal_ptr_ = std::make_shared<sp_decision::GimbalExecutor>(logger_ptr_);
        yaml_reader_ptr_ = std::make_shared<tools::yaml_reader>(ros::package::getPath("sp_decision") + "/config/points.yaml");
        loop_rate = 50.0;
        last_decision_ = "null";
        points_init();
        decision_sub_ = nh_.subscribe("/sentry/decision", 10, &ControlNode::decision_sub, this);
        enemy_pos_pub_ = nh_.advertise<geometry_msgs::Pose>("/need_filterate/enemy_pos", 10);
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
        param_list_.clear();
        std::string input = msg.decision;
        std::istringstream iss(input);
        std::string token;
        int num = 0;
        while (std::getline(iss, token, '-'))
        {
            if (num == 0)
                decision_ = token;
            else
                param_list_.push_back(std::stod(token));
            num++;
        }
        decision_type_ = msg.type;

        if (decision_type_ == 1 && last_decision_ != decision_) // 普通动作直接执行，不经过状态机
        {
            chassis_ptr_->action_status_ = 0;
            last_decision_ = decision_;
        }
        execute_decision();
        decision_cbk_mutex.unlock();
    }
    void ControlNode::execute_decision()
    {
        std::stringstream log_msg;
        log_msg << "decision: " << decision_;
        for (int i = 0; i < param_list_.size(); i++)
            log_msg << "-" << param_list_[i];
        logger_ptr_->logInfo(log_msg);

        ROS_INFO("decision: %s", decision_.c_str());
        if (decision_ == "null")
            return;
        else if (decision_ == "observe")
            gimbal_ptr_->gimbal_set(-param_list_[1], param_list_[0], true);
        else if (decision_ == "rotate")
        {
            if (param_list_[0] == 0)
                chassis_ptr_->rotate_state_ = ChassisExecutor::RotateState::IDLE;
            if (param_list_[0] == 1)
                chassis_ptr_->rotate_state_ = ChassisExecutor::RotateState::UPSLOPW;
            if (param_list_[0] == 2)
                chassis_ptr_->rotate_state_ = ChassisExecutor::RotateState::ROTATE;
        }
        else if (decision_ == "revive") // todo 增加复活指令
        {
            if (param_list_[0] == 0)
                ;
            if (param_list_[0] == 1)
                ;
            if (param_list_[0] == 2)
                ;
        }
        else if (decision_ == "remote_addblood") // todo 增加回血指令
        {
        }
    }
    // void ControlNode::run()
    // {
    //     ros::Rate rate(loop_rate);
    //     while (ros::ok() && control_thread_running)
    //     {
    //         if (decision_type_ == 1 && last_decision_ != decision_) // 普通动作直接执行，不经过状态机
    //         {
    //             chassis_ptr_->action_status_ = 0;
    //             last_decision_ = decision_;
    //         }
    //         execute_decision();
    //         rate.sleep();
    //     }
    // }
    // void ControlNode::run_start()
    // {
    //     control_thread_running = true;
    //     control_thread_ = std::thread(&ControlNode::run, this);
    // }
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
    void ControlNode::enemy_filiter(int enemy_id)
    {
        if ((ros::Time::now() - blackboard_ptr_->enemy_status[enemy_id].pos_update_time).toSec() < 0.5)
        {
            geometry_msgs::Pose enemy_pos;
            enemy_pos.position.x = blackboard_ptr_->enemy_status[enemy_id].robot_pos[0];
            enemy_pos.position.y = blackboard_ptr_->enemy_status[enemy_id].robot_pos[1];
            enemy_pos.position.z = 0.0;
            enemy_pos.orientation.x = 0.0;
            enemy_pos.orientation.y = 0.0;
            enemy_pos_pub_.publish(enemy_pos);
        }
    }
    void ControlNode::pursuit(int enemy_id)
    {
    }
    void ControlNode::add_blood()
    {
        chassis_ptr_->single_point_move(points_[0], points_[1]);
    }
    void ControlNode::reach_start_region()
    {
        std::vector<Eigen::Vector2d> points_1;
        for (int i = 2; i < 6; i++)
            points_1.push_back(points_[i]);
        decision_ = chassis_ptr_->range_move(points_1);
    }
}