#include "decision_node/decision_tree.hpp"
namespace sp_decision
{
    
    // 判定函数1：决策变量>条件量
    bool compare_1(double a, double b)
    {
        return (a > b) ? 1 : 0;
    }
    // 判定函数2：决策变量<条件量
    bool compare_2(double a, double b)
    {
        return (a < b) ? 1 : 0;
    }
    // 判定函数3：决策变量==条件量
    bool compare_3(double a, double b)
    {
        return (a == b) ? 1 : 0;
    }
    // 判定函数4：决策变量!=条件量
    bool compare_4(double a, double b)
    {
        return (a != b) ? 1 : 0;
    }
    void decision_tree::sighandler(int signum)
    {
        printf("用户退出...\n");
        exit(1);
    }
    decision_tree::decision_tree(const tools::yaml_reader::Ptr &yaml_ptr)
    {
        decision_pub =
            nh_.advertise<robot_msg::EnemyStage>("/sentry/decision", 1);
        last_time = std::chrono::system_clock::now();
        a = 2;
        b = 3;
        c = 0;
        num = 0;
        yaml_reader_ptr = yaml_ptr;
        node_ptr_init(); // 生成决策图
    }
    void decision_tree::print_tree()
    {
        for (auto &node : nodes_vector)
        {
            std::cout << "ID: " << node.id << std::endl;
            std::cout << "Left ID: " << node.left->id << std::endl;
            std::cout << "Right ID: " << node.right->id << std::endl;
            std::cout << "condition_value: " << node.condition_value << std::endl;
            std::cout << "variable_value: " << *node.variable_ptr << std::endl;
            std::cout << "action: " << node.decision_msg.decision << std::endl;
            std::cout << std::endl;
        }
    }
    void decision_tree::node_ptr_init()
    {
        if (yaml_reader_ptr->readYAML())
        {
            YAML::Node config = yaml_reader_ptr->getConfig();
            for (const auto &node : config["decision_tree_1"])
            {
                tree_node dt_node;
                std::string symbol; // 获取符号
                double value;
                dt_node.id = node["id"].as<int>();
                dt_node.left_id = node["left_id"].as<int>();
                dt_node.right_id = node["right_id"].as<int>();
                std::string variable = node["variable"].as<std::string>();
                std::string condition = node["condition"].as<std::string>();
                std::string action = node["action"].as<std::string>();
                if (action != "")
                {
                    dt_node.decision_pub=&decision_pub;
                    dt_node.decision_msg.decision = action;
                }
                size_t pos = 0;
                while (pos < condition.size() && condition[pos] == ' ')
                    ++pos;
                // 获取符号
                while (pos < condition.size() && !isdigit(condition[pos]) && condition[pos] != ' ')
                    symbol += condition[pos++];
                if (symbol == ">")
                {
                    dt_node.compare_function = compare_1;
                }
                if (symbol == "<")
                {
                    dt_node.compare_function = compare_2;
                }
                if (symbol == "==")
                {
                    dt_node.compare_function = compare_3;
                }
                if (symbol == "!=")
                {
                    dt_node.compare_function = compare_4;
                }
                // 获取数字部分
                value = std::stod(condition.substr(pos));
                dt_node.condition_value = value;
                // 获取变量名
                if (variable == "a")
                {
                    dt_node.variable_ptr = &a;
                }
                if (variable == "b")
                {
                    dt_node.variable_ptr = &b;
                }
                if (variable == "c")
                {
                    dt_node.variable_ptr = &c;
                }
                nodes_vector.push_back(dt_node);
            }
            // 打印解析后的数据
            for (auto &node : nodes_vector)
            {
                if (node.left_id != -1)
                    node.left = &nodes_vector[node.left_id - 1];
                else
                    node.left = &nodes_vector[0];
                if (node.right_id != -1)
                    node.right = &nodes_vector[node.right_id - 1];
                else
                    node.right = &nodes_vector[0];
            }
        }
    }
    tree_node *decision_tree::judge(tree_node *node)
    {

        if (node->id == 1)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            num++;
            std::cout << "num:" << num << std::endl;
            auto now = std::chrono::system_clock::now();
            std::chrono::milliseconds duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time);
            std::cout << "delat-time" << duration.count() << std::endl;
            last_time = std::chrono::system_clock::now();
        }
        if (node->compare(*node->variable_ptr, node->condition_value))
        {
            if (node->left->id != 1)
            {
                std::cout << "judge_true\t" << node->id << std::endl;
                if (node->decision_msg.decision != "")
                    node->pub_decision();
                return judge(node->left);
            }
            else
            {
                node->compare(*node->variable_ptr, node->condition_value);
                if (node->decision_msg.decision != "")
                    node->pub_decision();
                std::cout << "judge_true\t" << node->id << std::endl;
            }
        }
        else
        {
            if (node->right->id != 1)
            {
                std::cout << "judge_false\t" << node->id << std::endl;
                if (node->decision_msg.decision != "")
                    node->pub_decision();
                return judge(node->right);
            }
            else
            {
                node->compare(*node->variable_ptr, node->condition_value);
                if (node->decision_msg.decision != "")
                    node->pub_decision();
                std::cout << "judge_true\t" << node->id << std::endl;
            }
        }
    }
    void decision_tree::run()
    {
        signal(SIGINT, sighandler);
        while (ros::ok)
        {
            judge(&nodes_vector[0]);
        }
    }
}
