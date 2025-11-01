#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"   
#include "std_msgs/msg/string.hpp"

class DeciderNode : public rclcpp::Node
{
    public:
    DeciderNode()
    : Node("decider_node")
    {
        decision_publisher_ = this->create_publisher<std_msgs::msg::String>("degree_check", 10);

        degree_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "degree_topic", 10, std::bind(&DeciderNode::degree_callback, this, std::placeholders::_1));
        
       
    }

    private:
    void degree_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        int recived_num =msg->data;
        auto decision_msg = std_msgs::msg::String();
        if (msg->data > 33) {
            decision_msg.data = "Too high:" + std::to_string(recived_num);
        } 
        else if (msg->data < 27) {
            decision_msg.data = "Too low:" + std::to_string(recived_num);
        }
        else {
            decision_msg.data = "right:" + std::to_string(recived_num);
        }
        RCLCPP_INFO(this->get_logger(), " Decision: '%s'", decision_msg.data.c_str());
        decision_publisher_->publish(decision_msg);

    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr degree_subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr decision_publisher_;


};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DeciderNode>());
    rclcpp::shutdown();
    return 0;
}

