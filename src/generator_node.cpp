#include <chrono>
#include <memory>
#include <string>
#include <random> 

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"   
#include "std_msgs/msg/string.hpp" 

using namespace std::chrono_literals;

class GeneratorNode : public rclcpp::Node
{

public:
  GeneratorNode()
  : Node("generator_node"), count_(0)
  {
    degree_publisher_ = this->create_publisher<std_msgs::msg::Int32>("degree_topic", 10);
    feedback_subscription_ = this->create_subscription<std_msgs::msg::String>(
      "degree_check", 10, std::bind(&GeneratorNode::feedback_callback, this, std::placeholders::_1));
    
    timer_ = this->create_wall_timer(
      2s, std::bind(&GeneratorNode::publish_degree, this));
    
    random_device_ = std::make_unique<std::random_device>();
    gen_ = std::make_unique<std::mt19937>((*random_device_)());

   
  }


private:
    void publish_degree()
    {
        auto message = std_msgs::msg::Int32();

        std::uniform_int_distribution<> distrib(min_degree, max_degree);
        message.data = distrib(*gen_);
        RCLCPP_INFO(this->get_logger(), "Publishing degree: '%d'", message.data);
        degree_publisher_->publish(message);
    }

    void feedback_callback(const std_msgs::msg::String::SharedPtr msg)
    {   
        std::string feedback = msg->data;

        RCLCPP_INFO(this->get_logger(), "Received feedback: '%s'", msg->data.c_str());

        if (feedback.find("Too high") != std::string::npos) {
            max_degree = std::stoi(feedback.substr(feedback.find(':')+1))-1 ;
        } 
        else if (feedback.find("Too low") != std::string::npos) {
            min_degree = std::stoi(feedback.substr(feedback.find(':')+1))+1;
        } 
        
        RCLCPP_INFO(this->get_logger(), "Hatarok: [%d , %d]", min_degree, max_degree);
    }


    int max_degree = 40;
    int min_degree = 20;
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr degree_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr feedback_subscription_;
    size_t count_;

    
    std::unique_ptr<std::random_device> random_device_;
    std::unique_ptr<std::mt19937> gen_;
    std::unique_ptr<std::uniform_int_distribution<>> distrib_;
 
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GeneratorNode>());
  rclcpp::shutdown();
  return 0;
}