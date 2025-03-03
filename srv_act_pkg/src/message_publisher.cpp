#include "rclcpp/rclcpp.hpp"

#include "srv_act_pkg/msg/numbers.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class HelloWorldPubNode : public rclcpp::Node
{
  public: 
    HelloWorldPubNode() : Node("hello_world_pub_node") 
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("hello_world", 10);
      timer_ = this->create_wall_timer(1s, std::bind(&HelloWorldPubNode::publish_hello_world, this));
    }

  private:
    void publish_hello_world()
    {
      auto message = std_msgs::msg::String();
      auto numeros = srv_act_pkg::msg::Numbers();
      numeros.number_1 = 10;
      numeros.number_2 = 3;

      message.data = "Hello World  " + std::to_string(counter_) + "  " + std::to_string(numeros.number_1) + "  " + std::to_string(numeros.number_2);
      publisher_->publish(message);
      counter_++;
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t counter_ = 0;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HelloWorldPubNode>());
  rclcpp::shutdown();
  
  return 0;
}
