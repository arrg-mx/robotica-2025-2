#include "rclcpp/rclcpp.hpp"

#include "srv_act_pkg/srv/check_number.hpp"

typedef srv_act_pkg::srv::CheckNumber CheckNumberSrv;

class OddEvenCheckServiceNode : public rclcpp::Node
{
  public: 
    OddEvenCheckServiceNode() : Node("odd_even_check_service_node") 
    {
      service_server_ = this->create_service<CheckNumberSrv>(
        "odd_even_check", std::bind(&OddEvenCheckServiceNode::check_num_odd_even, this,
        std::placeholders::_1, std::placeholders::_2));
    }
  
  private:
    void check_num_odd_even(const CheckNumberSrv::Request::SharedPtr request,
                            CheckNumberSrv::Response::SharedPtr response)
    {
      
      int remainder = std::abs(request->number) % 2;

      switch(remainder) {
        case 0 :
          response->result = "Even";
          break;
        case 1 :
          response->result = "Odd";
      }
    }
    rclcpp::Service<CheckNumberSrv>::SharedPtr service_server_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OddEvenCheckServiceNode>());
  rclcpp::shutdown();
  
  return 0;
}