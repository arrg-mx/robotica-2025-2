#include "rclcpp/rclcpp.hpp"

#include "srv_act_pkg/srv/check_number.hpp"


class CheckNumberServiceNode : public rclcpp::Node
{
    public:
        CheckNumberServiceNode() : Node("check_number_service_node")
        {
            service_server_ = this->create_service<srv_act_pkg::srv::CheckNumber>(
                "check_number", std::bind(&CheckNumberServiceNode::check_number_even, this,
             std::placeholders::_1, std::placeholders::_2)
            );
        }

    private:
        void check_number_even(
            const srv_act_pkg::srv::CheckNumber::Request::SharedPtr request,
            srv_act_pkg::srv::CheckNumber::Response::SharedPtr response)
        {
            int remainder = request->number %2;

            switch (remainder)
            {
            case 0:
                response->result = "Par";        
                break;
            
            case 1:
                response->result = "Impar";
                break;
            }

        }
        rclcpp::Service<srv_act_pkg::srv::CheckNumber>::SharedPtr service_server_;
};



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CheckNumberServiceNode>());
    rclcpp::shutdown();

    return 0;
}

