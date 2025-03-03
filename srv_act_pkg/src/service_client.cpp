#include "rclcpp/rclcpp.hpp"
#include "srv_act_pkg/srv/check_number.hpp"


typedef srv_act_pkg::srv::CheckNumber CheckNumberSrv;


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto service_client_node = rclcpp::Node::make_shared("check_number_client_node");

    auto client = service_client_node->create_client<CheckNumberSrv>("check_number");

    auto request = std::make_shared<CheckNumberSrv::Request>();

    std::cout <<"Teclea un numero para determinar su es par o impar";
    std::cin >> request->number;

    client->wait_for_service();

    auto response = client->async_send_request(request);

    if(rclcpp::spin_until_future_complete(service_client_node, response) == rclcpp::FutureReturnCode::SUCCESS)
    {
        std::cout << "El numero es " << response.get()->result << std::endl;
    } else {
        std::cout << "Hay un error en el procesamiento de la respuesta";
    }

    rclcpp::shutdown();

    return 0;
}