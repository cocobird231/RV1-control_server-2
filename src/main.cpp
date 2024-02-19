#include "header.h"
#include "vehicle_interfaces/params.h"
#include "vehicle_interfaces/msg_json.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto params = std::make_shared<Params>("controlserver_params_node");
    auto server = std::make_shared<ControlServer>(params);
    rclcpp::executors::SingleThreadedExecutor* executor = new rclcpp::executors::SingleThreadedExecutor();
    executor->add_node(server);
    std::thread execTh(vehicle_interfaces::SpinExecutor, executor, "server", 1000.0);

    executor->cancel();
    execTh.join();
    server->close();
    delete executor;
    rclcpp::shutdown();
    return 0;
}