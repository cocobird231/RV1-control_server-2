#include "header.h"
#include "vehicle_interfaces/params.h"
#include "vehicle_interfaces/msg_json.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto params = std::make_shared<Params>("controlserver_params_node");
    auto server = std::make_shared<ControlServer>(params);
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(server);
    std::thread execTh(vehicle_interfaces::SpinExecutor, executor, "server", 1000.0);

    while (1)
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    executor->cancel();
    execTh.join();
    server->close();
    rclcpp::shutdown();
    return 0;
}