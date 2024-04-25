#pragma once
#include <chrono>
#include <functional>
#include <memory>
#include <regex>
#include <cmath>

#include <string>
#include <vector>
#include <deque>
#include <array>
#include <map>
#include <set>

#include <thread>
#include <atomic>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "vehicle_interfaces/msg_json.h"
#include "vehicle_interfaces/params.h"
#include "vehicle_interfaces/utils.h"
#include "vehicle_interfaces/msg/controller_info.hpp"
#include "vehicle_interfaces/srv/controller_info_req.hpp"
#include "vehicle_interfaces/msg/control_server_status.hpp"
#include "vehicle_interfaces/srv/control_server.hpp"

#define SERVICE_NAME "controlserver_0"
#define NODE_NAME "controlservertest_0_node"

using namespace std::chrono_literals;

std::atomic<bool> __global_exit_flag = false;

class Params : public vehicle_interfaces::GenericParams
{
public:
    std::string serviceName = "controlserver_0";

private:
    void _getParams()
    {
        this->get_parameter("serviceName", this->serviceName);
    }

public:
    Params(std::string nodeName) : vehicle_interfaces::GenericParams(nodeName)
    {
        this->declare_parameter<std::string>("serviceName", this->serviceName);
        this->_getParams();
    }
};

void PrintControlServer(const vehicle_interfaces::msg::ControlServerStatus& res)
{
    printf("Controller service name: %s\n", res.controller_service_name.c_str());
    printf("Output timer   (%-5.3lfms): %s\n", res.server_output_period_ms, res.server_output_timer_status == vehicle_interfaces::msg::ControlServerStatus::TIMER_STATUS_START ? "On" : "Off");
    printf("Safety timer   (%-5.3lfms): %s\n", res.server_safety_period_ms, res.server_safety_timer_status == vehicle_interfaces::msg::ControlServerStatus::TIMER_STATUS_START ? "On" : "Off");
    printf("IDClient timer (%-5.3lfms): %s\n", res.server_idclient_period_ms, res.server_idclient_timer_status == vehicle_interfaces::msg::ControlServerStatus::TIMER_STATUS_START ? "On" : "Off");
    printf("Publish timer  (%-5.3lfms): %s\n", res.server_publish_period_ms, res.server_publish_timer_status == vehicle_interfaces::msg::ControlServerStatus::TIMER_STATUS_START ? "On" : "Off");
}

vehicle_interfaces::ReasonResult<bool> SendRequest(std::string serviceName, vehicle_interfaces::srv::ControlServer::Request::SharedPtr req, vehicle_interfaces::msg::ControlServerStatus& dst)
{
    auto node = rclcpp::Node::make_shared("controlservertest_tmp_node");
    auto client = node->create_client<vehicle_interfaces::srv::ControlServer>(serviceName);
    auto result = client->async_send_request(req);
#if ROS_DISTRO == 0
    if (rclcpp::spin_until_future_complete(node, result, 500ms) == rclcpp::executor::FutureReturnCode::SUCCESS)
#else
    if (rclcpp::spin_until_future_complete(node, result, 500ms) == rclcpp::FutureReturnCode::SUCCESS)
#endif
    {
        auto res = result.get();
        dst = res->status;
        return { res->response, res->reason };
    }
    return { false, "Request failed." };
}

vehicle_interfaces::ReasonResult<bool> SendRequest(std::string serviceName, vehicle_interfaces::srv::ControllerInfoReq::Request::SharedPtr req, std::vector<vehicle_interfaces::msg::ControllerInfo>& dst)
{
    auto node = rclcpp::Node::make_shared("controlservertest_tmp_node");
    auto client = node->create_client<vehicle_interfaces::srv::ControllerInfoReq>(serviceName);
    auto result = client->async_send_request(req);
#if ROS_DISTRO == 0
    if (rclcpp::spin_until_future_complete(node, result, 500ms) == rclcpp::executor::FutureReturnCode::SUCCESS)
#else
    if (rclcpp::spin_until_future_complete(node, result, 500ms) == rclcpp::FutureReturnCode::SUCCESS)
#endif
    {
        auto res = result.get();
        dst = res->control_info_vec;
        return { res->response, "" };
    }
    return { false, "Request failed." };
}

int main(int argc, char** argv)
{
    // ctrl-c handler
    signal(SIGINT, 
        [](int)
        {
            __global_exit_flag = true;
        });

    rclcpp::init(argc, argv);
    auto params = std::make_shared<Params>("controlservertest_params_node");

    printf("/** \n\
 * Get information (ControlServerStatus): \n\
 *   i \n\
 * \n\
 * Get controller list (ControllerInfo): \n\
 *   c \n\
 * \n\
 * Remove controller (ControlServer): \n\
 *   r <service_name> \n\
 * \n\
 * Set output controller (ControlServer): \n\
 *   s <service_name> \n\
 * \n\
 * Set output signal timer (ControlServer): \n\
 *   st <timer_id> <TIMER_STATUS_XXX>\n\
 *   sp <timer_id> <period>\n\
 * \n\
 * Quit: \n\
 *   q\n\
 */\n");

    while (!__global_exit_flag)
    {
        std::this_thread::sleep_for(100ms);
        printf(">");
        std::string inputStr;
        std::getline(std::cin, inputStr);

        if (inputStr.size() < 1)
            continue;
        auto inputStrVec = vehicle_interfaces::split(inputStr, ", ");
        if (inputStrVec.size() == 1 && inputStrVec[0] == "q")
            __global_exit_flag = true;
        else if (inputStrVec.size() == 1 && inputStrVec[0] == "i")
        {
            auto req = std::make_shared<vehicle_interfaces::srv::ControlServer::Request>();
            vehicle_interfaces::msg::ControlServerStatus dst;
            auto res = SendRequest(params->serviceName, req, dst);
            if (res.result)
                std::cout << vehicle_interfaces::msg_show::ControlServerStatus::hprint(dst) << std::endl;
            else
                std::cerr << "Request failed: " << res.reason << std::endl;
        }
        else if (inputStrVec.size() == 1 && inputStrVec[0] == "c")
        {
            auto req = std::make_shared<vehicle_interfaces::srv::ControllerInfoReq::Request>();
            req->service_name = "all";

            std::vector<vehicle_interfaces::msg::ControllerInfo> dst;
            auto res = SendRequest(params->serviceName + "_ControllerInfoReq", req, dst);
            if (res.result)
            {
                vehicle_interfaces::HierarchicalPrint hp;
                hp.push(0, "Controller list");
                for (int i = 0; i < dst.size(); i++)
                {
                    hp.push(1, "Controller " + std::to_string(i));
                    hp.push(2, "Service name", vehicle_interfaces::msg_show::ControllerInfo::hprint(dst[i]));
                }
                std::cout << hp << std::endl;
            }
            else
                std::cerr << "Request failed: " << res.reason << std::endl;
        }
        else if (inputStrVec.size() < 2)
            continue;

        // inputStrVec size is at least 2.
        if (inputStrVec[0] == "r")
        {
            auto req = std::make_shared<vehicle_interfaces::srv::ControlServer::Request>();
            req->request = vehicle_interfaces::msg::ControlServerStatus();
            req->request.controller_action = vehicle_interfaces::msg::ControlServerStatus::CONTROLLER_ACTION_REMOVE;
            req->request.controller_service_name = inputStrVec[1];

            vehicle_interfaces::msg::ControlServerStatus dst;
            auto res = SendRequest(params->serviceName, req, dst);
            if (res.result)
            {
                printf("================ Control Server\n");
                PrintControlServer(dst);
            }
            else
                std::cerr << "Request failed: " << res.reason << std::endl;
        }
        else if (inputStrVec[0] == "s")
        {
            auto req = std::make_shared<vehicle_interfaces::srv::ControlServer::Request>();
            req->request = vehicle_interfaces::msg::ControlServerStatus();
            req->request.controller_action = vehicle_interfaces::msg::ControlServerStatus::CONTROLLER_ACTION_SELECT;
            req->request.controller_service_name = inputStrVec[1];

            vehicle_interfaces::msg::ControlServerStatus dst;
            auto res = SendRequest(params->serviceName, req, dst);
            if (res.result)
            {
                printf("================ Control Server\n");
                PrintControlServer(dst);
            }
            else
                std::cerr << "Request failed: " << res.reason << std::endl;
        }
        else if (inputStrVec[0] == "st" && inputStrVec.size() > 2)
        {
            auto req = std::make_shared<vehicle_interfaces::srv::ControlServer::Request>();
            req->request = vehicle_interfaces::msg::ControlServerStatus();
            req->request.server_action = vehicle_interfaces::msg::ControlServerStatus::SERVER_ACTION_SET_TIMER;
            try
            {
                if (inputStrVec[1] == "0")
                    req->request.server_output_timer_status = std::stoi(inputStrVec[2]);
                else if (inputStrVec[1] == "1")
                    req->request.server_safety_timer_status = std::stoi(inputStrVec[2]);
                else if (inputStrVec[1] == "2")
                    req->request.server_idclient_timer_status = std::stoi(inputStrVec[2]);
                else if (inputStrVec[1] == "3")
                    req->request.server_publish_timer_status = std::stoi(inputStrVec[2]);
                else
                    continue;
            }
            catch (...)
            {
                printf("Invalid timer status. Should be 0 to 2\n");
            }

            printf("Server action: %d\n", req->request.server_action);
            printf("Output timer status: %d\n", req->request.server_output_timer_status);
            printf("Safety timer status: %d\n", req->request.server_safety_timer_status);
            printf("IDClient timer status: %d\n", req->request.server_idclient_timer_status);
            printf("Publish timer status: %d\n", req->request.server_publish_timer_status);

            vehicle_interfaces::msg::ControlServerStatus dst;
            auto res = SendRequest(params->serviceName, req, dst);
            if (res.result)
            {
                printf("================ Control Server\n");
                PrintControlServer(dst);
            }
            else
                std::cerr << "Request failed: " << res.reason << std::endl;
        }
        else if (inputStrVec[0] == "sp" && inputStrVec.size() > 2)
        {
            auto req = std::make_shared<vehicle_interfaces::srv::ControlServer::Request>();
            req->request = vehicle_interfaces::msg::ControlServerStatus();
            req->request.server_action = vehicle_interfaces::msg::ControlServerStatus::SERVER_ACTION_SET_PERIOD;
            try
            {
                if (inputStrVec[1] == "0")
                    req->request.server_output_period_ms = std::stod(inputStrVec[2]);
                else if (inputStrVec[1] == "1")
                    req->request.server_safety_period_ms = std::stod(inputStrVec[2]);
                else if (inputStrVec[1] == "2")
                    req->request.server_idclient_period_ms = std::stod(inputStrVec[2]);
                else if (inputStrVec[1] == "3")
                    req->request.server_publish_period_ms = std::stod(inputStrVec[2]);
                else
                    continue;
            }
            catch(...)
            {
                printf("Invalid period. Should be a float number\n");
            }

            printf("Server action: %d\n", req->request.server_action);
            printf("Output timer period: %.2lf\n", req->request.server_output_period_ms);
            printf("Safety timer period: %.2lf\n", req->request.server_safety_period_ms);
            printf("IDClient timer period: %.2lf\n", req->request.server_idclient_period_ms);
            printf("Publish timer period: %.2lf\n", req->request.server_publish_period_ms);

            vehicle_interfaces::msg::ControlServerStatus dst;
            auto res = SendRequest(params->serviceName, req, dst);
            if (res.result)
            {
                printf("================ Control Server\n");
                PrintControlServer(dst);
            }
            else
                std::cerr << "Request failed: " << res.reason << std::endl;
        }
    }

    rclcpp::shutdown();
    return 0;
}
