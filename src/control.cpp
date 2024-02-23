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
#include "vehicle_interfaces/utils.h"
#include "vehicle_interfaces/msg/controller_info.hpp"
#include "vehicle_interfaces/srv/controller_info_req.hpp"
#include "vehicle_interfaces/msg/control_server.hpp"
#include "vehicle_interfaces/srv/control_server.hpp"

#define SERVICE_NAME "controlserver_0"
#define NODE_NAME "controlservertest_0_node"

using namespace std::chrono_literals;

class TestNode : public rclcpp::Node
{
private:
    rclcpp::Node::SharedPtr controlServerClientNode_;
    rclcpp::Client<vehicle_interfaces::srv::ControlServer>::SharedPtr controlServerClient_;

    rclcpp::Node::SharedPtr controllerInfoReqClientNode_;
    rclcpp::Client<vehicle_interfaces::srv::ControllerInfoReq>::SharedPtr controllerInfoReqClient_;

public:
    TestNode(const std::string& nodeName, const std::string& serviceName) : rclcpp::Node(nodeName)
    {
        this->controlServerClientNode_ = std::make_shared<rclcpp::Node>(nodeName + "_controlserver_client");
        this->controlServerClient_ = this->controlServerClientNode_->create_client<vehicle_interfaces::srv::ControlServer>(serviceName);

        this->controllerInfoReqClientNode_ = std::make_shared<rclcpp::Node>(nodeName + "_controllerinfo_req_client");
        this->controllerInfoReqClient_ = this->controllerInfoReqClientNode_->create_client<vehicle_interfaces::srv::ControllerInfoReq>(serviceName + "_Req");
    }

    bool sendRequest(const std::shared_ptr<vehicle_interfaces::srv::ControlServer::Request> req, vehicle_interfaces::msg::ControlServer& res)
    {
        auto result = this->controlServerClient_->async_send_request(req);
#if ROS_DISTRO == 0
        if (rclcpp::spin_until_future_complete(this->controlServerClientNode_, result, 200ms) == rclcpp::executor::FutureReturnCode::SUCCESS)
#else
        if (rclcpp::spin_until_future_complete(this->controlServerClientNode_, result, 200ms) == rclcpp::FutureReturnCode::SUCCESS)
#endif
        {
            auto response = result.get();
            res = response->status;
            return response->response;
        }
        return false;
    }

    bool sendRequest(const std::shared_ptr<vehicle_interfaces::srv::ControllerInfoReq::Request> req, std::vector<vehicle_interfaces::msg::ControllerInfo>& controlInfoVec)
    {
        auto result = this->controllerInfoReqClient_->async_send_request(req);
#if ROS_DISTRO == 0
        if (rclcpp::spin_until_future_complete(this->controllerInfoReqClientNode_, result, 200ms) == rclcpp::executor::FutureReturnCode::SUCCESS)
#else
        if (rclcpp::spin_until_future_complete(this->controllerInfoReqClientNode_, result, 200ms) == rclcpp::FutureReturnCode::SUCCESS)
#endif
        {
            auto response = result.get();
            if (response->response)
                controlInfoVec = response->control_info_vec;
            return response->response;
        }
        return false;
    }
};

void PrintControllerInfo(const vehicle_interfaces::msg::ControllerInfo& info)
{
    printf("%s %-15s %-8s %-4.4f %-4.4f %-3d %s\n", info.service_name.c_str(), 
        info.msg_type == vehicle_interfaces::msg::ControllerInfo::MSG_TYPE_CHASSIS ? "Chassis" : "SteeringWheel", 
        info.controller_mode == vehicle_interfaces::msg::ControllerInfo::CONTROLLER_MODE_TOPIC ? "Topic" : "Service", 
        info.timeout_ms, 
        info.period_ms, 
        info.privilege, 
        info.pub_type == vehicle_interfaces::msg::ControllerInfo::PUB_TYPE_NONE ? "PUB_TYPE_NONE" : 
            (info.pub_type == vehicle_interfaces::msg::ControllerInfo::PUB_TYPE_CONTROLLER ? "PUB_TYPE_CONTROLLER" : 
            (info.pub_type == vehicle_interfaces::msg::ControllerInfo::PUB_TYPE_CONTROLSERVER ? "PUB_TYPE_CONTROLSERVER" : "PUB_TYPE_BOTH")));
}

void PrintControlServer(const vehicle_interfaces::msg::ControlServer& res)
{
    printf("Controller service name: %s\n", res.controller_service_name.c_str());
    printf("Output timer   (%-5.3fms): %s\n", res.server_output_period_ms, res.server_output_timer_status == vehicle_interfaces::msg::ControlServer::TIMER_STATUS_START ? "On" : "Off");
    printf("Safety timer   (%-5.3fms): %s\n", res.server_safety_period_ms, res.server_safety_timer_status == vehicle_interfaces::msg::ControlServer::TIMER_STATUS_START ? "On" : "Off");
    printf("IDClient timer (%-5.3fms): %s\n", res.server_idclient_period_ms, res.server_idclient_timer_status == vehicle_interfaces::msg::ControlServer::TIMER_STATUS_START ? "On" : "Off");
    printf("Publish timer  (%-5.3fms): %s\n", res.server_publish_period_ms, res.server_publish_timer_status == vehicle_interfaces::msg::ControlServer::TIMER_STATUS_START ? "On" : "Off");
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestNode>(NODE_NAME, SERVICE_NAME);
    rclcpp::executors::SingleThreadedExecutor* executor = new rclcpp::executors::SingleThreadedExecutor();
    executor->add_node(node);
    std::thread execTh(vehicle_interfaces::SpinExecutor, executor, "control", 1000.0);

    bool stopF = false;
    printf("/** \n\
 * Get controller list (ControllerInfo): \n\
 *   g <service_name> \n\
 *   g all \n\
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
    while (!stopF)
    {
        printf(">");
        std::string inputStr;
        std::getline(std::cin, inputStr);

        if (inputStr.size() < 1)
            continue;
        auto inputStrVec = vehicle_interfaces::split(inputStr, ", ");
        if (inputStrVec.size() == 1 && inputStrVec[0] == "q")
            stopF = true;
        else if (inputStrVec.size() < 2)
            continue;
        if (inputStrVec[0] == "g")
        {
            auto req = std::make_shared<vehicle_interfaces::srv::ControllerInfoReq::Request>();
            req->service_name = "all";

            std::vector<vehicle_interfaces::msg::ControllerInfo> controlInfoVec;
            if (node->sendRequest(req, controlInfoVec))
            {
                printf("================ Controller List\n");
                for (int i = 0; i < controlInfoVec.size(); i++)
                {
                    printf("[%d] ", i);
                    PrintControllerInfo(controlInfoVec[i]);
                }
            }
            else
            {
                printf("Get controller info failed\n");
            }
        }
        else if (inputStrVec[0] == "r")
        {
            auto req = std::make_shared<vehicle_interfaces::srv::ControlServer::Request>();
            req->request.controller_action == vehicle_interfaces::msg::ControlServer::CONTROLLER_ACTION_REMOVE;
            req->request.controller_service_name = inputStrVec[1];

            vehicle_interfaces::msg::ControlServer res;
            if (!node->sendRequest(req, res))
                printf("Request control server failed\n");
            printf("================ Control Server\n");
            PrintControlServer(res);
        }
        else if (inputStrVec[0] == "s")
        {
            auto req = std::make_shared<vehicle_interfaces::srv::ControlServer::Request>();
            req->request.controller_action == vehicle_interfaces::msg::ControlServer::CONTROLLER_ACTION_SELECT;
            req->request.controller_service_name = inputStrVec[1];

            vehicle_interfaces::msg::ControlServer res;
            if (!node->sendRequest(req, res))
                printf("Request control server failed\n");
            printf("================ Control Server\n");
            PrintControlServer(res);
        }
        else if (inputStrVec[0] == "st" && inputStrVec.size() > 2)
        {
            auto req = std::make_shared<vehicle_interfaces::srv::ControlServer::Request>();
            req->request.controller_action == vehicle_interfaces::msg::ControlServer::SERVER_ACTION_SET_TIMER;
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

            vehicle_interfaces::msg::ControlServer res;
            if (!node->sendRequest(req, res))
                printf("Request control server failed\n");
            printf("================ Control Server\n");
            PrintControlServer(res);
        }
        else if (inputStrVec[0] == "sp" && inputStrVec.size() > 2)
        {
            auto req = std::make_shared<vehicle_interfaces::srv::ControlServer::Request>();
            req->request.controller_action == vehicle_interfaces::msg::ControlServer::SERVER_ACTION_SET_PERIOD;
            try
            {
                if (inputStrVec[1] == "0")
                    req->request.server_output_timer_status = std::stod(inputStrVec[2]);
                else if (inputStrVec[1] == "1")
                    req->request.server_safety_timer_status = std::stod(inputStrVec[2]);
                else if (inputStrVec[1] == "2")
                    req->request.server_idclient_timer_status = std::stod(inputStrVec[2]);
                else if (inputStrVec[1] == "3")
                    req->request.server_publish_timer_status = std::stod(inputStrVec[2]);
                else
                    continue;
            }
            catch(...)
            {
                printf("Invalid period. Should be a float number\n");
            }

            vehicle_interfaces::msg::ControlServer res;
            if (!node->sendRequest(req, res))
                printf("Request control server failed\n");
            printf("================ Control Server\n");
            PrintControlServer(res);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    executor->cancel();
    execTh.join();
    delete executor;
    rclcpp::shutdown();
    return 0;
}
