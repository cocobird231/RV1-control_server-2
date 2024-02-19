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
#include "vehicle_interfaces/control.h"
#include "vehicle_interfaces/utils.h"
#include "vehicle_interfaces/msg/chassis_info.hpp"
#include "vehicle_interfaces/msg/control_chassis.hpp"
#include "vehicle_interfaces/msg/controller_info.hpp"
#include "vehicle_interfaces/msg/control_steering_wheel.hpp"

#include "vehicle_interfaces/srv/control_chassis_reg.hpp"
#include "vehicle_interfaces/srv/control_chassis_req.hpp"
#include "vehicle_interfaces/srv/controller_info_reg.hpp"
#include "vehicle_interfaces/srv/controller_info_req.hpp"
#include "vehicle_interfaces/srv/control_steering_wheel_reg.hpp"
#include "vehicle_interfaces/srv/control_steering_wheel_req.hpp"

#include "vehicle_interfaces/msg/point2d.hpp"

#include "chassis_info.h"
#include "joystick_info.h"
#include "PolynomialSolver.h"
#include "idclient.h"

#define DEG2RAD 3.141592653589793 / 180.0
#define RAD2DEG 180.0 / 3.141592653589793

using namespace std::chrono_literals;

class Params : public vehicle_interfaces::GenericParams
{
private:
    // Callback
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr _paramsCallbackHandler;
    std::function<void(const rclcpp::Parameter)> cbFunc_;
    std::atomic<bool> cbFuncSetF_;

public:
    std::string chassisFilePath = "chassis.json";
    std::string joystickFilePath = "joystick.json";

    std::string internalIDServerIP = "192.168.1.42";
    int internalIDServerPort = 0;// Unused
    int internalIDServerDeviceID = 0;

    std::string externalIDServerIP = "61.220.23.240";
    std::string externalIDServerPort = "10000";
    std::string externalIDServerDeviceID = "CAR1";

    std::string serviceName = "controlserver";
    std::string topicName = "controlserver";
    double publishInterval_ms = 0.05;

    bool enableOutput = false;
    double outputPeriod_ms = 0.05;

private:
    void _getParams()
    {
        this->get_parameter("chassisFilePath", this->chassisFilePath);
        this->get_parameter("joystickFilePath", this->joystickFilePath);

        this->get_parameter("internalIDServerIP", this->internalIDServerIP);
        this->get_parameter("internalIDServerPort", this->internalIDServerPort);
        this->get_parameter("internalIDServerDeviceID", this->internalIDServerDeviceID);

        this->get_parameter("externalIDServerIP", this->externalIDServerIP);
        this->get_parameter("externalIDServerPort", this->externalIDServerPort);
        this->get_parameter("externalIDServerDeviceID", this->externalIDServerDeviceID);

        this->get_parameter("serviceName", this->serviceName);
        this->get_parameter("topicName", this->topicName);
        this->get_parameter("publishInterval_ms", this->publishInterval_ms);

        this->get_parameter("enableOutput", this->enableOutput);
        this->get_parameter("outputPeriod_ms", this->outputPeriod_ms);
    }

    rcl_interfaces::msg::SetParametersResult _paramsCallback(const std::vector<rclcpp::Parameter>& params)
    {
        rcl_interfaces::msg::SetParametersResult ret;
        ret.successful = true;
        ret.reason = "";

        if (!this->cbFuncSetF_)
            return ret;

        for (const auto& param : params)
        {
            try
            {
                this->cbFunc_(param);
            }
            catch (...)
            {
                ret.successful = false;
                ret.reason = "[Params::_paramsCallback] Caught Unknown Exception!";
            }
        }

        return ret;
    }

public:
    Params(std::string nodeName) : vehicle_interfaces::GenericParams(nodeName), 
        cbFuncSetF_(false)
    {
        this->declare_parameter<std::string>("chassisFilePath", this->chassisFilePath);
        this->declare_parameter<std::string>("joystickFilePath", this->joystickFilePath);

        this->declare_parameter<std::string>("internalIDServerIP", this->internalIDServerIP);
        this->declare_parameter<int>("internalIDServerPort", this->internalIDServerPort);
        this->declare_parameter<int>("internalIDServerDeviceID", this->internalIDServerDeviceID);

        this->declare_parameter<std::string>("externalIDServerIP", this->externalIDServerIP);
        this->declare_parameter<std::string>("externalIDServerPort", this->externalIDServerPort);
        this->declare_parameter<std::string>("externalIDServerDeviceID", this->externalIDServerDeviceID);

        this->declare_parameter<std::string>("serviceName", this->serviceName);
        this->declare_parameter<std::string>("topicName", this->topicName);
        this->declare_parameter<double>("publishInterval_ms", this->publishInterval_ms);

        this->declare_parameter<bool>("enableOutput", this->enableOutput);
        this->declare_parameter<double>("outputPeriod_ms", this->outputPeriod_ms);

        this->_getParams();

        this->_paramsCallbackHandler = this->add_on_set_parameters_callback(std::bind(&Params::_paramsCallback, this, std::placeholders::_1));
    }

    void addCallbackFunc(const std::function<void(const rclcpp::Parameter)>& callback)
    {
        this->cbFunc_ = callback;
        this->cbFuncSetF_ = true;
    }
};



/**
 * TODO:
 * [x] Control reg server
 * [x] Control req server
 * [x] Add controller
 * [x] Init chassis info (e.g. pred model)
 * [x] cvtSignal (rewrite from py_controlserver/genMotorPWM.py)
 * [x] wireless controller
 * [x] Read chassis.json
 * [x] Read joystick.json
 * [x] Connect jim id server
 * [x] Add safety thread
 * [x] Controller switch (consider timestamp)
 * [x] Publisher thread for chassis signal
 * [ ] Higher priority interruption
 * [x] Dynamic controller removal
 */
class ControlServer : public vehicle_interfaces::VehicleServiceNode
{
private:
    std::shared_ptr<Params> params_;// ControlServer parameters.
    vehicle_interfaces::msg::ChassisInfo cInfo_;// Chassis architecture information.
    std::atomic<bool> cInfoF_;// Check valid cInfo_.

    // Wireless controller.
    FILE *joystick_;// Joystick device.
    JoystickInfo jInfo_;// joystick button function configuration.
    std::atomic<bool> jInfoF_;// Check valid jInfo_.
    vehicle_interfaces::Timer* joystickTm_;// Call _joystickCbFunc().
    std::atomic<bool> wirelessBrakeF_;// State of wireless brake.
    std::atomic<bool> safetyOverControlF_;// State of safety over control.
    std::atomic<bool> wirelessF_;// Is wireless controller detected.

    // Controller storage.
    std::map<std::string, std::shared_ptr<vehicle_interfaces::ControlServerController> > controllerMap_;// Store registered controller.
    std::map<std::string, rclcpp::executors::SingleThreadedExecutor*> controllerExecMap_;
    std::map<std::string, std::thread*> controllerThMap_;
    std::deque<std::string> controllerIdx_;// For wireless joystick select controller by index.
    std::mutex controllerLock_;// Lock controllerMap_, controllerExecMap_, controllerThMap_ and controllerIdx_.

    // Controller switch.
    std::atomic<size_t> selectedControllerIdx_;
    std::string selectedControllerServiceName_;
    vehicle_interfaces::Timer* controllerSwitchTm_;// Call _controllerSwitchCbFunc().

    // Safety check.
    std::array<float, 8> emPs_;// Store 8-direction emergency percentages.
    std::mutex emPsLock_;// Lock emPs_.
    std::atomic<bool> safetyF_;// Check valid emPs_.
    vehicle_interfaces::Timer* safetyTm_;// Call _safetyCbFunc().

    // JimIDClient socket.
    JimIDClient *idclient_;
    std::atomic<bool> idclientF_;// Check valid idclient_.
    vehicle_interfaces::Timer* idclientTm_;// Call _idclientCbFunc().

    // Publisher.
    rclcpp::Publisher<vehicle_interfaces::msg::Chassis>::SharedPtr publisher_;// Publisher for chassis signal.
    vehicle_interfaces::msg::Chassis publishMsg_;// The output signal of controller switch.
    u_int64_t publishFrameId_;// Frame count for publish message.
    std::mutex publishMsgLock_;// Lock publishMsg_.
    vehicle_interfaces::Timer* publishTm_;// Call _publishCbFunc().

    // ControllerInfoReg and ControllerInfoReq server.
    rclcpp::Service<vehicle_interfaces::srv::ControllerInfoReg>::SharedPtr regServer_;// For Controller register.
    rclcpp::Service<vehicle_interfaces::srv::ControllerInfoReq>::SharedPtr reqServer_;// Get current Controller info.

    // Node control.
    std::atomic<bool> exitF_;



    /**
     * Chassis.
     */

    // Motor direction correction.
    std::vector<double> driveMotorDirCorr_;// Drive motor direction correction.
    std::vector<double> steeringMotorDirCorr_;// Steering motor direction correction.
    std::vector<double> brakeMotorDirCorr_;// Brake motor direction correction.

    // Motor predictor.
    std::vector<PolynomialSolver::PolynomialSolver*> driveMotorPred_;// Drive motor value mapping predictor.
    std::vector<PolynomialSolver::PolynomialSolver*> steeringMotorPred_;// Steering motor value mapping predictor.
    std::vector<PolynomialSolver::PolynomialSolver*> brakeMotorPred_;// Brake motor value mapping predictor.
    std::atomic<bool> driveMotorPredF_;
    std::atomic<bool> steeringMotorPredF_;
    std::atomic<bool> brakeMotorPredF_;

    // Chassis parameters. TO BE REWRITE.
    double CHASSIS_WHEELBASE;// unit: mm
    double CHASSIS_TRACK;// unit: mm
    double CHASSIS_CAR_LENGTH = 2200;// unit: mm CHASSIS_WHEELBASE
    double CHASSIS_CAR_WIDTH = 2080;// unit: mm CHASSIS_TRACK
    double CHASSIS_MAX_RADIUS_ACKERMANN = 70000;// unit: mm
    double CHASSIS_MIN_RADIUS_ACKERMANN = 7100;// unit: mm
    double CHASSIS_MAX_RADIUS_4WS = 40000;// unit: mm
    double CHASSIS_MIN_RADIUS_4WS = 4100;// unit: mm
    double CHASSIS_MIN_WHEEL_STEER_ANGLE = 0;// unit: degree
    double CHASSIS_MAX_WHEEL_STEER_ANGLE = 20;// unit: degree

    double STEERINGWHEEL_TOLERANCE = 1500;

private:
    template <typename T>
    void _safeSave(T* ptr, const T value, std::mutex& lock)
    {
        std::lock_guard<std::mutex> _lock(lock);
        *ptr = value;
    }

    template <typename T>
    T _safeCall(const T* ptr, std::mutex& lock)
    {
        std::lock_guard<std::mutex> _lock(lock);
        return *ptr;
    }



    /**
     * ================================================================
     * ChassisInfo initialization.
     * ================================================================
     */

    /** Check chassis configuration. */
    void _initChassisInfo(const vehicle_interfaces::msg::ChassisInfo& info)
    {
        // Check wheel_position size.
        if (info.wheel_position.size() != info.vehicle_type)
            throw "[ControlServer::_initChassisInfo] wheel_position size error.";
        
        // wheel_position validation.
        vehicle_interfaces::msg::Point2d ptFL, ptBR;
        ptFL.x = 0;
        ptFL.y = 0;
        ptBR.x = 0;
        ptBR.y = 0;

        for (const auto& wheel : info.wheel_position)
        {
            ptFL.x = wheel.x < ptFL.x ? wheel.x : ptFL.x;
            ptFL.y = wheel.y > ptFL.y ? wheel.y : ptFL.y;
            ptBR.x = wheel.x > ptBR.x ? wheel.x : ptBR.x;
            ptBR.y = wheel.y < ptBR.y ? wheel.y : ptBR.y;
        }

        if (ptFL.x != 0 || ptFL.y != 0 || ptBR.x <= 0 || ptBR.y >= 0)
            throw "[ControlServer::_initChassisInfo] wheel_position value error.";
        this->CHASSIS_WHEELBASE = -ptBR.y;
        this->CHASSIS_TRACK = ptBR.x;
    }

    /** Check motor configuration. */
    void _initMotorMapping(const vehicle_interfaces::msg::ChassisInfo& info)
    {
        // Motor direction correction.
        if (info.drive_motor_correction_vec.size() != info.vehicle_type)
            throw "[ControlServer::_initMotorMapping] drive_motor_correction_vec size error.";
        if (info.steering_motor_correction_vec.size() != info.vehicle_type)
            throw "[ControlServer::_initMotorMapping] steering_motor_correction_vec size error.";
        if (info.brake_motor_correction_vec.size() != info.vehicle_type)
            throw "[ControlServer::_initMotorMapping] brake_motor_correction_vec size error.";
        this->driveMotorDirCorr_ = info.drive_motor_correction_vec;
        this->steeringMotorDirCorr_ = info.steering_motor_correction_vec;
        this->brakeMotorDirCorr_ = info.brake_motor_correction_vec;

        size_t mapSz;
        mapSz = info.drive_motor_mapping_vec.size();
        if (mapSz > 0)
        {
            if (mapSz != info.vehicle_type)
                throw "[ControlServer::_initMotorMapping] drive_motor_mapping_vec size error.";
            for (int i = 0; i < mapSz; i++)
            {
                size_t inSz = info.drive_motor_mapping_vec[i].input_vec.size();
                size_t outSz = info.drive_motor_mapping_vec[i].output_vec.size();
                if (inSz != outSz || inSz <= 0)
                {
                    for (auto& i : this->driveMotorPred_)
                        delete i;
                    this->driveMotorPred_.clear();
                    throw "[ControlServer::_initMotorMapping] drive_motor_mapping_vec value error.";
                }
                this->driveMotorPred_.push_back(new PolynomialSolver::PolynomialSolver(info.drive_motor_mapping_vec[i].input_vec, 
                                                                                        info.drive_motor_mapping_vec[i].output_vec, 
                                                                                        3));
            }
            this->driveMotorPredF_ = true;
        }
        mapSz = info.steering_motor_mapping_vec.size();
        if (mapSz > 0)
        {
            if (mapSz != info.vehicle_type)
                throw "[ControlServer::_initMotorMapping] steering_motor_mapping_vec size error.";
            for (int i = 0; i < mapSz; i++)
            {
                size_t inSz = info.steering_motor_mapping_vec[i].input_vec.size();
                size_t outSz = info.steering_motor_mapping_vec[i].output_vec.size();
                if (inSz != outSz || inSz <= 0)
                {
                    for (auto& i : this->steeringMotorPred_)
                        delete i;
                    this->steeringMotorPred_.clear();
                    throw "[ControlServer::_initMotorMapping] steering_motor_mapping_vec value error.";
                }
                this->steeringMotorPred_.push_back(new PolynomialSolver::PolynomialSolver(info.steering_motor_mapping_vec[i].input_vec, 
                                                                                            info.steering_motor_mapping_vec[i].output_vec, 
                                                                                            3));
            }
            this->steeringMotorPredF_ = true;
        }
        mapSz = info.brake_motor_mapping_vec.size();
        if (mapSz > 0)
        {
            if (mapSz != info.vehicle_type)
                throw "[ControlServer::_initMotorMapping] brake_motor_mapping_vec size error.";
            for (int i = 0; i < mapSz; i++)
            {
                size_t inSz = info.brake_motor_mapping_vec[i].input_vec.size();
                size_t outSz = info.brake_motor_mapping_vec[i].output_vec.size();
                if (inSz != outSz || inSz <= 0)
                {
                    for (auto& i : this->brakeMotorPred_)
                        delete i;
                    this->brakeMotorPred_.clear();
                    throw "[ControlServer::_initMotorMapping] brake_motor_mapping_vec value error.";
                }
                this->brakeMotorPred_.push_back(new PolynomialSolver::PolynomialSolver(info.brake_motor_mapping_vec[i].input_vec, 
                                                                                        info.brake_motor_mapping_vec[i].output_vec, 
                                                                                        3));
            }
            this->brakeMotorPredF_ = true;
        }
    }



    /**
     * ================================================================
     * Chassis signal convert.
     * ================================================================
     */

    /**
     * Predict drive motor from RPM to PWM value.
     * @param[in] rpms input RPM vector.
     * @return PWM vector if predict successfully. Otherwise, %n elements of 0 which %n determined by vehicle_type under cInfo_.
     */
    std::vector<float> _driveMotorRPMToPWM(const std::vector<float>& rpms)
    {
        if (!this->driveMotorPredF_)
            return std::vector<float>(this->cInfo_.vehicle_type, 0);

        if (this->driveMotorPred_.size() != rpms.size())
            return std::vector<float>(this->cInfo_.vehicle_type, 0);

        std::vector<float> ret(rpms.size(), 0);

        for (int i = 0; i < rpms.size(); i++)
        {
            double tmp = this->driveMotorPred_[i]->predict(rpms[i]);
            ret[i] = tmp > this->cInfo_.drive_motor_pwm_value.max ? this->cInfo_.drive_motor_pwm_value.max : 
                    (tmp < this->cInfo_.drive_motor_pwm_value.min ? this->cInfo_.drive_motor_pwm_value.min : tmp);
        }
        return ret;
    }

    /**
     * Convert ControlSteeringWheel to ControlChassis (4D4S4B).
     * @param[in] src input controller signal (ControlSteeringWheel).
     * @param[out] dst output chassis signal (ControlChassis).
     * @return true if cInfo_ initialized. Otherwise, return false.
     */
    bool _cvtControlSteeringWheelToControlChassis(const vehicle_interfaces::msg::ControlSteeringWheel& src, vehicle_interfaces::msg::ControlChassis& dst)
    {
        if (!this->cInfoF_ || this->cInfo_.vehicle_type != vehicle_interfaces::msg::ChassisInfo::VEHICLE_TYPE_4D4S4B)
            return false;
        int steeringWheel = src.steering;
        int steeringWheelAbs = abs(src.steering);
        float refSpeed = vehicle_interfaces::LinearMapping1d(src.pedal_throttle, 0, 255, 0, 120);
        dst.drive_motor = { 0, 0, 0, 0 };
        dst.steering_motor = { 0, 0, 0, 0 };
        dst.parking_signal = { 0, 0, 0, 0 };
        std::vector<float> motorDirectionList(4, 1);

        if (steeringWheelAbs < this->STEERINGWHEEL_TOLERANCE)
            steeringWheel = 0;

        if (src.func_0 == 3)
        {
            if (steeringWheel > 0)
                motorDirectionList = { -1, 1, -1, 1 };// 1->-1; 2->1
            else if (steeringWheel < 0)
                motorDirectionList = { 1, -1, 1, -1 };
        }
        else
        {
            if (src.gear == vehicle_interfaces::msg::ControlSteeringWheel::GEAR_DRIVE)
                motorDirectionList = { 1, 1, 1, 1 };
            else if (src.gear == vehicle_interfaces::msg::ControlSteeringWheel::GEAR_REVERSE)
                motorDirectionList = { -1, -1, -1, -1 };
        }

        float innerAng = 0;
        float outerAng = 0;

        float innerVelo = 0;
        float outerVelo = 0;

        if (src.func_0 == 3)// Zero turn
        {
            dst.steering_motor = { -20, 20, 20, -20 };
            dst.drive_motor = this->_driveMotorRPMToPWM({ refSpeed, refSpeed, refSpeed, refSpeed });
        }
        else if (steeringWheel == 0)
        {
            dst.drive_motor = this->_driveMotorRPMToPWM({ refSpeed, refSpeed, refSpeed, refSpeed });
        }

        else if (src.func_0 == 1)// Ackermann
        {
            double steeringWheelAbsCorrection = vehicle_interfaces::GammaCorrection(steeringWheelAbs, 0.2, 0, 32768);
            double steeringRadius = vehicle_interfaces::LinearMapping1d(steeringWheelAbsCorrection, 0, 32768, this->CHASSIS_MAX_RADIUS_ACKERMANN, this->CHASSIS_MIN_RADIUS_ACKERMANN);
            double innerRadius = steeringRadius - this->CHASSIS_CAR_WIDTH / 2;
            double outerRadius = steeringRadius + this->CHASSIS_CAR_WIDTH / 2;
            // Steering
            innerAng = std::atan2(this->CHASSIS_CAR_LENGTH, innerRadius) * RAD2DEG;
            outerAng = std::atan2(this->CHASSIS_CAR_LENGTH, outerRadius) * RAD2DEG;
            // Speed
            innerVelo = refSpeed * std::sqrt(std::pow(innerRadius, 2) + std::pow(this->CHASSIS_CAR_LENGTH, 2)) / steeringRadius;
            outerVelo = refSpeed * std::sqrt(std::pow(outerRadius, 2) + std::pow(this->CHASSIS_CAR_LENGTH, 2)) / steeringRadius;
            float rearInnerVelo = refSpeed * innerRadius / steeringRadius;
            float rearOuterVelo = refSpeed * outerRadius / steeringRadius;
            // Assign lists
            if (steeringWheel > 0)// Turn right, inner: 11
            {
                if (src.gear == vehicle_interfaces::msg::ControlSteeringWheel::GEAR_DRIVE)
                {
                    dst.steering_motor = { innerAng, outerAng, 0, 0 };
                    dst.drive_motor = this->_driveMotorRPMToPWM({ innerVelo, outerVelo, rearInnerVelo, rearOuterVelo });
                }
                else if (src.gear == vehicle_interfaces::msg::ControlSteeringWheel::GEAR_REVERSE)
                {
                    dst.steering_motor = { 0, 0, outerAng, innerAng };
                    dst.drive_motor = this->_driveMotorRPMToPWM({ rearOuterVelo, rearInnerVelo, outerVelo, innerVelo });
                }
            }
            else// Turn left
            {
                innerAng = -innerAng;
                outerAng = -outerAng;
                if (src.gear == vehicle_interfaces::msg::ControlSteeringWheel::GEAR_DRIVE)
                {
                    dst.steering_motor = { outerAng, innerAng, 0, 0 };
                    dst.drive_motor = this->_driveMotorRPMToPWM({ outerVelo, innerVelo, rearOuterVelo, rearInnerVelo });
                }
                else if (src.gear == vehicle_interfaces::msg::ControlSteeringWheel::GEAR_REVERSE)
                {
                    dst.steering_motor = { 0, 0, innerAng, outerAng };
                    dst.drive_motor = this->_driveMotorRPMToPWM({ rearInnerVelo, rearOuterVelo, innerVelo, outerVelo });
                }
            }
        }

        else if (src.func_0 == 2)// 4ws center baseline
        {
            double steeringWheelAbsCorrection = vehicle_interfaces::GammaCorrection(steeringWheelAbs, 0.2, 0, 32768);
            double steeringRadius = vehicle_interfaces::LinearMapping1d(steeringWheelAbsCorrection, 0, 32768, this->CHASSIS_MAX_RADIUS_4WS, this->CHASSIS_MIN_RADIUS_4WS);
            double innerRadius = steeringRadius - this->CHASSIS_CAR_WIDTH / 2;
            double outerRadius = steeringRadius + this->CHASSIS_CAR_WIDTH / 2;
            // Steering
            innerAng = std::atan2(this->CHASSIS_CAR_LENGTH / 2, innerRadius) * RAD2DEG;
            outerAng = std::atan2(this->CHASSIS_CAR_LENGTH / 2, outerRadius) * RAD2DEG;
            // Speed
            innerVelo = refSpeed * std::sqrt(std::pow(innerRadius, 2) + std::pow(this->CHASSIS_CAR_LENGTH / 2, 2)) / steeringRadius;
            outerVelo = refSpeed * std::sqrt(std::pow(outerRadius, 2) + std::pow(this->CHASSIS_CAR_LENGTH / 2, 2)) / steeringRadius;
            // Assign lists
            if (steeringWheel > 0)// Turn right, inner: 11
            {
                if (src.gear == vehicle_interfaces::msg::ControlSteeringWheel::GEAR_DRIVE)
                {
                    dst.steering_motor = { innerAng, outerAng, -innerAng, -outerAng };
                    dst.drive_motor = this->_driveMotorRPMToPWM({ innerVelo, outerVelo, innerVelo, outerVelo });
                }
                else if (src.gear == vehicle_interfaces::msg::ControlSteeringWheel::GEAR_REVERSE)
                {
                    dst.steering_motor = { -outerAng, -innerAng, outerAng, innerAng };
                    dst.drive_motor = this->_driveMotorRPMToPWM({ outerVelo, innerVelo, outerVelo, innerVelo });
                }
            }
            else// Turn left
            {
                innerAng = -innerAng;
                outerAng = -outerAng;
                if (src.gear == vehicle_interfaces::msg::ControlSteeringWheel::GEAR_DRIVE)
                {
                    dst.steering_motor = { outerAng, innerAng, -outerAng, -innerAng };
                    dst.drive_motor = this->_driveMotorRPMToPWM({ outerVelo, innerVelo, outerVelo, innerVelo });
                }
                else if (src.gear == vehicle_interfaces::msg::ControlSteeringWheel::GEAR_REVERSE)
                {
                    dst.steering_motor = { -innerAng, -outerAng, innerAng, outerAng };
                    dst.drive_motor = this->_driveMotorRPMToPWM({ innerVelo, outerVelo, innerVelo, outerVelo });
                }
            }
        }

        else if (src.func_0 == 4)// parallel
        {
            double steeringWheelAbsCorrection = vehicle_interfaces::GammaCorrection(steeringWheelAbs, 0.2, 0, 32768);
            float steeringAngle = vehicle_interfaces::LinearMapping1d(steeringWheelAbsCorrection, 0, 32768, this->CHASSIS_MIN_WHEEL_STEER_ANGLE, this->CHASSIS_MAX_WHEEL_STEER_ANGLE);
            // Speed
            dst.drive_motor = this->_driveMotorRPMToPWM({ refSpeed, refSpeed, refSpeed, refSpeed });
            // Assign lists
            if (steeringWheel > 0)// Turn right, inner: 11
                dst.steering_motor = { steeringAngle, steeringAngle, steeringAngle, steeringAngle };
            else// Turn left
                dst.steering_motor = { -steeringAngle, -steeringAngle, -steeringAngle, -steeringAngle };
        }

        // Add direction to motors.
        for (int i = 0; i < 4; i++)
        {
            dst.drive_motor[i] *= motorDirectionList[i];
        }

        // Parking or Netural
        if (src.gear == vehicle_interfaces::msg::ControlSteeringWheel::GEAR_PARK || src.pedal_brake > 10)
        {
            dst.drive_motor = { 0, 0, 0, 0 };
            dst.parking_signal = { 1, 1, 1, 1 };
        }
        else if (src.gear == vehicle_interfaces::msg::ControlSteeringWheel::GEAR_NEUTRAL)
            dst.drive_motor = { 0, 0, 0, 0 };
        return true;
    }



    /**
     * ================================================================
     * Add/Remove controller.
     * ================================================================
     */

    /**
     * Check valid controler index. 
     * The selectedControllerIdx_ and selectedControllerServiceName_ will be modified to -1 and "" respectively if idx is invalid.
     * @param[in] lockF determined whether using controllerLock_ or not. Default: true.
     * @return true if index is valid. Otherwise, return false.
     */
    bool _checkSelectedController(bool lockF = true)
    {
        if (lockF)
            std::lock_guard<std::mutex> locker(this->controllerLock_);
        if (this->selectedControllerIdx_ >= 0 && this->selectedControllerIdx_ < this->controllerIdx_.size())
        {
            this->selectedControllerServiceName_ = this->controllerIdx_[this->selectedControllerIdx_];
            return true;
        }
        this->selectedControllerIdx_ = -1;
        this->selectedControllerServiceName_ = "";
        return false;
    }

    /**
     * Add ControlServerController node to map and executor, then new thread to spin.
     * IMPORTANT: This function is using controllerLock_.
     * @param[in] info describes the controller information.
     * @return true if controller added. Otherwise, return false.
     */
    bool _addController(const vehicle_interfaces::msg::ControllerInfo& info)
    {
        std::lock_guard<std::mutex> locker(this->controllerLock_);
        try
        {
            if (this->controllerMap_.find(info.service_name) == this->controllerMap_.end())// service_name not in controllerMap_
            {
                // Add controller to controllerMap_.
                this->controllerMap_[info.service_name] = std::make_shared<vehicle_interfaces::ControlServerController>(this->params_, info);

                // Convert function is required if controller msg type is ControlSteeringWheel.
                if (info.msg_type == vehicle_interfaces::msg::ControllerInfo::MSG_TYPE_STEERING_WHEEL)
                    this->controllerMap_[info.service_name]->setCvtFunc(std::bind(&ControlServer::_cvtControlSteeringWheelToControlChassis, this, std::placeholders::_1, std::placeholders::_2));

                // New executor and add controller.
                this->controllerExecMap_[info.service_name] = new rclcpp::executors::SingleThreadedExecutor();
                this->controllerExecMap_[info.service_name]->add_node(this->controllerMap_[info.service_name]);

                // New thread and spin executor.
                this->controllerThMap_[info.service_name] = new std::thread(vehicle_interfaces::SpinExecutor, this->controllerExecMap_[info.service_name], info.service_name, 1000.0);

                // Record controller name to vector. For controller switch selected by index.
                this->controllerIdx_.push_back(info.service_name);
                return true;
            }
            // Controller already exists.
            RCLCPP_WARN(this->get_logger(), "[ControlServer::_addController] Add %s controller failed: Controller exist.", info.service_name.c_str());
            return false;
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "[ControlServer::_addController] Caught error: %s", e.what());
            return false;
        }
        catch (...)
        {
            RCLCPP_ERROR(this->get_logger(), "[ControlServer::_addController] Caught unexpected error.");
            return false;
        }
    }

    /**
     * Remove ControlServerController node and delete thread and executor.
     * If removed controller was the selected one, the selected index will be set to -1.
     * IMPORTANT: This function is using controllerLock_.
     * @param[in] serviceName the name of controller desired to be removed.
     * @return true if remove successfully. Otherwise, return false.
     */
    bool _removeController(const std::string& serviceName)
    {
        std::lock_guard<std::mutex> locker(this->controllerLock_);
        if (this->controllerMap_.find(serviceName) == this->controllerMap_.end())// serviceName not exist.
            return false;

        // Check valid current index.
        bool currentIdxF = this->_checkSelectedController(false);

        // Start remove process...
        try
        {
            // Remove serviceName under controllerIdx_.
            size_t removeIdx = -1;
            for (int i = 0; i < this->controllerIdx_.size(); i++)
                if (this->controllerIdx_[i] == serviceName)
                {
                    removeIdx = i;
                    break;
                }
            if (removeIdx != -1)
                this->controllerIdx_.erase(this->controllerIdx_.begin() + removeIdx);

            // Retrieve currrent index.
            if (currentIdxF)
            {
                size_t retrieveIdx = -1;
                for (int i = 0; i < this->controllerIdx_.size(); i++)
                    if (this->controllerIdx_[i] == this->selectedControllerServiceName_)
                    {
                        retrieveIdx = i;
                        break;
                    }
                if (retrieveIdx != -1)
                {
                    this->selectedControllerIdx_ = retrieveIdx;
                    this->selectedControllerServiceName_ = this->controllerIdx_[retrieveIdx];
                }
                else
                {
                    this->selectedControllerIdx_ = -1;
                    this->selectedControllerServiceName_ = "";
                }
            }
            else
            {
                this->selectedControllerIdx_ = -1;
                this->selectedControllerServiceName_ = "";
            }

            // Stop executor.
            this->controllerExecMap_[serviceName]->cancel();
            // Wait for spin end.
            this->controllerThMap_[serviceName]->join();

            // Delete thread first
            delete this->controllerThMap_[serviceName];
            this->controllerThMap_.erase(serviceName);

            // Delete executor.
            delete this->controllerExecMap_[serviceName];
            this->controllerExecMap_.erase(serviceName);

            // Delete ControlServerController node.
            this->controllerMap_.erase(serviceName);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "[ControlServer::_removeController] Caught error: %s", e.what());
            return false;
        }
        catch (...)
        {
            RCLCPP_ERROR(this->get_logger(), "[ControlServer::_removeController] Caught unexpected error.");
            return false;
        }
        return true;
    }

    /**
     * Ascent selected controller index.
     * This function is using controllerLock_.
     */
    void _increaseSelectedIdx()
    {
        std::lock_guard<std::mutex> locker(this->controllerLock_);
        if (this->controllerIdx_.size() > 0)
        {
            if (this->selectedControllerIdx_ < this->controllerIdx_.size() - 1)
                this->selectedControllerIdx_++;
            else
                this->selectedControllerIdx_ = this->controllerIdx_.size() - 1;
        }
        else
            this->selectedControllerIdx_ = -1;
    }

    /**
     * Descent selected controller index.
     * This function is using controllerLock_.
     */
    void _decreaseSelectedIdx()
    {
        std::lock_guard<std::mutex> locker(this->controllerLock_);
        if (this->controllerIdx_.size() > 0)
        {
            if (this->selectedControllerIdx_ > 0)
                this->selectedControllerIdx_--;
            else
                this->selectedControllerIdx_ = 0;
        }
        else
            this->selectedControllerIdx_ = -1;
    }



    /**
     * ================================================================
     * Sending signals.
     * ================================================================
     */

    /** Send emergency brake signal. */
    void _brkSignal()
    {
        if (this->idclientF_)
            this->idclientF_ = this->idclient_->sendDriveMotorSignal({ 0, 0, 0, 0 }, { true, true, true, true });
    }



    /**
     * ================================================================
     * Threads.
     * ================================================================
     */

    /**
     * Timer callback function for joystick signal input.
     */
    void _joystickCbFunc()
    {
        // Open joystick.
        if (!this->wirelessF_)
        {
            if (this->joystick_ != nullptr)
                fclose(this->joystick_);
            this->joystick_ = fopen(this->jInfo_.device.c_str(), "rb+");
            if (!this->joystick_)
            {
                this->wirelessF_ = false;
                RCLCPP_ERROR(this->get_logger(), "[ControlServer::_joystickCbFunc] Open %s failed.\n", this->jInfo_.device.c_str());
                return;
            }
            this->wirelessF_ = true;
        }

        // Read joystick data.
        js_event msg;
        const size_t ret = fread(&msg, sizeof(js_event), 1, this->joystick_);
        if (ret == 1)
        {
            printf("time: %-5d, value: %-5d, type: %-5d, number: %-5d\n", msg.time, msg.value, msg.type, msg.number);
            if (js_event_equal(msg, this->jInfo_.brakeBtn))
                this->wirelessBrakeF_ = true;
            else if (js_event_equal(msg, this->jInfo_.releaseBrakeBtn))
                this->wirelessBrakeF_ = false;
            if (js_event_equal(msg, this->jInfo_.enableSafetyBtn))
                this->safetyOverControlF_ = true;
            else if (js_event_equal(msg, this->jInfo_.disableSafetyBtn))
                this->safetyOverControlF_ = false;
            else if (js_event_equal(msg, this->jInfo_.ascentIdxBtn))
                this->_increaseSelectedIdx();
            else if (js_event_equal(msg, this->jInfo_.descentIdxBtn))
                this->_decreaseSelectedIdx();
        }
        else if (feof(this->joystick_))
        {
            this->wirelessF_ = false;
            RCLCPP_ERROR(this->get_logger(), "[ControlServer::_joystickCbFunc] EoF.");
        }
        else if (ferror(this->joystick_))
        {
            this->wirelessF_ = false;
            RCLCPP_ERROR(this->get_logger(), "[ControlServer::_joystickCbFunc] Error reading %s.", this->jInfo_.device.c_str());
        }
    }

    /**
     * Timer callback function for safety check.
     */
    void _safetyCbFunc()
    {
        std::array<float, 8> tmp;
        try
        {
            if (this->getEmergency("nearest", tmp))
            {
                this->_safeSave(&this->emPs_, tmp, this->emPsLock_);
                this->safetyF_ = true;
            }
            this->safetyF_ = false;
            RCLCPP_ERROR(this->get_logger(), "[ControlServer::_safetyCbFunc] Get emergency error.");
        }
        catch (...)
        {
            this->safetyF_ = false;
            RCLCPP_ERROR(this->get_logger(), "[ControlServer::_safetyCbFunc] Get emergency error.");
        }
    }

    /**
     * Timer callback function for idclient.
     */
    void _idclientCbFunc()
    {
        if (!this->idclientF_)// Need reconnect to id server.
        {
            JimIDClientProp prop;
            prop.host = this->params_->internalIDServerIP.c_str();
            prop.controllerId = this->params_->internalIDServerDeviceID;
            prop.driveMotorIdVec = { 11, 12, 13, 14 };
            prop.steeringMotorIdVec = { 41, 42, 43, 44 };
            prop.verbose = false;
            if (this->idclient_ == nullptr)
            {
                this->idclient_ = new JimIDClient(prop);
                this->idclientF_ = this->idclient_->connect();
            }
            else
            {
                this->idclient_->close();
                delete this->idclient_;
                this->idclient_ = new JimIDClient(prop);
                this->idclientF_ = this->idclient_->connect();
            }
        }
    }

    /**
     * Timer callback function for publisher.
     */
    void _publishCbFunc()
    {
        std::lock_guard<std::mutex> locker(this->publishMsgLock_);
        this->publisher_->publish(this->publishMsg_);
    }

    /**
     * Timer callback function for controller switch.
     */
    void _controllerSwitchCbFunc()
    {
        std::unique_lock<std::mutex> controllerLocker(this->controllerLock_, std::defer_lock);

        // Check joystick and valid selected controller.
        if (!this->wirelessF_ || this->wirelessBrakeF_ || !this->_checkSelectedController())
        {
            this->_brkSignal();
            RCLCPP_WARN(this->get_logger(), "[ControlServer::_controllerSwitchCbFunc] Wireless brake or controller index error.");
            return;
        }

        // Get output signal from controller map.
        vehicle_interfaces::msg::ControlChassis msg;
        std::chrono::high_resolution_clock::time_point timestamp;
        std::string controllerServiceName;
        controllerLocker.lock();
        controllerServiceName = this->controllerIdx_[this->selectedControllerIdx_];
        bool isSignalValidF = this->controllerMap_[controllerServiceName]->getSignal(msg, timestamp);
        vehicle_interfaces::msg::ControllerInfo info = this->controllerMap_[controllerServiceName]->getInfo();
        controllerLocker.unlock();


        /**
         * Check chassis signal and safety.
         */

        // Check chassis signal timeout.
        if (!isSignalValidF || std::chrono::high_resolution_clock::now() - timestamp > std::chrono::duration<float, std::milli>(info.period_ms * 2))
        {
            this->_brkSignal();
            RCLCPP_WARN(this->get_logger(), "[ControlServer::_controllerSwitchCbFunc] Invalid chassis signal.");
            return;
        }

        // Check chassis signal size.
        if (msg.drive_motor.size() != this->cInfo_.vehicle_type || msg.steering_motor.size() != this->cInfo_.vehicle_type)
        {
            this->_brkSignal();
            RCLCPP_WARN(this->get_logger(), "[ControlServer::_controllerSwitchCbFunc] Chassis signal size error.");
            return;
        }

        // Safety (8-direction emergency detection).
        if (!this->safetyF_)
        {
            this->_brkSignal();
            RCLCPP_WARN(this->get_logger(), "[ControlServer::_controllerSwitchCbFunc] Get emergency error.");
            return;
        }

        std::array<float, 8> emPArr = this->_safeCall(&this->emPs_, this->emPsLock_);
        float driveSum = 0;
        for (const auto& i : msg.drive_motor)
            driveSum += i;

        if (emPArr[0] > 0.7 && driveSum > 0)// Forward emergency.
        {
            this->_brkSignal();
            return;
        }
        else if (emPArr[1] > 0.7 && driveSum < 0)// Backward emergency.
        {
            this->_brkSignal();
            return;
        }


        /**
         * Chassis signal and safety check passed. 
         */

        // Motor direction correction.
        for (int i = 0; i < this->cInfo_.vehicle_type; i++)
        {
            msg.drive_motor[i] *= this->cInfo_.drive_motor_correction_vec[i];
            // TODO: steering motor angle to distance prediction.
            msg.steering_motor[i] = (5 + msg.steering_motor[i] * this->cInfo_.steering_motor_correction_vec[i] * 0.25) * 1000;
        }

        // Send chassis signals.
        if (this->idclientF_)
            this->idclientF_ = this->idclient_->sendDriveMotorSignal(msg.drive_motor, msg.parking_signal) &&
                                this->idclient_->sendSteeringMotorSignal(msg.steering_motor);

        // Filled publish message.
        std::unique_lock<std::mutex> publishMsgLocker(this->publishMsgLock_, std::defer_lock);
        publishMsgLocker.lock();
        this->publishMsg_.header.priority = vehicle_interfaces::msg::Header::PRIORITY_CONTROL;
        this->publishMsg_.header.device_type = vehicle_interfaces::msg::Header::DEVTYPE_NONE;
        this->publishMsg_.header.device_id = this->params_->nodeName;
        this->publishMsg_.header.frame_id = this->publishFrameId_++;
        this->publishMsg_.header.stamp_type = this->getTimestampType();
        this->publishMsg_.header.stamp = this->getTimestamp();
        this->publishMsg_.header.stamp_offset = this->getCorrectDuration().nanoseconds();
        this->publishMsg_.header.ref_publish_time_ms = this->params_->publishInterval_ms;

        this->publishMsg_.unit_type = vehicle_interfaces::msg::Chassis::UNIT_PWM;
        this->publishMsg_.drive_motor = msg.drive_motor;
        this->publishMsg_.steering_motor = msg.steering_motor;
        this->publishMsg_.parking_signal = msg.parking_signal;
        this->publishMsg_.controller_name = controllerServiceName;
        publishMsgLocker.unlock();
    }



    /**
     * ================================================================
     * ROS2 service callback functions.
     * ================================================================
     */

    /** Callback function for Controller register. */
    void _regServerCallback(const std::shared_ptr<vehicle_interfaces::srv::ControllerInfoReg::Request> request, 
                            std::shared_ptr<vehicle_interfaces::srv::ControllerInfoReg::Response> response)
    {
        if (this->exitF_)
        {
            response->response = false;
            return;
        }

        if (this->_addController(request->request))
        {
            RCLCPP_INFO(this->get_logger(), "[ControlServer::_regServerCallback] Add %s controller.", request->request.service_name.c_str());
            response->response = true;
        }
        else
            response->response = false;
    }

    /** Callback function for request controller information. */
    void _reqServerCallback(const std::shared_ptr<vehicle_interfaces::srv::ControllerInfoReq::Request> request, 
                            std::shared_ptr<vehicle_interfaces::srv::ControllerInfoReq::Response> response)
    {
        if (this->exitF_)
        {
            response->response = false;
            return;
        }

        std::lock_guard<std::mutex> locker(this->controllerLock_);
        response->response = true;
        if (request->service_name == "all")
        {
            for (const auto& i : this->controllerIdx_)
                response->control_info_vec.push_back(this->controllerMap_[i]->getInfo());
        }
        else if (this->controllerMap_.find(request->service_name) != this->controllerMap_.end())
        {
            response->control_info_vec.push_back(this->controllerMap_[request->service_name]->getInfo());
        }
        else
            response->response = false;
    }

public:
    ControlServer(const std::shared_ptr<Params>& params) : 
        vehicle_interfaces::VehicleServiceNode(params), 
        rclcpp::Node(params->nodeName), 
        params_(params), 
        // Chassis info init.
        cInfoF_(false), 
        // Wireless controller init.
        joystick_(nullptr), 
        jInfoF_(false), 
        joystickTm_(nullptr), 
        wirelessBrakeF_(false), 
        safetyOverControlF_(false), 
        wirelessF_(false), 
        // Controller switch init.
        selectedControllerIdx_(-1), 
        selectedControllerServiceName_(""), 
        controllerSwitchTm_(nullptr), 
        // Safety check init.
        safetyF_(false), 
        safetyTm_(nullptr), 
        // Socket of IDClient init.
        idclient_(nullptr), 
        idclientF_(false), 
        idclientTm_(nullptr), 
        // Publisher timer init.
        publishFrameId_(0), 
        publishTm_(nullptr), 
        // Node.
        exitF_(false), 
        // Chassis.
        driveMotorPredF_(false), 
        steeringMotorPredF_(false), 
        brakeMotorPredF_(false)
    {
        try
        {
            // Check ChassisInfo.
            if (ReadChassisInfo(params->chassisFilePath, this->cInfo_))
            {
                PrintChassisInfo(this->cInfo_);
                this->_initMotorMapping(this->cInfo_);
                this->_initChassisInfo(this->cInfo_);
                this->cInfoF_ = true;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "[ControlServer] Failed to read chassis file: %s", params->chassisFilePath.c_str());
                return;
            }

            // Check JoystickInfo.
            if (ReadJoystickInfo(params->joystickFilePath, this->jInfo_))
            {
                this->jInfoF_ = true;
                this->joystickTm_ = new vehicle_interfaces::Timer(1000, std::bind(&ControlServer::_joystickCbFunc, this));
                this->joystickTm_->start();
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "[ControlServer] Failed to read joystick file: %s", params->joystickFilePath.c_str());
                return;
            }

            // Create JimIDClient.
            this->idclientTm_ = new vehicle_interfaces::Timer(1000, std::bind(&ControlServer::_idclientCbFunc, this));
            this->idclientTm_->start();

            // Init safety.
            this->emPs_.fill(1);
            this->safetyTm_ = new vehicle_interfaces::Timer(params->publishInterval_ms, std::bind(&ControlServer::_safetyCbFunc, this));
            this->safetyTm_->start();

            // Controller switch.
            this->controllerSwitchTm_ = new vehicle_interfaces::Timer(params->outputPeriod_ms, std::bind(&ControlServer::_controllerSwitchCbFunc, this));
            this->controllerSwitchTm_->start();

            // Create publisher.
            this->publisher_ = this->create_publisher<vehicle_interfaces::msg::Chassis>(params->topicName, 10);
            this->publishTm_ = new vehicle_interfaces::Timer(params->publishInterval_ms, std::bind(&ControlServer::_publishCbFunc, this));
            this->publishTm_->start();

            // Create register and request services.
            this->regServer_ = this->create_service<vehicle_interfaces::srv::ControllerInfoReg>(params->serviceName + "_Reg", 
                std::bind(&ControlServer::_regServerCallback, this, std::placeholders::_1, std::placeholders::_2));

            this->reqServer_ = this->create_service<vehicle_interfaces::srv::ControllerInfoReq>(params->serviceName + "_Req", 
                std::bind(&ControlServer::_reqServerCallback, this, std::placeholders::_1, std::placeholders::_2));
        }
        catch (const std::string& e)
        {
            RCLCPP_ERROR(this->get_logger(), "[ControlServer] Caught error: %s", e.c_str());
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "[ControlServer] Caught error: %s", e.what());
        }
        catch (...)
        {
            RCLCPP_ERROR(this->get_logger(), "[ControlServer] Caught unexpected error.");
        }

        RCLCPP_INFO(this->get_logger(), "[ControlServer] Constructed.");
    }

    ~ControlServer()
    {
        this->close();
    }

    void close()
    {
        if (this->exitF_)// Ignore process if called repeatedly.
            return;
        this->exitF_ = true;// All looping process will be braked if exitF_ set to true.

        // Destroy publisher timer.
        if (this->publishTm_ != nullptr)
        {
            this->publishTm_->destroy();
            delete this->publishTm_;
        }
        // Join and delete controller switch thread.
        if (this->controllerSwitchTm_ != nullptr)
        {
            this->controllerSwitchTm_->destroy();
            delete this->controllerSwitchTm_;
        }
        // Join and delete safety thread.
        if (this->safetyTm_ != nullptr)
        {
            this->safetyTm_->destroy();
            delete this->safetyTm_;
        }
        // Destroy idclient timer.
        if (this->idclientTm_ != nullptr)
        {
            this->idclientTm_->destroy();
            delete this->idclientTm_;
        }
        // Delete idclient.
        if (this->idclient_ != nullptr)
        {
            this->idclient_->close();
            delete this->idclient_;
        }
        // Destroy joystick timer.
        if (this->joystickTm_ != nullptr)
        {
            this->joystickTm_->destroy();
            delete this->joystickTm_;
        }
        // Delete joystick.
        if (this->joystick_ != nullptr)
        {
            fclose(this->joystick_);
        }
        // Get all controller name and deleted by calling _removeController().
        auto tmp = this->_safeCall(&this->controllerIdx_, this->controllerLock_);
        for (const auto& serviceName : tmp)
            this->_removeController(serviceName);
    }
};
