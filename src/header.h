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
#include "vehicle_interfaces/msg_json.h"
#include "vehicle_interfaces/msg/chassis_info.hpp"
#include "vehicle_interfaces/msg/control_chassis.hpp"
#include "vehicle_interfaces/msg/controller_info.hpp"
#include "vehicle_interfaces/msg/control_server.hpp"
#include "vehicle_interfaces/msg/control_steering_wheel.hpp"

#include "vehicle_interfaces/srv/control_chassis_reg.hpp"
#include "vehicle_interfaces/srv/control_chassis_req.hpp"
#include "vehicle_interfaces/srv/controller_info_reg.hpp"
#include "vehicle_interfaces/srv/controller_info_req.hpp"
#include "vehicle_interfaces/srv/control_server.hpp"
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
    double publishInterval_ms = 50.0;

    bool enableOutput = false;
    double outputPeriod_ms = 80;
    double safetyCheckPeriod_ms = 50.0;
    double idclientCheckPeriod_ms = 1000.0;

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
        this->get_parameter("safetyCheckPeriod_ms", this->safetyCheckPeriod_ms);
        this->get_parameter("idclientCheckPeriod_ms", this->idclientCheckPeriod_ms);
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
        this->declare_parameter<double>("safetyCheckPeriod_ms", this->safetyCheckPeriod_ms);
        this->declare_parameter<double>("idclientCheckPeriod_ms", this->idclientCheckPeriod_ms);

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
 * Feature:
 * [x] Control reg server
 * [x] Control req server
 * [x] Add controller
 * [x] Init chassis info (e.g. pred model)
 * [x] cvtSignal (rewrite from py_controlserver/genMotorPWM.py)
 * [x] joystick controller
 * [x] Read chassis.json
 * [x] Read joystick.json
 * [x] Connect jim id server
 * [x] Add safety thread
 * [x] Controller switch (consider timestamp)
 * [x] Publisher thread for chassis signal
 * [x] Higher priority interruption
 * [x] Dynamic controller removal
 * 
 * Functionality test:
 * [x] Test control reg server
 * [x] Test control req server
 * [x] Test joystick controller
 * [x] Test idclient
 * [x] Test safety
 * [x] Test controller switch
 * [x] Test publisher
 * [/] Test dynamic controller addition and removal
 * [ ] Test higher priority interruption
 */
class ControlServer : public vehicle_interfaces::VehicleServiceNode
{
private:
    const std::shared_ptr<Params> params_;// ControlServer parameters.
    vehicle_interfaces::msg::ChassisInfo cInfo_;// Chassis architecture information.
    std::atomic<bool> cInfoF_;// Check valid cInfo_.
    // TODO: Add lock for cInfo_.
    // TODO: cInfo_ should be modifiable by service.

    // Joystick controller.
    JoystickInfo jInfo_;// joystick button function configuration.
    std::atomic<bool> jInfoF_;// Check valid jInfo_.
    FILE *joystick_;// Joystick device.
    std::atomic<bool> joystickF_;// Is joystick controller detected.
    std::thread* joystickTh_;// Call _joystickTh().
    std::atomic<bool> joystickBrakeF_;// State of joystick brake.
    std::atomic<bool> safetyOverControlF_;// State of safety over control.

    // Controller storage.
    std::map<std::string, std::shared_ptr<vehicle_interfaces::BaseControllerClient> > controllerMap_;// Store registered controller.
    std::map<std::string, rclcpp::executors::SingleThreadedExecutor*> controllerExecMap_;
    std::map<std::string, std::thread*> controllerThMap_;
    std::deque<std::string> controllerIdx_;// The vector of controller service name.
    std::mutex controllerLock_;// Lock controllerMap_, controllerExecMap_, controllerThMap_ and controllerIdx_.

    // Controller switch.
    std::atomic<size_t> selectedControllerIdx_;// The index of selected controller in controllerIdx_.
    std::string selectedControllerServiceName_;// The service name of selected controller.
    vehicle_interfaces::msg::ControllerInfo selectedControllerInfo_;// The information of selected controller.
    vehicle_interfaces::Timer* controllerSwitchTm_;// Call _controllerSwitchCbFunc().
    std::atomic<double> controllerSwitchTmPeriod_ms_;// The period of controller switch.
    std::atomic<bool> controllerSwitchTmF_;// Enable/Disable controller switch output.

    // Safety check.
    std::array<float, 8> emPs_;// Store 8-direction emergency percentages.
    std::mutex emPsLock_;// Lock emPs_.
    std::atomic<bool> safetyF_;// Check valid emPs_.
    vehicle_interfaces::Timer* safetyTm_;// Call _safetyCbFunc().
    std::atomic<double> safetyTmPeriod_ms_;// The period of safety check.
    std::atomic<bool> safetyTmF_;// Enable/Disable safety check.

    // JimIDClient socket.
    JimIDClient *idclient_;
    std::atomic<bool> idclientF_;// Check valid idclient_.
    vehicle_interfaces::Timer* idclientTm_;// Call _idclientCbFunc().
    std::atomic<double> idclientTmPeriod_ms_;// The period of idclient check.
    std::atomic<bool> idclientTmF_;// Enable/Disable idclient check.

    // Publisher.
    rclcpp::Publisher<vehicle_interfaces::msg::Chassis>::SharedPtr publisher_;// Publisher for chassis signal.
    vehicle_interfaces::msg::Chassis publishMsg_;// The output signal of controller switch.
    u_int64_t publishFrameId_;// Frame count for publish message.
    std::mutex publishMsgLock_;// Lock publishMsg_.
    vehicle_interfaces::Timer* publishTm_;// Call _publishCbFunc().
    std::atomic<double> publishTmPeriod_ms_;// The period of publish message.
    std::atomic<bool> publishTmF_;// Enable/Disable publish message.

    // ControllerInfoReg and ControllerInfoReq server.
    rclcpp::Service<vehicle_interfaces::srv::ControllerInfoReg>::SharedPtr controllerInfoRegServer_;// For Controller register.
    rclcpp::Service<vehicle_interfaces::srv::ControllerInfoReq>::SharedPtr controllerInfoReqServer_;// Get current Controller info.
    rclcpp::Service<vehicle_interfaces::srv::ControlServer>::SharedPtr statusServer_;// Set/Get current status.

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
     * Set header for published message.
     * @param[out] msg Header message.
     */
    void _getHeader(vehicle_interfaces::msg::Header& msg)
    {
        msg.priority = vehicle_interfaces::msg::Header::PRIORITY_CONTROL;
        msg.device_type = vehicle_interfaces::msg::Header::DEVTYPE_NONE;
        msg.device_id = this->params_->nodeName;
        msg.frame_id = this->publishFrameId_++;
        msg.stamp_type = this->getTimestampType();
        msg.stamp = this->getTimestamp();
        msg.stamp_offset = this->getCorrectDuration().nanoseconds();
        msg.ref_publish_time_ms = this->params_->publishInterval_ms;
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
    std::vector<double> _driveMotorRPMToPWM(const std::vector<double>& rpms)
    {
        if (!this->driveMotorPredF_)
            return std::vector<double>(this->cInfo_.vehicle_type, 0);

        if (this->driveMotorPred_.size() != rpms.size())
            return std::vector<double>(this->cInfo_.vehicle_type, 0);

        std::vector<double> ret(rpms.size(), 0);

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
        double refSpeed = vehicle_interfaces::LinearMapping1d(src.pedal_throttle, 0, 255, 0, 120);
        dst.drive_motor = { 0, 0, 0, 0 };
        dst.steering_motor = { 0, 0, 0, 0 };
        dst.parking_signal = { 0, 0, 0, 0 };
        std::vector<double> motorDirectionList(4, 1);

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

        double innerAng = 0;
        double outerAng = 0;

        double innerVelo = 0;
        double outerVelo = 0;

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
            double rearInnerVelo = refSpeed * innerRadius / steeringRadius;
            double rearOuterVelo = refSpeed * outerRadius / steeringRadius;
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
            double steeringAngle = vehicle_interfaces::LinearMapping1d(steeringWheelAbsCorrection, 0, 32768, this->CHASSIS_MIN_WHEEL_STEER_ANGLE, this->CHASSIS_MAX_WHEEL_STEER_ANGLE);
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
     * Set output controller by index.
     * The selectedControllerIdx_ and selectedControllerServiceName_ will be modified to -1 and "" respectively if index is invalid. 
     * Otherwise, the selectedControllerIdx_ and selectedControllerServiceName_ will be modified to index and related service name respectively.
     * @param[in] idx the index of controller desired to be selected.
     * @return true if index is valid. Otherwise, return false.
     * @note This function is using controllerLock_.
     */
    bool _setOutputController(size_t idx)
    {
        std::lock_guard<std::mutex> locker(this->controllerLock_);
        if (idx >= 0 && idx < this->controllerIdx_.size())
        {
            this->selectedControllerIdx_ = idx;
            this->selectedControllerServiceName_ = this->controllerIdx_[idx];
            this->selectedControllerInfo_ = this->controllerMap_[this->selectedControllerServiceName_]->getInfo();
            return true;
        }
        this->selectedControllerIdx_ = -1;
        this->selectedControllerServiceName_ = "";
        this->selectedControllerInfo_ = vehicle_interfaces::msg::ControllerInfo();
        return false;
    }

    /**
     * Set output controller by controller service name.
     * The selectedControllerIdx_ and selectedControllerServiceName_ will be modified to -1 and "" respectively if service name is invalid. 
     * Otherwise, the selectedControllerServiceName_ and selectedControllerIdx_ will be modified to service name and related index respectively.
     * @param[in] serviceName the service name of controller desired to be selected.
     * @return true if service name is valid. Otherwise, return false.
     * @note This function is using controllerLock_.
     */
    bool _setOutputController(const std::string& serviceName)
    {
        std::lock_guard<std::mutex> locker(this->controllerLock_);
        if (this->controllerMap_.find(serviceName) != this->controllerMap_.end())
        {
            this->selectedControllerServiceName_ = serviceName;
            this->selectedControllerInfo_ = this->controllerMap_[this->selectedControllerServiceName_]->getInfo();
            for (int i = 0; i < this->controllerIdx_.size(); i++)
                if (this->controllerIdx_[i] == serviceName)
                {
                    this->selectedControllerIdx_ = i;
                    return true;
                }
        }
        this->selectedControllerIdx_ = -1;
        this->selectedControllerServiceName_ = "";
        this->selectedControllerInfo_ = vehicle_interfaces::msg::ControllerInfo();
        return false;
    }

    /**
     * Check valid output controller.
     * @return true if index and service name of output controller is valid. Otherwise, return false.
     * @note This function is using controllerLock_.
     */
    bool _checkOutputController()
    {
        std::lock_guard<std::mutex> locker(this->controllerLock_);
        return this->selectedControllerIdx_ != -1 && 
                this->selectedControllerServiceName_ != "";
    }

    /**
     * Get ControllerInfo of output controller.
     * @param[out] outInfo describes the output controller information.
     * @return true if index and service name of output controller is valid. Otherwise, return false.
     * @note This function is using controllerLock_.
     */
    bool _getOutputControllerInfo(vehicle_interfaces::msg::ControllerInfo& outInfo)
    {
        if (this->_checkOutputController())
        {
            std::lock_guard<std::mutex> locker(this->controllerLock_);
            outInfo = this->controllerMap_[this->selectedControllerServiceName_]->getInfo();
            return true;
        }
        return false;
    }

    /**
     * Get index and service name of output controller.
     * @return a pair of index and service name of output controller.
     * @note This function is using controllerLock_.
     */
    std::pair<size_t, std::string> _getOutputControllerPos()
    {
        std::lock_guard<std::mutex> locker(this->controllerLock_);
        return { this->selectedControllerIdx_, this->selectedControllerServiceName_ };
    }

    /**
     * Add ControlServerController node to map and executor, then new thread to spin.
     * @param[in] info describes the controller information.
     * @return true if controller added. Otherwise, return false.
     * @note This function is using controllerLock_.
     */
    bool _addController(const vehicle_interfaces::msg::ControllerInfo& info)
    {
        std::lock_guard<std::mutex> locker(this->controllerLock_);
        try
        {
            if (this->controllerMap_.find(info.service_name) == this->controllerMap_.end())// service_name not in controllerMap_
            {
                if (info.msg_type == vehicle_interfaces::msg::ControllerInfo::MSG_TYPE_STEERING_WHEEL)
                {
                    // Add controller to controllerMap_.
                    auto controller = std::make_shared<vehicle_interfaces::SteeringWheelControllerClient>(info);
                    // Convert function is required if controller msg type is ControlSteeringWheel.
                    // TODO: more convert function for PWM, RPM and ANGLE.
                    controller->setCvtFunc(std::bind(&ControlServer::_cvtControlSteeringWheelToControlChassis, this, std::placeholders::_1, std::placeholders::_2));
                    controller->setInterruptFunc(std::bind(&ControlServer::_controllerInterruptCbFunc, this, std::placeholders::_1));
                    this->controllerMap_[info.service_name] = controller;
                }
                else if (info.msg_type == vehicle_interfaces::msg::ControllerInfo::MSG_TYPE_CHASSIS)
                {
                    // Add controller to controllerMap_.
                    auto controller = std::make_shared<vehicle_interfaces::ChassisControllerClient>(info);
                    controller->setInterruptFunc(std::bind(&ControlServer::_controllerInterruptCbFunc, this, std::placeholders::_1));
                    this->controllerMap_[info.service_name] = controller;
                }
                else
                    throw "Unknown msg_type.";

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
     * @param[in] serviceName the name of controller desired to be removed.
     * @return true if remove successfully. Otherwise, return false.
     * @note This function is using controllerLock_.
     */
    bool _removeController(const std::string& serviceName)
    {
        // Check current controller.
        bool currentIdxF = this->_checkOutputController();

        std::lock_guard<std::mutex> locker(this->controllerLock_);
        if (this->controllerMap_.find(serviceName) == this->controllerMap_.end())// serviceName not exist.
            return false;

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
            // delete this->controllerThMap_[serviceName];
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
            this->selectedControllerServiceName_ = this->controllerIdx_[this->selectedControllerIdx_];
        }
        else
        {
            this->selectedControllerIdx_ = -1;
            this->selectedControllerServiceName_ = "";
        }
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
            this->selectedControllerServiceName_ = this->controllerIdx_[this->selectedControllerIdx_];
        }
        else
        {
            this->selectedControllerIdx_ = -1;
            this->selectedControllerServiceName_ = "";
        }
    }

    /**
     * Controller interrupt event handler.
     * @param[in] info describes the controller information.
     */
    void _controllerInterruptCbFunc(const vehicle_interfaces::msg::ControllerInfo& info)
    {
        auto currentInfo = this->_safeCall(&this->selectedControllerInfo_, this->controllerLock_);// Retrieve current selected controller info.
        if (info.privilege < currentInfo.privilege)// Callback controller has higher privilege.
            this->_setOutputController(info.service_name);
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
     * (Sub-thread) Loop function for joystick signal input.
     * Function stops when exitF_ set to true.
     */
    void _joystickTh()
    {
        while (!this->exitF_)
        {
            // Open joystick.
            if (!this->joystickF_)
            {
                if (this->joystick_ != nullptr)
                    fclose(this->joystick_);
                this->joystick_ = fopen(this->jInfo_.device.c_str(), "rb+");
                if (!this->joystick_)
                {
                    this->joystickF_ = false;
                    RCLCPP_ERROR(this->get_logger(), "[ControlServer::_joystickTh] Open %s failed.\n", this->jInfo_.device.c_str());
                    std::this_thread::sleep_for(1s);
                    continue;
                }
                this->joystickF_ = true;
            }

            // Read joystick data.
            js_event msg;
            const size_t ret = fread(&msg, sizeof(js_event), 1, this->joystick_);
            if (ret == 1)
            {
                // printf("time: %-5d, value: %-5d, type: %-5d, number: %-5d\n", msg.time, msg.value, msg.type, msg.number);
                if (js_event_equal(msg, this->jInfo_.brakeBtn))
                {
                    this->joystickBrakeF_ = true;
                    RCLCPP_INFO(this->get_logger(), "[ControlServer::_joystickTh] Brake button pressed.");
                }
                else if (js_event_equal(msg, this->jInfo_.releaseBrakeBtn))
                {
                    this->joystickBrakeF_ = false;
                    RCLCPP_INFO(this->get_logger(), "[ControlServer::_joystickTh] Release brake button pressed.");
                }
                if (js_event_equal(msg, this->jInfo_.enableSafetyBtn))
                {
                    this->safetyOverControlF_ = true;
                    RCLCPP_INFO(this->get_logger(), "[ControlServer::_joystickTh] Enable safety button pressed.");
                }
                else if (js_event_equal(msg, this->jInfo_.disableSafetyBtn))
                {
                    this->safetyOverControlF_ = false;
                    RCLCPP_INFO(this->get_logger(), "[ControlServer::_joystickTh] Disable safety button pressed.");
                }
                else if (js_event_equal(msg, this->jInfo_.ascentIdxBtn))
                {
                    this->_increaseSelectedIdx();
                    RCLCPP_INFO(this->get_logger(), "[ControlServer::_joystickTh] Ascent index button pressed.");
                }
                else if (js_event_equal(msg, this->jInfo_.descentIdxBtn))
                {
                    this->_decreaseSelectedIdx();
                    RCLCPP_INFO(this->get_logger(), "[ControlServer::_joystickTh] Descent index button pressed.");
                }
            }
            else if (feof(this->joystick_))
            {
                this->joystickF_ = false;
                RCLCPP_ERROR(this->get_logger(), "[ControlServer::_joystickTh] EoF.");
            }
            else if (ferror(this->joystick_))
            {
                this->joystickF_ = false;
                RCLCPP_ERROR(this->get_logger(), "[ControlServer::_joystickTh] Error reading %s.", this->jInfo_.device.c_str());
            }
        }
    }

    /**
     * (Sub-thread) Timer callback function for safety check.
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
                return;
            }
            this->safetyF_ = false;
            RCLCPP_ERROR(this->get_logger(), "[ControlServer::_safetyCbFunc] Get emergency error.");
        }
        catch (...)
        {
            this->safetyF_ = false;
            RCLCPP_ERROR(this->get_logger(), "[ControlServer::_safetyCbFunc] Caught unexpected error.");
        }
    }

    /**
     * (Sub-thread) Timer callback function for idclient.
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
            try
            {
                if (this->idclient_ == nullptr)
                {
                    this->idclient_ = new JimIDClient(prop);
                    this->idclientF_ = this->idclient_->connect();
                }
                else
                {
                    delete this->idclient_;
                    this->idclient_ = new JimIDClient(prop);
                    this->idclientF_ = this->idclient_->connect();
                }
            }
            catch (...)
            {
                if (this->idclient_ != nullptr)
                    delete this->idclient_;
                this->idclient_ = nullptr;
                this->idclientF_ = false;
                RCLCPP_ERROR(this->get_logger(), "[ControlServer::_idclientCbFunc] Caught unexpected error.");
            }
            if (!this->idclientF_)
                RCLCPP_ERROR(this->get_logger(), "[ControlServer::_idclientCbFunc] ID client connection failed.");
        }
    }

    /**
     * (Sub-thread) Timer callback function for publisher.
     */
    void _publishCbFunc()
    {
        this->publisher_->publish(this->_safeCall(&this->publishMsg_, this->publishMsgLock_));
    }

    /**
     * (Sub-thread) Timer callback function for controller switch.
     */
    void _controllerSwitchCbFunc()
    {
        std::unique_lock<std::mutex> controllerLocker(this->controllerLock_, std::defer_lock);

        // Check joystick and valid selected controller.
        if (!this->joystickF_ || this->joystickBrakeF_ || !this->_checkOutputController())
        {
            this->_brkSignal();
            RCLCPP_WARN(this->get_logger(), "[ControlServer::_controllerSwitchCbFunc] Joystick brake or controller index error.");
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
        if (!isSignalValidF)
        {
            this->_brkSignal();
            RCLCPP_WARN(this->get_logger(), "[ControlServer::_controllerSwitchCbFunc] Invalid chassis signal.");
            return;
        }

        if (std::chrono::high_resolution_clock::now() - timestamp > std::chrono::duration<float, std::milli>(info.period_ms * 2))
        {
            this->_brkSignal();
            RCLCPP_WARN(this->get_logger(), "[ControlServer::_controllerSwitchCbFunc] Chassis signal timeout.");
            return;
        }

        // Check chassis signal size.
        if (msg.drive_motor.size() != this->cInfo_.vehicle_type || msg.steering_motor.size() != this->cInfo_.vehicle_type)
        {
            this->_brkSignal();
            RCLCPP_WARN(this->get_logger(), "[ControlServer::_controllerSwitchCbFunc] Chassis signal size error.");
            return;
        }

        // Safety service check.
        if (!this->safetyF_)
        {
            this->_brkSignal();
            RCLCPP_WARN(this->get_logger(), "[ControlServer::_controllerSwitchCbFunc] Safety service error.");
            return;
        }

        // Safety over control (8-direction emergency detection).
        if (this->safetyOverControlF_)
        {
            std::array<float, 8> emPArr = this->_safeCall(&this->emPs_, this->emPsLock_);
            float driveSum = 0;
            for (const auto& i : msg.drive_motor)
                driveSum += i;

            if (emPArr[0] > 0.7 && driveSum > 0)// Forward emergency.
            {
                this->_brkSignal();
                RCLCPP_WARN(this->get_logger(), "[ControlServer::_controllerSwitchCbFunc] (forward) Safety over control.");
                return;
            }
            else if (emPArr[1] > 0.7 && driveSum < 0)// Backward emergency.
            {
                this->_brkSignal();
                RCLCPP_WARN(this->get_logger(), "[ControlServer::_controllerSwitchCbFunc] (backward) Safety over control.");
                return;
            }
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
        auto cvt = vehicle_interfaces::msg_to_msg::Chassis::convert(msg);
        this->_getHeader(cvt.header);
        cvt.controller_name = controllerServiceName;
        this->_safeSave(&this->publishMsg_, cvt, this->publishMsgLock_);
    }



    /**
     * ================================================================
     * ROS2 service callback functions.
     * ================================================================
     */

    /** Callback function for Controller register. */
    void _controllerInfoRegServerCbFunc(const std::shared_ptr<vehicle_interfaces::srv::ControllerInfoReg::Request> request, 
                            std::shared_ptr<vehicle_interfaces::srv::ControllerInfoReg::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "[ControlServer::_controllerInfoRegServerCbFunc]");
        if (this->exitF_)
        {
            RCLCPP_INFO(this->get_logger(), "[ControlServer::_controllerInfoRegServerCbFunc] exit flag set.");
            response->response = false;
            return;
        }

        if (this->_addController(request->request))
        {
            RCLCPP_INFO(this->get_logger(), "[ControlServer::_controllerInfoRegServerCbFunc] Add %s controller.", request->request.service_name.c_str());
            response->response = true;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "[ControlServer::_controllerInfoRegServerCbFunc] Add %s controller failed.", request->request.service_name.c_str());
            response->response = false;
        }
    }

    /** Callback function for request controller information. */
    void _controllerInfoReqServerCbFunc(const std::shared_ptr<vehicle_interfaces::srv::ControllerInfoReq::Request> request, 
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

    /** Callback function for set/get Control Server information. */
    void _statusServerCbFunc(const std::shared_ptr<vehicle_interfaces::srv::ControlServer::Request> request, 
                            std::shared_ptr<vehicle_interfaces::srv::ControlServer::Response> response)
    {
        if (this->exitF_)
        {
            response->response = false;
            return;
        }
        response->response = true;
        if (request->request.controller_action == vehicle_interfaces::msg::ControlServer::CONTROLLER_ACTION_SELECT)
        {
            response->response &= this->_setOutputController(request->request.controller_service_name);
        }
        else if (request->request.controller_action == vehicle_interfaces::msg::ControlServer::CONTROLLER_ACTION_REMOVE)
        {
            response->response &= this->_removeController(request->request.controller_service_name);
        }
        if (request->request.server_action & vehicle_interfaces::msg::ControlServer::SERVER_ACTION_SET_PERIOD > 0)
        {
            if (request->request.server_output_period_ms > 0)
            {
                this->controllerSwitchTm_->setInterval(request->request.server_output_period_ms);
                this->controllerSwitchTmPeriod_ms_ = request->request.server_output_period_ms;
            }
            if (request->request.server_safety_period_ms > 0)
            {
                this->safetyTm_->setInterval(request->request.server_safety_period_ms);
                this->safetyTmPeriod_ms_ = request->request.server_safety_period_ms;
            }
            if (request->request.server_idclient_period_ms > 0)
            {
                this->idclientTm_->setInterval(request->request.server_idclient_period_ms);
                this->idclientTmPeriod_ms_ = request->request.server_idclient_period_ms;
            }
            if (request->request.server_publish_period_ms > 0)
            {
                this->publishTm_->setInterval(request->request.server_publish_period_ms);
                this->publishTmPeriod_ms_ = request->request.server_publish_period_ms;
            }
        }
        if (request->request.server_action & vehicle_interfaces::msg::ControlServer::SERVER_ACTION_SET_TIMER > 0)
        {
            if (request->request.server_output_timer_status == vehicle_interfaces::msg::ControlServer::TIMER_STATUS_START)
            {
                this->controllerSwitchTm_->start();
                this->controllerSwitchTmF_ = true;
            }
            else if (request->request.server_output_timer_status == vehicle_interfaces::msg::ControlServer::TIMER_STATUS_STOP)
            {
                this->controllerSwitchTm_->stop();
                this->controllerSwitchTmF_ = false;
            }
            if (request->request.server_safety_timer_status == vehicle_interfaces::msg::ControlServer::TIMER_STATUS_START)
            {
                this->safetyTm_->start();
                this->safetyTmF_ = true;
            }
            else if (request->request.server_safety_timer_status == vehicle_interfaces::msg::ControlServer::TIMER_STATUS_STOP)
            {
                this->safetyTm_->stop();
                this->safetyTmF_ = false;
            }
            if (request->request.server_idclient_timer_status == vehicle_interfaces::msg::ControlServer::TIMER_STATUS_START)
            {
                this->idclientTm_->start();
                this->idclientTmF_ = true;
            }
            else if (request->request.server_idclient_timer_status == vehicle_interfaces::msg::ControlServer::TIMER_STATUS_STOP)
            {
                this->idclientTm_->stop();
                this->idclientTmF_ = false;
            }
            if (request->request.server_publish_timer_status == vehicle_interfaces::msg::ControlServer::TIMER_STATUS_START)
            {
                this->publishTm_->start();
                this->publishTmF_ = true;
            }
            else if (request->request.server_publish_timer_status == vehicle_interfaces::msg::ControlServer::TIMER_STATUS_STOP)
            {
                this->publishTm_->stop();
                this->publishTmF_ = false;
            }
        }
        auto [conIdx, conName] = this->_getOutputControllerPos();
        vehicle_interfaces::msg::ControlServer res;
        res.controller_service_name = conName;
        res.server_output_timer_status = this->controllerSwitchTmF_ ? vehicle_interfaces::msg::ControlServer::TIMER_STATUS_START : vehicle_interfaces::msg::ControlServer::TIMER_STATUS_STOP;
        res.server_output_period_ms = this->controllerSwitchTmPeriod_ms_;
        res.server_safety_timer_status = this->safetyTmF_ ? vehicle_interfaces::msg::ControlServer::TIMER_STATUS_START : vehicle_interfaces::msg::ControlServer::TIMER_STATUS_STOP;
        res.server_safety_period_ms = this->safetyTmPeriod_ms_;
        res.server_idclient_timer_status = this->idclientTmF_ ? vehicle_interfaces::msg::ControlServer::TIMER_STATUS_START : vehicle_interfaces::msg::ControlServer::TIMER_STATUS_STOP;
        res.server_idclient_period_ms = this->idclientTmPeriod_ms_;
        res.server_publish_timer_status = this->publishTmF_ ? vehicle_interfaces::msg::ControlServer::TIMER_STATUS_START : vehicle_interfaces::msg::ControlServer::TIMER_STATUS_STOP;
        res.server_publish_period_ms = this->publishTmPeriod_ms_;
        res.chassis_info = this->cInfo_;
        response->status = res;
    }

public:
    ControlServer(const std::shared_ptr<Params>& params) : 
        vehicle_interfaces::VehicleServiceNode(params), 
        rclcpp::Node(params->nodeName), 
        params_(params), 
        // Chassis info init.
        cInfoF_(false), 
        // Joystick controller init.
        joystick_(nullptr), 
        jInfoF_(false), 
        joystickTh_(nullptr), 
        joystickBrakeF_(false), 
        safetyOverControlF_(false), 
        joystickF_(false), 
        // Controller switch init.
        selectedControllerIdx_(-1), 
        selectedControllerServiceName_(""), 
        controllerSwitchTm_(nullptr), 
        controllerSwitchTmPeriod_ms_(0), 
        controllerSwitchTmF_(false), 
        // Safety check init.
        safetyF_(false), 
        safetyTm_(nullptr), 
        safetyTmPeriod_ms_(0), 
        safetyTmF_(false), 
        // Socket of IDClient init.
        idclient_(nullptr), 
        idclientF_(false), 
        idclientTm_(nullptr), 
        idclientTmPeriod_ms_(0), 
        idclientTmF_(false), 
        // Publisher timer init.
        publishFrameId_(0), 
        publishTm_(nullptr), 
        publishTmPeriod_ms_(0), 
        publishTmF_(false), 
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
            RCLCPP_INFO(this->get_logger(), "[ControlServer] Loading chassis file: %s", params->chassisFilePath.c_str());
            if (ReadChassisInfo(params->chassisFilePath, this->cInfo_))
            {
                PrintChassisInfo(this->cInfo_);
                this->_initMotorMapping(this->cInfo_);
                this->_initChassisInfo(this->cInfo_);
                this->cInfoF_ = true;
                RCLCPP_INFO(this->get_logger(), "[ControlServer] Chassis initialized.");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "[ControlServer] Failed to read chassis file: %s", params->chassisFilePath.c_str());
                return;
            }

            // Check JoystickInfo.
            RCLCPP_INFO(this->get_logger(), "[ControlServer] Loading joystick file: %s", params->joystickFilePath.c_str());
            if (ReadJoystickInfo(params->joystickFilePath, this->jInfo_))
            {
                this->jInfoF_ = true;
                this->joystickTh_ = new std::thread(&ControlServer::_joystickTh, this);
                RCLCPP_INFO(this->get_logger(), "[ControlServer] Joystick initialized.");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "[ControlServer] Failed to read joystick file: %s", params->joystickFilePath.c_str());
                return;
            }

            // Create idclient timer.
            this->idclientTmPeriod_ms_ = params->idclientCheckPeriod_ms;
            RCLCPP_INFO(this->get_logger(), "[ControlServer] Initializing idclient timer...");
            this->idclientTm_ = new vehicle_interfaces::Timer(params->idclientCheckPeriod_ms, std::bind(&ControlServer::_idclientCbFunc, this));
            this->idclientTm_->start();

            // Create safety timer.
            this->safetyTmPeriod_ms_ = params->safetyCheckPeriod_ms;
            RCLCPP_INFO(this->get_logger(), "[ControlServer] Initializing safety timer...");
            this->emPs_.fill(1);
            this->safetyTm_ = new vehicle_interfaces::Timer(params->safetyCheckPeriod_ms, std::bind(&ControlServer::_safetyCbFunc, this));
            this->safetyTm_->start();

            // Create controller switch timer.
            this->controllerSwitchTmPeriod_ms_ = params->outputPeriod_ms;
            this->controllerSwitchTmF_ = params->enableOutput;
            RCLCPP_INFO(this->get_logger(), "[ControlServer] Initializing controller switch timer...");
            this->controllerSwitchTm_ = new vehicle_interfaces::Timer(params->outputPeriod_ms, std::bind(&ControlServer::_controllerSwitchCbFunc, this));
            if (params->enableOutput)
                this->controllerSwitchTm_->start();

            // Create publisher timer.
            this->publishTmPeriod_ms_ = params->publishInterval_ms;
            RCLCPP_INFO(this->get_logger(), "[ControlServer] Initializing publisher timer...");
            this->publisher_ = this->create_publisher<vehicle_interfaces::msg::Chassis>(params->topicName, 10);
            this->publishTm_ = new vehicle_interfaces::Timer(params->publishInterval_ms, std::bind(&ControlServer::_publishCbFunc, this));
            this->publishTm_->start();

            // Create register and request services.
            this->controllerInfoRegServer_ = this->create_service<vehicle_interfaces::srv::ControllerInfoReg>(params->serviceName + "_Reg", 
                std::bind(&ControlServer::_controllerInfoRegServerCbFunc, this, std::placeholders::_1, std::placeholders::_2));

            this->controllerInfoReqServer_ = this->create_service<vehicle_interfaces::srv::ControllerInfoReq>(params->serviceName + "_Req", 
                std::bind(&ControlServer::_controllerInfoReqServerCbFunc, this, std::placeholders::_1, std::placeholders::_2));

            this->statusServer_ = this->create_service<vehicle_interfaces::srv::ControlServer>(params->serviceName, 
                std::bind(&ControlServer::_statusServerCbFunc, this, std::placeholders::_1, std::placeholders::_2));
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
            delete this->idclient_;
        }
        // Destroy joystick timer.
        if (this->joystickTh_ != nullptr)
        {
            this->joystickTh_->join();
            delete this->joystickTh_;
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
