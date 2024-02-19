#pragma once
#include <fstream>
#include "nlohmann/json.hpp"

#include "vehicle_interfaces/msg_json.h"
#include "vehicle_interfaces/msg/chassis_info.hpp"


/**
 * ChassisInfo functions.
 */
bool ReadChassisInfo(std::string filePath, vehicle_interfaces::msg::ChassisInfo& outInfo);
bool CvtJSONToChassisInfo(const nlohmann::json& json, vehicle_interfaces::msg::ChassisInfo& outInfo);
bool WriteChassisInfo(std::string savePath, const vehicle_interfaces::msg::ChassisInfo& info);
bool CvtChassisInfoToJSON(const vehicle_interfaces::msg::ChassisInfo& info, nlohmann::json& outJSON);
void PrintChassisInfo(const vehicle_interfaces::msg::ChassisInfo& info);


bool ReadChassisInfo(std::string filePath, vehicle_interfaces::msg::ChassisInfo& outInfo)
{
    nlohmann::json json;
    json.update(nlohmann::json::parse(std::ifstream(filePath)));
    return CvtJSONToChassisInfo(json, outInfo);
}

bool CvtJSONToChassisInfo(const nlohmann::json& json, vehicle_interfaces::msg::ChassisInfo& outInfo)
{
    try
    {
        std::vector<vehicle_interfaces::msg::Point2d> point2dTmp;
        std::vector<vehicle_interfaces::msg::MappingData> mappingDataTmp;
        std::vector<double> doubleTmp;
        std::vector<vehicle_interfaces::msg::MotorValueRange> motorValueRangeTmp;

        outInfo.vehicle_type = json["vehicle_type"];

        for (const auto& i : json["wheel_position"].items())
            point2dTmp.push_back(vehicle_interfaces::json_to_msg::Point2d::dump(i.value()));
        outInfo.wheel_position = point2dTmp;
        point2dTmp.clear();

        outInfo.chassis_centroid = vehicle_interfaces::json_to_msg::Point2d::dump(json["chassis_centroid"]);
        outInfo.chassis_cg = vehicle_interfaces::json_to_msg::Point2d::dump(json["chassis_cg"]);

        for (const auto& i : json["drive_motor_mapping_vec"].items())
            mappingDataTmp.push_back(vehicle_interfaces::json_to_msg::MappingData::dump(i.value()));
        outInfo.drive_motor_mapping_vec = mappingDataTmp;
        mappingDataTmp.clear();

        for (const auto& i : json["steering_motor_mapping_vec"].items())
            mappingDataTmp.push_back(vehicle_interfaces::json_to_msg::MappingData::dump(i.value()));
        outInfo.steering_motor_mapping_vec = mappingDataTmp;
        mappingDataTmp.clear();

        for (const auto& i : json["brake_motor_mapping_vec"].items())
            mappingDataTmp.push_back(vehicle_interfaces::json_to_msg::MappingData::dump(i.value()));
        outInfo.brake_motor_mapping_vec = mappingDataTmp;
        mappingDataTmp.clear();

        for (const auto& i : json["drive_motor_correction_vec"].items())
            doubleTmp.push_back(i.value());
        outInfo.drive_motor_correction_vec = doubleTmp;
        doubleTmp.clear();

        for (const auto& i : json["steering_motor_correction_vec"].items())
            doubleTmp.push_back(i.value());
        outInfo.steering_motor_correction_vec = doubleTmp;
        doubleTmp.clear();

        for (const auto& i : json["brake_motor_correction_vec"].items())
            doubleTmp.push_back(i.value());
        outInfo.brake_motor_correction_vec = doubleTmp;
        doubleTmp.clear();

        outInfo.drive_motor_pwm_value = vehicle_interfaces::json_to_msg::MotorValueRange::dump(json["drive_motor_pwm_value"]);
        outInfo.drive_motor_rpm_value = vehicle_interfaces::json_to_msg::MotorValueRange::dump(json["drive_motor_rpm_value"]);
        outInfo.steering_motor_pwm_value = vehicle_interfaces::json_to_msg::MotorValueRange::dump(json["steering_motor_pwm_value"]);
        outInfo.steering_motor_angle_value = vehicle_interfaces::json_to_msg::MotorValueRange::dump(json["steering_motor_angle_value"]);
        outInfo.brake_motor_pwm_value = vehicle_interfaces::json_to_msg::MotorValueRange::dump(json["brake_motor_pwm_value"]);
        outInfo.brake_motor_psi_value = vehicle_interfaces::json_to_msg::MotorValueRange::dump(json["brake_motor_psi_value"]);
    }
    catch(const std::exception& e)
    {
        std::cerr << "[CvtJSONToChassisInfo] Caught exception: " << e.what() << '\n';
        return false;
    }
    catch(...)
    {
        std::cerr << "[CvtJSONToChassisInfo] Caught unknown exception.";
        return false;
    }
    return true;
}

bool WriteChassisInfo(std::string savePath, const vehicle_interfaces::msg::ChassisInfo& info)
{
    nlohmann::json json;
    if (CvtChassisInfoToJSON(info, json))
    {
        std::ofstream outFile(savePath);
        outFile << json << std::endl;
        return true;
    }
    return false;
}

bool CvtChassisInfoToJSON(const vehicle_interfaces::msg::ChassisInfo& info, nlohmann::json& outJSON)
{
    try
    {
        std::vector<nlohmann::json> tmp;
        outJSON["vehicle_type"] = info.vehicle_type;

        for (const auto&i : info.wheel_position)
            tmp.push_back(vehicle_interfaces::msg_to_json::Point2d::dump(i));
        outJSON["wheel_position"] = tmp;
        tmp.clear();

        outJSON["chassis_centroid"] = vehicle_interfaces::msg_to_json::Point2d::dump(info.chassis_centroid);
        outJSON["chassis_cg"] = vehicle_interfaces::msg_to_json::Point2d::dump(info.chassis_cg);

        for (const auto&i : info.drive_motor_mapping_vec)
            tmp.push_back(vehicle_interfaces::msg_to_json::MappingData::dump(i));
        outJSON["drive_motor_mapping_vec"] = tmp;
        tmp.clear();

        for (const auto&i : info.steering_motor_mapping_vec)
            tmp.push_back(vehicle_interfaces::msg_to_json::MappingData::dump(i));
        outJSON["steering_motor_mapping_vec"] = tmp;
        tmp.clear();

        for (const auto&i : info.brake_motor_mapping_vec)
            tmp.push_back(vehicle_interfaces::msg_to_json::MappingData::dump(i));
        outJSON["brake_motor_mapping_vec"] = tmp;
        tmp.clear();

        outJSON["drive_motor_correction_vec"] = info.drive_motor_correction_vec;
        outJSON["steering_motor_correction_vec"] = info.steering_motor_correction_vec;
        outJSON["brake_motor_correction_vec"] = info.brake_motor_correction_vec;
        outJSON["drive_motor_pwm_value"] = vehicle_interfaces::msg_to_json::MotorValueRange::dump(info.drive_motor_pwm_value);
        outJSON["drive_motor_rpm_value"] = vehicle_interfaces::msg_to_json::MotorValueRange::dump(info.drive_motor_rpm_value);
        outJSON["steering_motor_pwm_value"] = vehicle_interfaces::msg_to_json::MotorValueRange::dump(info.steering_motor_pwm_value);
        outJSON["steering_motor_angle_value"] = vehicle_interfaces::msg_to_json::MotorValueRange::dump(info.steering_motor_angle_value);
        outJSON["brake_motor_pwm_value"] = vehicle_interfaces::msg_to_json::MotorValueRange::dump(info.brake_motor_pwm_value);
        outJSON["brake_motor_psi_value"] = vehicle_interfaces::msg_to_json::MotorValueRange::dump(info.brake_motor_psi_value);
    }
    catch(const std::exception& e)
    {
        std::cerr << "[CvtChassisInfoToJSON] Caught exception: " << e.what() << '\n';
        return false;
    }
    catch(...)
    {
        std::cerr << "[CvtChassisInfoToJSON] Caught unknown exception.";
        return false;
    }
    return true;
}

void PrintChassisInfo(const vehicle_interfaces::msg::ChassisInfo& info)
{
    printf("%-30s: %d\n", "vehicle_type", info.vehicle_type);
    for (const auto& i : info.wheel_position)
        printf("%-30s: (%5.5f, %5.5f)\n", "wheel_position", i.x, i.y);
    printf("%-30s: (%5.5f, %5.5f)\n", "chassis_centroid", info.chassis_centroid.x, info.chassis_centroid.y);
    printf("%-30s: (%5.5f, %5.5f)\n", "chassis_cg", info.chassis_cg.x, info.chassis_cg.y);
    printf("%-30s: size: %ld\n", "drive_motor_mapping_vec", info.drive_motor_mapping_vec.size());
    for (const auto& i : info.drive_motor_mapping_vec)
    {
        size_t sz = i.input_vec.size() < i.output_vec.size() ? i.input_vec.size() : i.output_vec.size();
        printf("%-30s: size: (%ld, %ld)\n", "drive_motor_mapping_vec", i.input_vec.size(), i.output_vec.size());
        for (int j = 0; j < sz; j++)
            printf("%-30s: (%5.5f, %5.5f)\n", "drive_motor_mapping_vec", i.input_vec[j], i.output_vec[j]);
    }
    printf("%-30s: size: %ld\n", "steering_motor_mapping_vec", info.steering_motor_mapping_vec.size());
    for (const auto& i : info.steering_motor_mapping_vec)
    {
        size_t sz = i.input_vec.size() < i.output_vec.size() ? i.input_vec.size() : i.output_vec.size();
        printf("%-30s: size: (%ld, %ld)\n", "steering_motor_mapping_vec", i.input_vec.size(), i.output_vec.size());
        for (int j = 0; j < sz; j++)
            printf("%-30s: (%5.5f, %5.5f)\n", "steering_motor_mapping_vec", i.input_vec[j], i.output_vec[j]);
    }
    printf("%-30s: size: %ld\n", "brake_motor_mapping_vec", info.brake_motor_mapping_vec.size());
    for (const auto& i : info.brake_motor_mapping_vec)
    {
        size_t sz = i.input_vec.size() < i.output_vec.size() ? i.input_vec.size() : i.output_vec.size();
        printf("%-30s: size: (%ld, %ld)\n", "brake_motor_mapping_vec", i.input_vec.size(), i.output_vec.size());
        for (int j = 0; j < sz; j++)
            printf("%-30s: (%5.5f, %5.5f)\n", "brake_motor_mapping_vec", i.input_vec[j], i.output_vec[j]);
    }
    printf("%-30s: size: %ld\n", "drive_motor_correction_vec", info.drive_motor_correction_vec.size());
    for (const auto& i : info.drive_motor_correction_vec)
        printf("%-30s: %5.5f\n", "drive_motor_correction_vec", i);
    printf("%-30s: size: %ld\n", "steering_motor_correction_vec", info.steering_motor_correction_vec.size());
    for (const auto& i : info.steering_motor_correction_vec)
        printf("%-30s: %5.5f\n", "steering_motor_correction_vec", i);
    printf("%-30s: size: %ld\n", "brake_motor_correction_vec", info.brake_motor_correction_vec.size());
    for (const auto& i : info.brake_motor_correction_vec)
        printf("%-30s: %5.5f\n", "brake_motor_correction_vec", i);
    printf("%-30s: min: %5.5f max: %5.5f init: %5.5f\n", "drive_motor_pwm_value", info.drive_motor_pwm_value.min, info.drive_motor_pwm_value.max, info.drive_motor_pwm_value.init);
    printf("%-30s: min: %5.5f max: %5.5f init: %5.5f\n", "drive_motor_rpm_value", info.drive_motor_rpm_value.min, info.drive_motor_rpm_value.max, info.drive_motor_rpm_value.init);
    printf("%-30s: min: %5.5f max: %5.5f init: %5.5f\n", "steering_motor_pwm_value", info.steering_motor_pwm_value.min, info.steering_motor_pwm_value.max, info.steering_motor_pwm_value.init);
    printf("%-30s: min: %5.5f max: %5.5f init: %5.5f\n", "steering_motor_angle_value", info.steering_motor_angle_value.min, info.steering_motor_angle_value.max, info.steering_motor_angle_value.init);
    printf("%-30s: min: %5.5f max: %5.5f init: %5.5f\n", "brake_motor_pwm_value", info.brake_motor_pwm_value.min, info.brake_motor_pwm_value.max, info.brake_motor_pwm_value.init);
    printf("%-30s: min: %5.5f max: %5.5f init: %5.5f\n", "brake_motor_psi_value", info.brake_motor_psi_value.min, info.brake_motor_psi_value.max, info.brake_motor_psi_value.init);
}
