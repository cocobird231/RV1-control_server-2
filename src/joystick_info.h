#pragma once
#include <linux/joystick.h>

#include <string>
#include <fstream>
#include "nlohmann/json.hpp"

#include "vehicle_interfaces/msg_json.h"

struct JoystickInfo;
bool js_event_equal(js_event event1, js_event event2);
bool ReadJoystickInfo(std::string filePath, JoystickInfo& outInfo);
bool CvtJSONToJoystickInfo(const nlohmann::json& json, JoystickInfo& outInfo);


// js_event struct defined under <linux/joystick.h>
// struct js_event {
// 	__u32 time;	/* event timestamp in milliseconds */
// 	__s16 value;	/* value */
// 	__u8 type;	/* event type */
// 	__u8 number;	/* axis/button number */
// };

bool js_event_equal(js_event event1, js_event event2)
{
    return event1.value == event2.value && 
            event1.type == event2.type && 
            event1.number == event2.number;
}

struct JoystickInfo
{
    std::string device;// Device path.
    js_event brakeBtn;// Wireless brake.
    js_event releaseBrakeBtn;// Release wireless brake.
    js_event enableSafetyBtn;// Enable safety over control.
    js_event disableSafetyBtn;// Disable safety over control.
    js_event initBtn;// Unused.
    js_event ascentIdxBtn;// Increase selected index number.
    js_event descentIdxBtn;// Decrease selected index number.
};

/**
 * JoystickInfo functions.
 */
bool ReadJoystickInfo(std::string filePath, JoystickInfo& outInfo)
{
    nlohmann::json json;
    json.update(nlohmann::json::parse(std::ifstream(filePath)));
    return CvtJSONToJoystickInfo(json, outInfo);
}

bool CvtJSONToJoystickInfo(const nlohmann::json& json, JoystickInfo& outInfo)
{
    try
    {
        outInfo.device = json["device"];
        outInfo.brakeBtn = { 0, json["brakeBtn"]["value"], json["brakeBtn"]["type"], json["brakeBtn"]["number"] };
        outInfo.releaseBrakeBtn = { 0, json["releaseBrakeBtn"]["value"], json["releaseBrakeBtn"]["type"], json["releaseBrakeBtn"]["number"] };
        outInfo.enableSafetyBtn = { 0, json["enableSafetyBtn"]["value"], json["enableSafetyBtn"]["type"], json["enableSafetyBtn"]["number"] };
        outInfo.disableSafetyBtn = { 0, json["disableSafetyBtn"]["value"], json["disableSafetyBtn"]["type"], json["disableSafetyBtn"]["number"] };
        outInfo.initBtn = { 0, json["initBtn"]["value"], json["initBtn"]["type"], json["initBtn"]["number"] };
        outInfo.ascentIdxBtn = { 0, json["ascentIdxBtn"]["value"], json["ascentIdxBtn"]["type"], json["ascentIdxBtn"]["number"] };
        outInfo.descentIdxBtn = { 0, json["descentIdxBtn"]["value"], json["descentIdxBtn"]["type"], json["descentIdxBtn"]["number"] };
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