#ver=2.0
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import json
import yaml
from yaml import load, Loader

pkgName = 'cpp_controlserver'

def generate_launch_description():
    commonFilePath = os.path.join(get_package_share_directory(pkgName), 'launch/common.yaml')
    with open(commonFilePath, 'r') as f:
        data = yaml.load(f, Loader=Loader)

    serviceFilePath = os.path.join(get_package_share_directory(pkgName), 'launch/service.json')
    with open(serviceFilePath, 'r') as f:
        serviceData = json.load(f)

    return LaunchDescription([
        Node(
            package=pkgName,
            namespace=data['generic_prop']['namespace'],
            executable="server",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "chassisFilePath" : data['chassis_prop']['chassisFilePath'], 
                    "joystickFilePath" : data['chassis_prop']['joystickFilePath'], 
                    "internalIDServerIP" : data['internal_prop']['hostIP'], 
                    "internalIDServerPort" : data['internal_prop']['port'], 
                    "internalIDServerDeviceID" : data['internal_prop']['ID'], 
                    "externalIDServerIP" : data['external_prop']['hostIP'], 
                    "externalIDServerPort" : data['external_prop']['port'], 
                    "externalIDServerDeviceID" : data['external_prop']['ID'], 
                    "serviceName" : data['service_prop']['serviceName'] + '_' + str(data['generic_prop']['id']), 
                    "topicName" : data['topic_Control']['topicName'] + '_' + str(data['generic_prop']['id']), 
                    "publishInterval_ms" : data['topic_Control']['publishInterval_ms'], 
                    "enableOutput" : data['control_prop']['enableOutput'], 
                    "outputPeriod_ms" : data['control_prop']['outputPeriod_ms'], 

                    # Settings for Params class under vehicle_interfaces/params.h
                    # Do not change the settings rashly
                    "nodeName" : data['generic_prop']['nodeName'] + '_' + str(data['generic_prop']['id']) + '_node', 
                    "id" : data['generic_prop']['id'], 
                    "devInfoService" : serviceData['devInfoService'], 
                    "devInterface" : serviceData['devInterface'], 
                    "devMultiNode" : serviceData['devMultiNode'], 
                    "qosService" : serviceData['qosService'], 
                    "qosDirPath" : serviceData['qosDirPath'], 
                    "safetyService" : serviceData['safetyService'], 
                    "timesyncService" : serviceData['timesyncService'], 
                    "timesyncPeriod_ms" : serviceData['timesyncPeriod_ms'], 
                    "timesyncAccuracy_ms" : serviceData['timesyncAccuracy_ms'], 
                    "timesyncWaitService" : serviceData['timesyncWaitService'], 
                }
            ]
        )
    ])
