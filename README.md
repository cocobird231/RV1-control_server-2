*`Established: 2024/04/29`* *`Updated: 2024/04/29`*

## About The Project
The chassis control signal management service for robot vehicle ver. 1 project. The project aims to provide the control signal management service for the robot vehicle. The service will receive the control signal from the controller node, check the safety of the control signal, and send the control signal to the chassis motor node. The service will also provide the multiple control signal priority management, the control signal will be executed based on the priority level.

## Getting Started

### Prerequisites
- ROS2 `Foxy` or later (`Humble` recommended)
    Install ROS2 from official website: [ROS2 official website](https://docs.ros.org/en/humble/Installation.html) or simply run the following command to automatically install ROS2:
    ```bash
    curl -fsSL ftp://61.220.23.239/scripts/install-ros2.sh | bash
    ```
    **NOTE:** The script only supports `Foxy` and `Humble` versions depending on the Ubuntu version.
    **NOTE:** The script will create a new workspace at `~/ros2_ws`.
    **NOTE:** The script will create an alias `humble` or `foxy` for global ROS2 environment setup (e.g. `source /opt/ros/<$ROS_DISTRO>/setup.bash`) depending on the ROS2 version.
- [vehicle_interfaces](https://github.com/cocobird231/RV1-vehicle_interfaces.git)
- nlohmann-json3-dev

The required packages are listed in the `requirements_apt.txt` file. Install the required packages by running the following command:
```bash
xargs sudo apt install -y < requirements_apt.txt
```
**NOTE:** The required packages will be installed automatically while installing the package using the (`vcu-installer`)[https://github.com/cocobird231/RV1-vcu-install.git].


### Installation
There are two ways to install the package: manually or using the `vcu-installer`. 

#### Install Manually
1. Check if `vehicle_interfaces` package is installed. If not, install the package by following the instructions in the [vehicle_interfaces](https://github.com/cocobird231/RV1-vehicle_interfaces.git).
2. Clone the repository under `~/ros2_ws/src` and rename it to `cpp_controlserver`:
    ```bash
    git clone https://github.com/cocobird231/RV1-controlserver.git cpp_controlserver
    ```
3. Change the directory to the `~/ros2_ws` workspace and build the package:
    ```bash
    # Change directory to workspace.
    cd ~/ros2_ws

    # Source the local environment.
    . install/setup.bash

    # Build the package.
    colcon build --symlink-install --packages-select cpp_controlserver
    ```
    **NOTE:** The package is installed in the local workspace.


#### Install Using `vcu-installer`
1. Run the installer and press `Scan` button under Package Management window. If the installer not installed, install the installer by following the instructions in the [`vcu-installer`](https://github.com/cocobird231/RV1-vcu-install.git).

2. Checked the `Control Server 2` checkbox under package list, right-click to modify the internet setting, then press the `Install` button to install the package.

3. The installer will create the start-up script for the package under `/etc/xdg/autostart` directory. The package will be started automatically after the system boot-up.


## Usage
The package contains two executables: `server` and `control`. The `server` executable is used to run the main service, while the `control` executable is used to control the service.

### Run the Main Service
First, modify the `common.yaml` file to set the parameters for the control server serivce, especially the `chassisFilePath` and `joystickFilePath`.

The `chassisFilePath` is the path to the chassis configuration file, the chassis configuration file describes the components of the chassis, it must be imported to calculate the incoming control signal.
**NOTE:** The control server will not work properly if the chassis configuration file is **not imported**.

The `joystickFilePath` is the path to the joystick configuration file. The joystick configuration file describes the joystick information, it must be imported to receive the control signal from the joystick.
**NOTE:** The joystick plays an important role in the control server, it has the highest priority to send emergency stop signal to the chassis motor node, and it can switch the output signal between multiple controllers, or decide to enable/disable the safety check.
**NOTE:** The control server will not work properly if joystick is **not connected**.

1. Source the local workspace:
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

2. Run the main service using `launch`:
    ```bash
    ros2 launch cpp_controlserver launch.py
    ```
    **NOTE:** The `common.yaml` file default the namespace to `V0`.


### Control the Service
1. Source the local workspace:
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

2. Run the control service:
    ```bash
    ros2 run cpp_controlserver control
    ```
    **NOTE:** If the main service is using the namespace (e.g. `V0`), the control service should use the same namespace to control the main service:
    ```bash
    ros2 run cpp_controlserver control --ros-args -r __ns:=/V0
    ```


## Description

### Control Service
The control executable demonstrates the control of the control server using `ControlServer.srv` under vehicle_interfaces package. The `ControlServer.srv` contains the following items:
```.srv
# Request field
ControlServerStatus request # Send ControlServerStatus to control server.

# Response field
bool response # Whether the service is successfully executed.

string reason # If response flase, describes the reason.

ControlServerStatus status # Response current control server status.
```

The `ControlServerStatus` message can be used to describe the control signal of control server, or used to describe the current status of control server. The `ControlServerStatus` message contains the following items:
```.msg
# Set to CONTROLLER_ACTION_XXX. See ControlServerStatus.msg for more information.
uint8 controller_action 0

# The the service name of controller which to be selected as main controller or remove from list.
string controller_service_name

# Set to SERVER_ACTION_XXX. See ControlServerStatus.msg for more information.
uint8 server_action 0

# The status of output timer. Set to TIMER_STATUS_XXX.
uint8 server_output_timer_status 0

# The period of output signals in _ms. Server will ignore this if value <= 0.
float64 server_output_period_ms 0

# The status of safety check timer. Set to TIMER_STATUS_XXX.
uint8 server_safety_timer_status 0

# The period of safety check in _ms. Server will ignore this if value <= 0.
float64 server_safety_period_ms 0

# The status of idclient check timer. Set to TIMER_STATUS_XXX.
uint8 server_idclient_timer_status 0

# The period of idclient check in _ms. Server will ignore this if value <= 0.
float64 server_idclient_period_ms 0


# The status of publish timer. Set to TIMER_STATUS_XXX.
uint8 server_publish_timer_status 0

# The period of publish in _ms. Server will ignore this if value <= 0.
float64 server_publish_period_ms 0

# Set to CHASSIS_ACTION_XXX.
uint8 chassis_action 0

# The information of chassis settings.
ChassisInfo chassis_info
```

### `common.yaml` File
The `common.yaml` file is used to set the parameters for the main service. The parameters are listed below:
- `chassis_prop`:
    - `chassisFilePath`: (string) The path to the chassis configuration file.
        The chassis configuration file contains the chassis motor information, the file should be in the JSON format.
    - `joystickFilePath`: (string) The path to the joystick configuration file.
        The joystick configuration file contains the joystick information, the file should be in the JSON format.
- `internal_prop`:
    - `hostIP`: (string) The host IP address of the internal ID server.
        The internal ID server is used to communicate with the chassis motor node.
    - `port`: (int) Not used.
        Not used.
    - `ID`: (int) Not used.
        Not used.
- `service_prop`:
    - `serviceName`: (string) The service name.
        The service name revealed on the ROS2 network.
- `topic_prop`:
    - `topicName`: (string) The topic name.
        The topic name revealed on the ROS2 network.
- `timer_prop`:
    - `enableOutput`: (bool) Enable the output timer.
        The output timer is used to send the control signal to the chassis motor node.
    - `outputPeriod_ms`: (double) The period of output timer in milliseconds.
        The output timer period is used to set the period of sending the control signal to the chassis motor node.
    - `enableSafetyCheck`: (bool) Enable the safety timer.
        The safety timer is used to check the safety of the control signal.
    - `safetyCheckPeriod_ms`: (double) The period of safety timer in milliseconds.
        The safety timer period is used to set the period of checking the safety of the control signal.
    - `enableIdclientCheck`: (bool) Enable the ID client timer.
        The ID client timer is used to check the ID client connection.
    - `idclientCheckPeriod_ms`: (double) The period of ID client timer in milliseconds.
        The ID client timer period is used to set the period of checking the ID client connection.
    - `enablePublish`: (bool) Enable the publish timer.
        The publish timer is used to publish the control signal status.
    - `publishPeriod_ms`: (double) The period of publish timer in milliseconds.
        The publish timer period is used to set the period of publishing the control signal status.
- `generic_prop`:
    - `namespace`: (string) The namespace of the node.
        The namespace is used to separate the services.
    - `nodeName`: (string) The node name.
        The node name revealed on the ROS2 network.
    - `id`: (int) The node id.
        The id of the node.

### `chassis.json` File
The `chassis.json` describes the components of the chassis, including wheel information, motor mapping and correction etc.. For more information, check the website: [To be added]().

### `joystick.json` File
The `joystick.json` describes the joystick information, including the joystick button mapping, joystick axis mapping, etc.. For more information, check the website: [To be added]().