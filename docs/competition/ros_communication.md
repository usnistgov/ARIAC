# ROS Communication Overview

## Topics

List of topics with the message type and a brief description.

| Topic Name                     | MSG type                            | Description                                          |
| ---                            | ---                                 | ---                                                  | 
| `/ariac/orders`                | `ariac_msgs/msg/Order`              | Orders that the competitors should submit            |
| `/ariac/competition_state`     | `ariac_msgs/msg/CompetitionState`   | Current state of the competition                     | 
| `/ariac/bin_parts`             | `ariac_msgs/msg/BinParts`           | Parts in each bin at program start-up                |
| `/ariac/conveyor_parts`        | `ariac_msgs/msg/ConveyorParts`      | Parts that will come on the conveyor belt            |
| `/ariac/agv{n}_status`         | `ariac_msgs/msg/AGVStatus`          | State of the AGV {n} (location, position, velocity)  |
| `/ariac/{robot}_gripper_state` | `ariac_msgs/msg/VacuumGripperState` | State of {robot}'s gripper (enabled, attached, type) |
| `/ariac/conveyor_state`        | `ariac_msgs/msg/ConveyorBeltState`  | State of the conveyor (enabled, power)               |
| `/ariac/robot_health`          | `ariac_msgs/msg/Robots`             | Health of the robots                                 |
| `/ariac/sensor_health`         | `ariac_msgs/msg/Sensors`            | Health of the sensors                                |

## Services

List of service with the service type and a brief description.

| Service Name                    | SRV type                              | Description                                                        |
| ---                             | ---                                   | ---                                                                | 
| `/ariac/start_competition`      | `std_srvs/srv/Trigger`                | Start the competition                                              |
| `/ariac/end_competition`        | `std_srvs/srv/Trigger`                | End the competition                                                | 
| `/ariac/submit_order`           | `ariac_msgs/srv/SubmitOrder`          | Submit an order with the requested `order_id`                      |
| `/ariac/perform_quality_check`  | `ariac_msgs/srv/PerformQualityCheck`  | Check the quality of a kitting order with the requested `order_id` |
| `/ariac/move_agv{n}`            | `ariac_msgs/srv/MoveAGV`              | Move the AGV {n} to the requested location                         |
| `/ariac/agv{n}_lock_tray`       | `std_srvs/srv/Trigger`                | Lock a kit tray to AGV {n}                                         |
| `/ariac/agv{n}_unlock_tray`     | `std_srvs/srv/Trigger`                | Unlock a kit tray to AGV {n}                                       |
| `/ariac/{robot}_enable_gripper` | `ariac_msgs/srv/VacuumGripperControl` | Set the state of {robot}'s gripper to the request state            |
| `/ariac/{robot}_change_gripper` | `ariac_msgs/srv/ChangeGripper`        | Change the type of {robot}'s gripper to the request type           |

## Sensor Topics

List of sensor topics and their msg types:

| Sensor Type               | Topic name(s)                                                                       |	MSG type                                              |
| ---                       | ---                                                                                 | ---                                                   |
| `break_beam`              | `/ariac/sensors/{sensor_name}/status` `/ariac/sensors/{sensor_name}/status`         | ariac_msgs/BreakBeamStatus ariac_msgs/BreakBeamStatus |
| `proximity`               | `/ariac/sensors/{sensor_name}/scan`                                                 |	sensor_msgs/Range                                     |
| `laser_profiler`          | `/ariac/sensors/{sensor_name}/scan`                                                 |	sensor_msgs/LaserScan                                 |
| `lidar`	                  | `/ariac/sensors/{sensor_name}/scan`	                                                | sensor_msgs/PointCloud                                |
| `rgb_camera`              | `/ariac/sensors/{sensor_name}/rgb_image`                                            |	sensor_msgs/Image sensor_msgs/Image                   |
| `rgbd_camera`             | `/ariac/sensors/{sensor_name}/rgb_image` `/ariac/sensors/{sensor_name}/depth_image` | sensor_msgs/Image                                     |
| `basic_logical_camera`    | `/ariac/sensors/{sensor_name}/image`                                                | ariac_msgs/BasicLogicalCameraImage                    |
| `advanced_logical_camera` | `/ariac/sensors/{sensor_name}/image`                                                | ariac_msgs/AdvancedLogicalCameraImage                 |