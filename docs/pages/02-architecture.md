# System architecture

This page describes the cell top-down: hardware and computers, the network, the software components and where they
run, the ROS graph (nodes, topics, services, actions), the coordinate frames, and the process model.
The per-module pages go one level deeper.

## Physical setup

```mermaid
flowchart TB
    subgraph ceiling["Ceiling, ~1.47 m above the table"]
        K["Azure Kinect DK<br/>RGB 1280x720 + depth 640x576 (NFOV)<br/>IMU"]
    end
    subgraph table["Table"]
        A["UFACTORY Lite6<br/>6-DoF arm, reach ~0.44 m"]
        CL["qB-AdaptiveGripper claw<br/>on a 20 mm plate, rotated -45 deg"]
        O["Objects<br/>(tape rolls, bottles, ...)"]
        A --- CL
    end
    subgraph box["Claw electronics (on the arm)"]
        E["ESP32-C3"] -->|"UART TX/RX 115200"| BL["Hiwonder BusLinker V2.5"]
        BL -->|"half-duplex bus"| S["HX-06L bus servo"]
        PSU["24 V rail -> buck 7.2 V (servo), buck 5 V (logic)"]
    end
    K -. "looks down on" .-> table
    S -->|"gears drive the crank"| CL
```

## Computers and network

```mermaid
flowchart LR
    subgraph qbarm["qBArm - Ubuntu 24.04, ROS 2 Jazzy - 192.168.1.171 (Wi-Fi)"]
        direction TB
        DS["Fast DDS discovery server<br/>UDP 11811 (systemd)"]
        MRA["micro-ROS agent<br/>UDP 8888 (systemd)"]
        CELL["the cell (cell start real)<br/>drivers, MoveIt, RViz, perception, pick"]
        DOC["docs server<br/>TCP 8080 (systemd)"]
        AP["qbarm-claw access point<br/>USB Wi-Fi (Archer T4U), 10.42.0.1"]
    end
    subgraph gpu["hbh-ai - Pop!_OS, RTX 3060 - 192.168.1.220"]
        SRV["qb_arm_vision container<br/>FastAPI, TCP 8770"]
    end
    ARM["Lite6 controller<br/>192.168.1.23"]
    ESP["Claw ESP32-C3<br/>qbag-fad7bc, 10.42.0.10"]
    KIN["Azure Kinect"]
    CELL -- "xArm SDK (TCP)" --> ARM
    CELL -- "USB 3" --> KIN
    CELL -- "HTTP POST /pipeline<br/>RGB-D snapshot" --> SRV
    ESP -- "Wi-Fi 2.4 GHz, channel 1" --> AP
    AP -- "micro-ROS / XRCE-DDS over UDP" --> MRA
    CELL -. "DDS via discovery server" .- DS
    MRA -. "DDS" .- DS
```

| Host | Address | Role |
|---|---|---|
| qBArm | `192.168.1.171` (DHCP, not reserved yet) | Everything ROS: drivers, MoveIt, RViz, perception client, pick logic, micro-ROS agent, discovery server, docs |
| Lite6 controller | `192.168.1.23` | Arm controller; qBArm talks to it with the UFACTORY SDK inside the ros2_control hardware plugin |
| hbh-ai | `192.168.1.220` (`hbh-ai.local`) | GPU inference server (shared with other services: ollama, immich, speech-to-speech on the second GPU) |
| Claw ESP32-C3 | `10.42.0.10` on `qbarm-claw` (fixed by MAC) | Claw controller; micro-ROS client; OTA on port 3232 |
| Spare ESP32-C3 | `10.42.0.11` on `qbarm-claw` | Same firmware, not wired; only one board may run at a time |
| `qbarm-claw` | `10.42.0.1/24`, 2.4 GHz channel 1 | qBArm's own access point for the claw, on a second (USB) Wi-Fi adapter next to the arm; NetworkManager, WPA2, DHCP with fixed addresses per board |

**ROS discovery.** ROS 2 nodes normally find each other with multicast. qBArm instead runs a *Fast DDS discovery
server* ("ROS master"-like) on UDP 11811; every node points to it with `ROS_DISCOVERY_SERVER`
(`ros_env.sh`). Other machines join with `ROS_DISCOVERY_SERVER=192.168.1.171:11811`. Side effect: new nodes and
CLI tools need ~10–15 s until they see all topics.

## Software components

```mermaid
flowchart TB
    subgraph cell["The cell - one process group, started by cell start sim|real"]
        subgraph arm["Arm (xarm_ros2 + qb_arm)"]
            CM["ros2_control_node<br/>controller_manager + UFRobotSystemHardware<br/>(fake hardware in sim)"]
            TC["lite6_traj_controller<br/>JointTrajectoryController"]
            JSP["joint_state_publisher<br/>merges arm + claw joints (real)"]
            RSP["robot_state_publisher<br/>URDF -> TF"]
            MG["move_group<br/>MoveIt"]
            RV["rviz2"]
        end
        subgraph cam["Camera (namespace /kinect)"]
            KD["Azure Kinect driver"]
            CTF["static TF world -> camera_base<br/>from camera_pose.yaml"]
            OC["obstacle_cloud (optional)"]
        end
        subgraph vis["Perception + picking (namespace /qb_arm_vision)"]
            OD["object_detector"]
            PE["pick_executor"]
        end
        PSS["planning_scene_setup<br/>adds the table"]
        CR["claw_relay / claw_driver<br/>(sim only)"]
        SRP["sim_ready_pose (sim only)"]
    end
    subgraph ext["Outside the cell"]
        MRA2["micro-ROS agent"] --- ESP2["ESP32 firmware<br/>node /claw/qbag_esp32"]
        GPU2["GPU server on hbh-ai"]
    end
    OD -- HTTP --> GPU2
    OD -- "collision objects" --> MG
    PE -- "IK, plans, execution,<br/>scene edits" --> MG
    PE -- "/claw/command" --> ESP2
    MG -- "FollowJointTrajectory" --> TC
    TC --- CM
    ESP2 -- "/claw/joint_states" --> JSP
    CM -- "/ufactory/joint_states" --> JSP
    JSP -- "/joint_states" --> RSP
    JSP -- "/joint_states" --> MG
    KD -- "RGB, depth" --> OD
    KD -- "depth" --> OC
    OC -- "/kinect/obstacle_points" --> MG
```

### Who does what

| Component | Package | Responsibility |
|---|---|---|
| `ros2_control_node` | xarm_ros2 | Runs the controller manager at 150 Hz; the hardware plugin `UFRobotSystemHardware` talks to the Lite6 controller (servo mode, mode 1) |
| `lite6_traj_controller` | ros2_controllers | Executes joint trajectories (action `/lite6_traj_controller/follow_joint_trajectory`) |
| `joint_state_publisher` | ROS | Real arm only: merges `/ufactory/joint_states` (arm) and `/claw/joint_states` (claw) into `/joint_states` |
| `robot_state_publisher` | ROS | Turns the URDF + `/joint_states` into the TF tree |
| `move_group` | MoveIt | Planning scene, collision checking, IK, motion planning (OMPL), Cartesian paths, trajectory execution |
| `rviz2` | ROS | Visualisation and interactive planning; **closing it stops the whole cell** (xArm launch behaviour) |
| Kinect driver | qb_arm_kinectdk_ros2 | Camera images, depth, point cloud, IMU, camera model (namespace `/kinect`) |
| `world_to_camera_base` | qb_arm | Static TF: where the camera hangs, from `config/camera_pose.yaml` |
| `obstacle_cloud` | qb_arm | Depth → workspace-cropped, voxel-thinned cloud for MoveIt's octomap (only with `obstacles:=true`) |
| `planning_scene_setup` | qb_arm | Adds the table as a collision box, then exits |
| `object_detector` | qb_arm_vision | Service `/qb_arm_vision/detect`: snapshot → GPU server → objects + grasps → planning scene, markers, debug image. Service `/qb_arm_vision/surface_map`: height map of a region from 7 fresh depth frames, robot cut out |
| `pick_executor` | qb_arm_vision | Services `/qb_arm_vision/pick`, `/place`, `/release`: grasp → MoveIt plans → execution → claw; sets the held object down after checking the spot in a height map |
| ESP32 firmware | qb_arm_gripper | Node `/claw/qbag_esp32`: `/claw/command`, `/claw/torque` → servo; publishes `/claw/joint_states`, voltage, temperature |
| `claw_driver` | qb_arm | Sim only: a simulated claw with the same topics (or `claw_relay` for sim arm + real claw) |
| `sim_ready_pose` | qb_arm | Sim only: moves the fake arm off the all-zero pose (claw inside the base) |
| `ufactory_driver` (`/uf_api`) | xarm_ros2 (`xarm_api`) | Real arm only: UFACTORY's service driver (second connection to the controller); the pick executor uses `set_mode`/`set_state`/`motion_enable` to restore servo mode |
| GPU server | qb_arm_vision/server | `POST /pipeline`: Grounding DINO, SAM 2, Contact-GraspNet |

## The ROS graph

Interfaces between the components (real arm). `→` publishes/calls.

| Name | Kind | Type | From → To |
|---|---|---|---|
| `/joint_states` | topic | `sensor_msgs/JointState` | joint_state_publisher → robot_state_publisher, move_group, pick_executor |
| `/ufactory/joint_states` | topic | `sensor_msgs/JointState` | hardware plugin → joint_state_publisher |
| `/ufactory/robot_states` | topic | `xarm_msgs/RobotMsg` | hardware plugin → (monitoring: state, mode, err) |
| `/claw/joint_states` | topic | `sensor_msgs/JointState` | ESP32 → joint_state_publisher (20 Hz) |
| `/claw/command` | topic | `std_msgs/Float64` | pick_executor, you → ESP32 (claw_joint in rad) |
| `/claw/torque` | topic | `std_msgs/Bool` | you → ESP32 (false = limp) |
| `/claw/supply_voltage`, `/claw/temperature` | topic | `std_msgs/Float32` | ESP32 → (monitoring, 1 Hz) |
| `/tf`, `/tf_static` | topic | `tf2_msgs/TFMessage` | robot_state_publisher, static publishers → everyone |
| `/kinect/rgb/image_raw`, `/kinect/depth_to_rgb/image_raw`, `/kinect/rgb/camera_info` | topic | `sensor_msgs/Image`, `CameraInfo` | Kinect driver → object_detector (only during a snapshot / surface map) |
| `/kinect/depth/image_raw` | topic | `sensor_msgs/Image` | Kinect driver → obstacle_cloud |
| `/kinect/obstacle_points` | topic | `sensor_msgs/PointCloud2` | obstacle_cloud → move_group (octomap) |
| `/qb_arm_vision/detect` | service | `qb_arm_vision_interfaces/Detect` | you → object_detector |
| `/qb_arm_vision/objects` | topic (latched) | `ObjectArray` | object_detector → pick_executor |
| `/qb_arm_vision/markers`, `/qb_arm_vision/debug_image` | topic (latched) | `MarkerArray`, `Image` | object_detector → RViz |
| `/qb_arm_vision/surface_map` | service | `qb_arm_vision_interfaces/SurfaceMap` | pick_executor → object_detector |
| `/qb_arm_vision/surface_map_markers` | topic (latched) | `MarkerArray` | object_detector → RViz (the last height map: a cube per seen cell, blue = table, red = 10 cm+) |
| `/robot_description` | topic (latched) | `std_msgs/String` | robot_state_publisher → object_detector (link collision boxes), pick_executor |
| `/qb_arm_vision/pick` | service | `qb_arm_vision_interfaces/Pick` | you → pick_executor |
| `/qb_arm_vision/place` | service | `qb_arm_vision_interfaces/Place` | you → pick_executor |
| `/qb_arm_vision/release` | service | `std_srvs/Trigger` | you → pick_executor |
| `/compute_ik`, `/compute_cartesian_path`, `/get_planning_scene`, `/apply_planning_scene`, `/check_state_validity` | services | MoveIt | pick_executor, object_detector → move_group |
| `/move_action`, `/execute_trajectory` | actions | MoveIt | pick_executor → move_group |
| `/lite6_traj_controller/follow_joint_trajectory` | action | `control_msgs/FollowJointTrajectory` | move_group → controller |

## Coordinate frames (TF tree)

Every position in the system is expressed in some frame; TF knows how they relate at every moment.
`world` is the robot base on the table surface.

```mermaid
flowchart TB
    world["world<br/>(= robot base, on the table)"] --> link_base
    link_base --> link1 --> link2 --> link3 --> link4 --> link5 --> link6 --> link_eef["link_eef<br/>(flange)"]
    link_eef -->|"20 mm up, -45 deg about z<br/>(config/claw.yaml)"| og["other_geometry_link<br/>(claw base)"]
    og -->|"91.5 mm"| tcp["link_tcp<br/>(centre between the open pads)"]
    og --> lc["claw_left_crank"] --> lf["claw_left_finger"]
    og --> lr["claw_left_rocker"]
    og --> rc["claw_right_crank"] --> rf["claw_right_finger"]
    og --> rr["claw_right_rocker"]
    world -->|"static, camera_pose.yaml<br/>x -0.380 y 0.571 z 1.475"| cb["camera_base"]
    cb --> dcl["depth_camera_link"]
    cb --> rcl["rgb_camera_link"]
    cb --> imu["imu_link"]
```

- **`link_tcp`** (tool centre point) is the frame every grasp is expressed in: origin between the two open grip
  pads, **z** pointing along the fingers (the approach direction), **y** the direction the fingers close in.
- **`rgb_camera_link`** is the frame of the colour image and the registered depth; the perception pipeline
  deprojects pixels there and transforms them to `world`.

## Process model: the `cell` script

Everything that belongs to one run of the cell is started as **one process group** and stopped as one. This is what
keeps runs clean (see lesson 6 on the [status page](01-status.md)).

```mermaid
stateDiagram-v2
    [*] --> Stopped
    Stopped --> Refused: cell start, but leftover cell processes exist
    Refused --> Stopped: cell stop --force
    Stopped --> Running: cell start sim|real<br/>setsid ros2 launch qb_arm MODE.launch.py
    Running --> Stopping: cell stop<br/>SIGINT to the whole group
    Running --> Stopping: RViz closed<br/>(launch shuts down)
    Stopping --> Stopping: still alive after 20 s -> SIGTERM,<br/>after 10 s more -> SIGKILL
    Stopping --> Stopped: group empty
```

Launch file hierarchy:

```mermaid
flowchart LR
    real["real.launch.py<br/>sim:=false claw_hw:=true"] --> cellL["cell.launch.py"]
    sim["sim.launch.py<br/>sim:=true claw_hw:=false"] --> cellL
    cellL --> bring["bringup.launch.py"]
    cellL --> odl["qb_arm_vision<br/>object_detector.launch.py"]
    cellL --> srp["sim_ready_pose (sim)"]
    bring --> lm["lite6_moveit.launch.py<br/>URDF/SRDF with the claw,<br/>ros2_control, move_group, RViz"]
    bring --> kin["kinect.launch.py<br/>driver + camera TF"]
    bring --> occ["obstacle_cloud (obstacles:=true)"]
    bring --> pss["planning_scene_setup"]
```

Always running outside the cell (systemd): `ros2-discovery`, `ros2-microros-agent`, `qb-arm-docs`.
The claw firmware is independent of the cell: it keeps its connection to the agent and holds its last command.
