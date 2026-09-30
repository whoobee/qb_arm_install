# Glossary

| Term | Meaning |
|---|---|
| **ACM** (Allowed Collision Matrix) | MoveIt table of body pairs whose contact is not treated as a collision |
| **Action** | ROS request/response for long tasks, with feedback, cancellable (e.g. executing a trajectory) |
| **Agent (micro-ROS)** | Program on qBArm that represents micro-ROS clients (the ESP32) in the normal ROS graph |
| **Antipodal grasp** | Two contacts whose connecting line lies in both friction cones; it holds by squeezing |
| **Approach axis** | Direction the gripper moves in towards the object; `link_tcp` z |
| **Attached object** | An object MoveIt treats as part of the robot (moves with the TCP) after grasping |
| **BusLinker** | Hiwonder adapter between a UART and the servo's half-duplex bus |
| **Cartesian path** | Motion in which the TCP follows a straight line in space |
| **Cell** | The whole running system on qBArm, managed by the `cell` script as one process group |
| **Closing axis** | Line along which the fingers move; `link_tcp` y |
| **Collision object** | A shape in MoveIt's planning scene (table, detected objects) |
| **Contact-GraspNet (CGN)** | Neural network predicting 6-DoF grasps from a point cloud (trained for the Panda hand) |
| **Deprojection** | Pixel + depth → 3D point, using the camera intrinsics |
| **Discovery server** | Fast DDS service through which ROS 2 nodes find each other (instead of multicast) |
| **Extrinsics** | Pose of the camera relative to the robot (`world → camera_base`) |
| **Finger drop** | How much further along the approach the claw's fingers are when closed than when open (up to 18.8 mm) |
| **Flying pixels** | Depth pixels on object edges that mix foreground and background and float in between |
| **Vision workspace** | The polygon (and z range) in `config/boundaries.yaml` the camera looks at; everything else is blacked out before detection |
| **Veil (mixed-pixel ramp)** | Flying pixels behind a tall edge seen from the camera: a gradual ramp (10–25 mm beside a 10 cm bin rim) that no per-pixel filter separates from a real object; those cells count as not seen |
| **FK / IK** | Forward kinematics (joints → pose) / inverse kinematics (pose → joints) |
| **Grasp** | Pose of the TCP + width at which the gripper closes on an object |
| **Height map** | Grid of 1 cm cells over the table, each holding the height of what stands there (`surface_map`); used to check place spots and container fill |
| **Grip target** | Claw command when gripping (1.2 rad), past pads-touching, so the position-controlled servo keeps pushing on the object |
| **INA219** | Current/voltage sensor (I²C) in the servo supply, for a real "gripping" signal |
| **qbarm-claw** | qBArm's own Wi-Fi access point for the claw (second USB adapter, 10.42.0.1) |
| **Stall guard** | Firmware limit: a servo stopped short of its target keeps only 30 steps of push; heat derating |
| **Grounding DINO** | Open-vocabulary object detector: text prompt → boxes |
| **ICP** | Iterative Closest Point: aligns a point cloud to a model by repeatedly matching nearest points |
| **Intrinsics (K)** | Camera focal lengths and principal point (fx, fy, cx, cy) |
| **KDL** | The numerical IK solver used for the Lite6 |
| **Keep-out zone** | A box in `config/boundaries.yaml` the arm may never enter: a MoveIt collision object `keepout_<name>` |
| **`link_tcp`** | The claw's tool centre point: between the open pads, z along the fingers, y closing |
| **LX protocol** | Hiwonder bus-servo serial protocol (`0x55 0x55`, id, length, command, params, checksum) |
| **micro-ROS** | ROS 2 for microcontrollers |
| **Mimic joint** | URDF joint that copies another joint's value (× multiplier) |
| **Mode (xArm)** | Controller mode; ROS control needs servo mode 1; after faults the arm is often in mode 0 |
| **MoveIt / move_group** | Motion planning framework / its central node |
| **NMS** | Non-maximum suppression: of overlapping boxes keep only the most confident |
| **Octomap** | 3D occupancy grid built from sensor data, used as an obstacle model |
| **OMPL** | Library of sampling-based motion planners used by MoveIt (RRT for the Lite6 here) |
| **OTA** | Over-the-air firmware update (over Wi-Fi) |
| **Parallelogram linkage** | Two equal bars keeping the finger parallel while it moves on an arc |
| **Planning group** | Set of joints MoveIt plans together (`lite6`, `qbag`) |
| **Planning scene** | MoveIt's world model: robot state, objects, ACM, attached objects |
| **Multipath (ToF)** | Time-of-flight light that bounces off another surface before returning: inside a white bin the floor reads ~1 cm too low |
| **Pre-grasp** | Pose before the grasp, backed off along the approach (10 cm), reached by free motion |
| **Process group** | Unix group of processes that can be signalled together (`kill -- -PGID`) |
| **ros2_control** | ROS framework running controllers against hardware interfaces |
| **RViz** | ROS 3D visualisation |
| **SAM 2** | Segment Anything 2: box → pixel mask |
| **Servo mode** | Lite6 mode 1, in which the controller follows streamed joint positions |
| **SRDF** | Semantic robot description: groups, named states, end effector, disabled collisions |
| **TCP** | Tool centre point (`link_tcp`) |
| **TF** | ROS system that tracks transforms between coordinate frames |
| **TOTG** | Time-optimal trajectory generation: assigns timestamps within velocity/acceleration limits |
| **Trajectory** | Joint positions over time |
| **URDF / xacro** | Robot description (links and joints) / XML macro language to generate it |
| **XRCE-DDS** | The DDS variant micro-ROS clients speak with the agent |
