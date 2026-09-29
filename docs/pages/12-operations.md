# Operations

Runbook for daily use: starting and stopping, picking, calibration, recovery, safety rules and troubleshooting.

## Safety rules

> 1. **Never send the real arm to the all-zero joint pose** (xArm "home", UFACTORY app "go home") with the claw
>    mounted: the claw ends up inside the robot base.
> 2. **Keep the emergency stop within reach** during every real motion. First runs of anything new: `plan_only` first,
>    check the plan in RViz, then execute.
> 3. **Never power the ESP32 from the buck converter and USB at the same time** (back-feed into the PC's USB port).
> 4. After a fault or an emergency stop, **a person recovers the arm** (clear the error, move it clear). The software
>    does not retry.
> 5. Don't leave the claw squeezing an object for long: the servo heats up while holding (watch `/claw/temperature`).

## Start and stop

Always through `cell` — it starts everything as one process group and stops all of it.

```bash
cell status                 # anything running? leftovers?
cell start real             # real arm + real claw + camera + MoveIt + RViz + vision
cell start sim              # simulated arm and claw (camera is real)
cell start real obstacles:=true   # extra launch arguments pass through
cell log                    # follow the output
cell stop                   # stop everything (waits until it is gone)
cell stop --force           # also kill leftovers from runs not started with cell
```

After ~60 s the cell is ready (MoveIt, controllers, camera, vision). Closing RViz stops the cell.
Discovery through the discovery server is slow: CLI tools may need 10–15 s before they see new topics.

## Detect and pick

```bash
# detect (prompt: phrases separated by ". ")
ros2 service call /qb_arm_vision/detect qb_arm_vision_interfaces/srv/Detect "{prompt: 'tape roll. bottle.'}"
# plan only - check the plan in RViz
ros2 service call /qb_arm_vision/pick qb_arm_vision_interfaces/srv/Pick "{object_id: obj_1, plan_only: true}"
# execute
ros2 service call /qb_arm_vision/pick qb_arm_vision_interfaces/srv/Pick "{object_id: obj_1}"
# open the claw and drop the object
ros2 service call /qb_arm_vision/release std_srvs/srv/Trigger
```

In RViz: the **Detections** image shows masks, labels, grasp counts and why detections were dropped; markers show
object hulls (blue = graspable) and grasps (green = best).

## The claw by hand

```bash
ros2 topic pub -r 1 /claw/command std_msgs/msg/Float64 "{data: 0.0}"    # open (Ctrl-C after it moved)
ros2 topic pub -r 1 /claw/command std_msgs/msg/Float64 "{data: 0.96}"   # close
ros2 topic pub -r 1 /claw/torque std_msgs/msg/Bool "{data: false}"      # limp: move it by hand
ros2 topic echo /claw/joint_states --field position
ros2 topic echo /claw/temperature
```

Use `-r 1` for a few seconds instead of `--once`: a one-shot publisher can exit before discovery has matched it.

## Recovery after a fault

Symptoms: a pick fails with `MoveIt error -4`; `/ufactory/robot_states` shows `err` ≠ 0 or `state` 4.

```bash
ros2 topic echo --once /ufactory/robot_states | grep -E "^(state|mode|err):"
```

1. Look at the arm: what did it touch? Is it clear to move?
2. Clear the error and move the arm clear (UFACTORY app, or manual mode) — **not** to the zero pose.
3. The arm is now typically in mode 0; MoveIt needs servo mode 1: `cell stop && cell start real` sets it up again.
4. Detect again before the next pick (objects may have moved).

Known error codes: **C31** collision caused abnormal joint current; **C16** servo error joint 6 (fixed once by
power-cycling the arm).

## Calibration

| What | How |
|---|---|
| Camera tilt | camera still, cell (or `kinect`) running: `ros2 run qb_arm measure_camera_tilt` |
| Camera yaw (unknown) | arm visible: `ros2 run qb_arm fit_camera_yaw --apply` |
| Camera pose (fine) | `ros2 run qb_arm refine_camera_pose --apply` (4-DOF; `--full` for 6-DOF) |
| Claw open/closed positions | torque off, move by hand to each end, read `/claw/joint_states`, set `CLAW_OPEN_POS`/`CLAW_CLOSED_POS` in `platformio.ini`, `pio run -e gripper_ota -t upload` |
| Claw mounting | `config/claw.yaml` (`mount_offset`, `mount_yaw`), check the model against the real claw in RViz |

Restart the cell after changing YAML files.

## Firmware update

```bash
cd ~/prj/qb_arm_gripper && pio run -e gripper_ota -t upload
```

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `cell start` refuses: leftover processes | an earlier run not started with `cell`: `cell stop --force` |
| `No Kinect images` / `Failed to open K4A device` | the camera was still held by a previous driver; `cell stop`, wait a few seconds, start again |
| `No reachable grasp for obj_N` | object too far (> ~33 cm from the base for top-down grasps), or the start state is invalid (check the log for `CheckStartStateCollision`) |
| `CheckStartStateCollision ... claw_* - link_base` | the arm is at/near the zero pose (sim: `sim_ready_pose` should have moved it) |
| `Invalid Trajectory: start point deviates` | the arm moved between planning and execution, or two executors are running: `cell status` |
| `Claw did not reach X rad` after closing | expected when gripping (the object stops the fingers) |
| Claw topics missing | ESP32 not powered / not on Wi-Fi: `ping 192.168.1.123`; agent: `systemctl status ros2-microros-agent` |
| Claw doesn't move, but answers | servo supply off (voltage ~3.3 V instead of ~7.3 V) |
| Detection fails / slow | GPU server: `curl http://hbh-ai.local:8770/health`; on hbh-ai `docker compose logs -f` |
| `Overrun detected!` in the log | controller loop timing under CPU load (no real-time kernel); harmless unless constant |
| move_group crashed during octomap use | known MoveIt race (planning scene request with the octomap component reads it without a lock); obstacles are off by default |
