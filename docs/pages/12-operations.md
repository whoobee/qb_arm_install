# Operations

Runbook for daily use: starting and stopping, picking, calibration, recovery, safety rules and troubleshooting.

**The control page: http://192.168.1.171:8081** — start / stop the cell, watch the arm and the claw (temperatures,
current, angle), detect objects with a typed prompt, pick and place them by clicking in the camera image (plan first,
then execute), edit the boundaries, follow the log.
Everything below also works from a terminal.

## Safety rules

> 1. **Never send the real arm to the all-zero joint pose** (xArm "home", UFACTORY app "go home") with the claw
>    mounted: the claw ends up inside the robot base.
> 2. **Keep the emergency stop within reach** during every real motion. First runs of anything new: `plan_only` first,
>    check the plan in RViz, then execute.
> 3. **Never power the ESP32 from the buck converter and USB at the same time** (back-feed into the PC's USB port).
> 4. After a fault or an emergency stop, **a person recovers the arm** (clear the error, move it clear). The software
>    does not retry.
> 5. Don't leave the claw squeezing an object for long: the servo heats up while holding (watch `/claw/servo_temperature`;
>    the firmware derates from 60 °C and goes limp at 70 °C — a held object then drops).

## Start and stop

Always through `cell` — it starts everything as one process group and stops all of it.

```bash
cell status                 # anything running? leftovers?
cell start real             # real arm + real claw + camera + MoveIt + RViz + vision
cell start sim              # simulated arm and claw (camera is real)
cell start real obstacles:=false  # extra launch arguments pass through (here: no camera octomap)
cell log                    # follow the output
cell stop                   # stop everything (waits until it is gone)
cell stop --force           # also kill leftovers from runs not started with cell
```

After ~60 s the cell is ready (MoveIt, controllers, camera, vision). Closing RViz stops the cell.
Discovery through the discovery server is slow: CLI tools may need 10–15 s before they see new topics.

## Detect and pick

```bash
# detect (prompt: phrases separated by ". "; descriptive phrases work best: "white bin." rather than "bin.")
# the reply lists the objects by name: white_bin, tape_1, tape_2 (same name: nearest to the robot first)
ros2 service call /qb_arm_vision/detect qb_arm_vision_interfaces/srv/Detect "{prompt: 'tape roll. bottle.'}"
# plan only - check the plan in RViz
ros2 service call /qb_arm_vision/pick qb_arm_vision_interfaces/srv/Pick "{object_id: tape, plan_only: true}"
# execute
ros2 service call /qb_arm_vision/pick qb_arm_vision_interfaces/srv/Pick "{object_id: tape}"
# set it down at a point on the table ((0, 0) = back where it was picked)
ros2 service call /qb_arm_vision/place qb_arm_vision_interfaces/srv/Place "{position: {x: 0.25, y: 0.10}}"
# or relative to a detected object: on / into / next_to (side: left, right, front, back or empty = any)
ros2 service call /qb_arm_vision/place qb_arm_vision_interfaces/srv/Place "{relation: next_to, reference: white_bin}"
# into a container: the reply says how full it is ("white_bin 7% full, 35% of the inside seen"); plan_only first
ros2 service call /qb_arm_vision/place qb_arm_vision_interfaces/srv/Place "{relation: into, reference: white bin, plan_only: true}"
# or open the claw and drop it where it is
ros2 service call /qb_arm_vision/release std_srvs/srv/Trigger
```

In RViz: the **Detections** image shows masks, labels, grasp counts and why detections were dropped; markers show
object hulls (blue = graspable) and grasps (green = best). Every place first checks the spot in a fresh **height map**
(add a MarkerArray display on `/qb_arm_vision/surface_map_markers` to see it: a cube per 1 cm cell, blue = table,
red = 10 cm and higher, gaps = not seen). A place is refused when the spot is occupied (*"something 23 mm high at …"*),
mostly not seen (*"70 % of it not seen by the camera"* — e.g. under the arm or right behind a tall object), or, into a
container, full there — or *"under the arm"*: the camera can't see below the arm, so a spot right under or beside
it can't be checked (place elsewhere, or move the arm away first).
`ros2 param set /qb_arm_vision/pick_executor check_place false` turns the check off for the next places.

## Boundaries: where the camera looks, where the arm may never go

**Editor** — the **Boundaries** tab of the control page **http://192.168.1.171:8081**: a top view of the table
made from the camera (the image projected onto the table plane, so rectangles are rectangles in robot coordinates;
tall things look stretched). **+ vision zone** (green, drag its corners freely) / **+ no-go zone** (red box: drag
to move, corners to resize; heights, turn and name in the side panel; the dashed outline is the MoveIt margin).
Heights: one z range for all vision zones, one per no-go zone (a liftable desk: from the floor to above the arm's
reach). The zones are checked live exactly as the cell checks them; **Preview in camera view** shows what the
camera would ignore; **Save** writes the file (the old one is kept as `boundaries.yaml.<time>.bak`), **Save &
restart cell** also restarts the cell (the arm doesn't move).

By hand:

`~/prj/ros2_ws/src/qb_arm/config/boundaries.yaml`: the **vision workspace** (a polygon on the table; the detector
ignores everything outside it) and the **keep-out zones** (boxes MoveIt never lets the arm enter, e.g. the desk).
After editing: `cell stop`, `cell start real` (no build). A broken file stops the cell with the reason in the log
(`cell log`); another file: `cell start real boundaries_file:=/path/file.yaml`. Check:

```bash
ros2 run qb_arm show_boundaries --output ~/boundaries.png   # green workspace, red keep-out, dark = ignored
```

In RViz the keep-out zones are red boxes in the planning scene, the workspace a green outline (Markers display). On
start-up `planning_scene_setup` logs each zone and whether the arm is clear of them; if the arm is inside one, every
plan fails until it is moved out by hand.

## The claw by hand

```bash
ros2 topic pub -r 1 /claw/command std_msgs/msg/Float64 "{data: 0.0}"    # open (Ctrl-C after it moved)
ros2 topic pub -r 1 /claw/command std_msgs/msg/Float64 "{data: 0.96}"   # close
ros2 topic pub -r 1 /claw/torque std_msgs/msg/Bool "{data: false}"      # limp: move it by hand
ros2 topic echo /claw/joint_states --field position
ros2 topic echo /claw/servo_temperature
ros2 topic echo /claw/rssi                                              # Wi-Fi signal (dBm)
```

The simulated claw (sim cell) listens on `/sim_claw/command` instead.

Use `-r 1` for a few seconds instead of `--once`: a one-shot publisher can exit before discovery has matched it.

## Recovery after a fault

Symptoms: a pick fails with `MoveIt error -4`; `/ufactory/robot_states` shows `err` ≠ 0 or `state` 4.

```bash
ros2 topic echo --once /ufactory/robot_states | grep -E "^(state|mode|err):"
```

1. Look at the arm: what did it touch? Is it clear to move?
2. Clear the error and move the arm clear (UFACTORY app, or manual mode) — **not** to the zero pose.
3. The arm is now typically in mode 0; MoveIt needs servo mode 1. The next pick request sets it back automatically
   (only when there is no arm error); `cell stop && cell start real` works too.
4. Detect again before the next pick (objects may have moved).

Known error codes: **C31** collision caused abnormal joint current; **C16** servo error joint 6 (fixed once by
power-cycling the arm).

## Calibration

| What | How |
|---|---|
| Camera tilt | camera still, cell (or `kinect`) running: `ros2 run qb_arm measure_camera_tilt` |
| Camera yaw (unknown) | arm visible: `ros2 run qb_arm fit_camera_yaw --apply` |
| Camera pose (fine) | `ros2 run qb_arm refine_camera_pose --apply` (4-DOF; `--full` for 6-DOF) |
| Table plane | table mostly clear: `ros2 run qb_arm measure_table --apply` (after moving the table or the camera) |
| Claw open/closed positions | torque off, move by hand to each end, read `/claw/joint_states`, set `CLAW_OPEN_POS`/`CLAW_CLOSED_POS` in `platformio.ini`, `pio run -e gripper_ota -t upload` |
| Claw mounting | `config/claw.yaml` (`mount_offset`, `mount_yaw`), check the model against the real claw in RViz |

Restart the cell after changing YAML files; for qb_arm_vision's config also `colcon build` (its config files are copied, not symlinked).

## Firmware update

```bash
cd ~/prj/qb_arm_gripper && pio run -e gripper_ota -t upload
```

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `cell start` refuses: leftover processes | an earlier run not started with `cell`: `cell stop --force` |
| `No Kinect images` / `Failed to open K4A device` | the camera was still held by a previous driver; `cell stop`, wait a few seconds, start again |
| `No reachable grasp for <object>` | object too far (> ~33 cm from the base for top-down grasps), or the start state is invalid (check the log for `CheckStartStateCollision`) |
| `CheckStartStateCollision ... claw_* - link_base` | the arm is at/near the zero pose (sim: `sim_ready_pose` should have moved it) |
| `Invalid Trajectory: start point deviates` | the arm moved between planning and execution, or two executors are running: `cell status` |
| `Claw did not reach X rad` after closing | expected when gripping (the object stops the fingers) |
| Claw topics missing | ESP32 not powered / not on Wi-Fi: `ping 10.42.0.10`, `iw dev wlxec750c316d15 station dump` (is it connected to `qbarm-claw`?), `nmcli con show --active` (is `qbarm-claw` up?); agent: `systemctl status ros2-microros-agent` |
| `qbarm-claw` won't start: dnsmasq "address in use" | an orphaned dnsmasq from a crashed NetworkManager: `pgrep -a dnsmasq`, kill the one with `10.42.0.1`, `sudo nmcli con up qbarm-claw` |
| "Nothing grasped" although the object was held | the empty check (`check_grip`) can't tell from servo position alone; keep it off until the INA219 is fitted |
| Claw doesn't move, but answers | servo supply off (voltage ~3.3 V instead of ~7.3 V) |
| Detection fails / slow | GPU server: `curl http://hbh-ai.local:8770/health`; on hbh-ai `docker compose logs -f` |
| `Overrun detected!` in the log | controller loop timing under CPU load (no real-time kernel); harmless unless constant |
| move_group crashed during octomap use | known MoveIt race (planning scene request with the octomap component reads it without a lock); never request the `OCTOMAP` component of `/get_planning_scene` in own code |
