# Operations

Runbook for daily use: starting and stopping, picking, calibration, recovery, safety rules and troubleshooting.

**The control page: http://192.168.1.171:8081** — start / stop the cell, watch the arm and the claw (temperatures,
current, angle — always in the status bar), detect objects with a typed prompt, pick an object by clicking it and
place it by clicking the target (a menu at the mouse: into / on / next to, or a point on the table; plan first, then
execute), tune speeds and pauses and edit the boundaries (Config), read the log with filters and search.
Everything below also works from a terminal.

## Safety rules

> 1. **Never send the real arm to the all-zero joint pose** (xArm "home", UFACTORY app "go home") with the claw
>    mounted: the claw ends up inside the robot base.
> 2. **Keep the emergency stop within reach** during every real motion. First runs of anything new: `plan_only` first,
>    check the plan in RViz, then execute.
> 3. **Never power the ESP32 from the buck converter and USB at the same time** (back-feed into the PC's USB port).
> 4. **The software recovers arm faults by itself** (since 2026-10-04): an error during a motion — a collision too — is
>    cleared and the step carried on, up to 3 times per step ([automatic recovery](06-pick-execution.md)). **The
>    emergency stop is the safety**: its codes (C1, C2) are never cleared automatically. Off: `auto_recover: false`.
> 5. Don't leave the claw squeezing an object for long: the servo heats up while holding (watch `/claw/servo_temperature`;
>    the firmware derates from 60 °C and goes limp at 70 °C — a held object then drops).

## Start and stop

Always through `cell` — it starts everything as one process group and stops all of it.

```bash
cell status                 # anything running? leftovers?
cell start real             # real arm + real claw + camera + MoveIt + RViz + vision
cell start sim              # simulated arm and claw (camera is real)
cell start real obstacles:=false  # extra launch arguments pass through (here: no camera octomap)
cell start real ik_solver:=kdl    # the xArm's KDL IK solver instead of TRAC-IK (the default)
cell start real joint5_limit:=124 # joint5 (wrist bend) over its full range, not +-115 deg (C22 risk; to get out of a fold)
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
# back to the home pose (after a place it goes there by itself); save the current pose as home
ros2 service call /qb_arm_vision/home qb_arm_vision_interfaces/srv/Home "{plan_only: true}"
ros2 service call /qb_arm_vision/save_home std_srvs/srv/Trigger
```

In RViz: the **Detections** image shows masks, labels, grasp counts and why detections were dropped; markers show
object hulls (blue = graspable) and grasps (green = best). Every place first checks the spot in a fresh **height map**
(add a MarkerArray display on `/qb_arm_vision/surface_map_markers` to see it: a cube per 1 cm cell, blue = table,
red = 10 cm and higher, gaps = not seen). A place is refused when the spot is occupied (*"something 23 mm high at …"*),
mostly not seen (*"70 % of it not seen by the camera"* — e.g. under the arm or right behind a tall object), or, into a
container, full there — or *"under the arm"*: the camera can't see below the arm, so a spot right under or beside
it can't be checked (place elsewhere, or move the arm away first).
`ros2 param set /qb_arm_vision/pick_executor check_place false` turns the check off for the next places.

## Holding things: jog, spots and poses

Control tab, with the cell running and the switch on **real** (in *plan* every button only plans, shown in RViz):
the **jog pad** over the camera image moves the claw (and whatever it holds) in small straight steps — pick the step,
click a direction (inner ring: forward / back / left / right; outer ring: tilt; left bar: up / down; right bar: turn;
**home** in the middle, click twice; **open** / **close** the claw under it), or tick *keys*: arrows, PgUp / PgDn, Q / E. Jog steps are **not
collision-checked** — watch the claw. To come back to a place: **+ save here** in the bar at the bottom of the image,
a name, *as spot* (position) or *as pose* (position + orientation); its button under *go to* (click twice) takes the
arm there. Or click free table in the camera image: **go here** takes the claw above that point (height in cm, 10 by
default; claw down or as it is). Exact values and dragging on a top view: Config → spots.

```bash
ros2 service call /qb_arm_vision/jog qb_arm_vision_interfaces/srv/Jog "{translation: {z: 0.005}}"
ros2 service call /qb_arm_vision/go_to qb_arm_vision_interfaces/srv/GoTo "{name: 'solder spot', plan_only: true}"
```

## Handing objects over: give and hold this

Control tab, the bar at the bottom of the camera image, switch on **real**, with the cell running and the hand tracker on (the live view shows your hands):

- **give to me** (click twice; the claw holds something): hold your hand out still in front of the arm, 34–75 cm from
  its base, not over the desk. The arm comes slowly and stops with the object ~10 cm in front of your palm. Take it:
  it opens once your hand has been at the object for 1 s, or as soon as you pull on it. Then it backs off and goes home.
- **hold this** (click twice; the claw is empty): hold the object out still. The claw opens and comes to ~10 cm in front
  of your palm. Put the object between the fingers and **keep holding it**, your fingers at least 4 cm clear of the
  claw (hold it by its far end): it closes after 0.5 s. Small objects, or the camera can't see between the fingers:
  press **close now**. It **holds it right there** (`handed_1`, `handed_2`, …): let go, then jog it where you need it
  (or go to a spot). The reply says the size it measured once your hand had left — MoveIt checks later moves with
  that box. When done: **give to me** gives it back, **home** takes it home, or click a spot or an object in the image
  to put it there (*back where picked* doesn't exist for it). Nothing put in within 30 s: the claw stays open there.
- **stop** stops the arm where it is. It also stops by itself when a hand comes within 5 cm of the arm. When your hand
  moves or the camera loses it, it pauses and goes on once your hand is still again (at most 3 times).
- With the switch on **plan**, *give to me* / *hold this* only plan the way to your hand (RViz).

```bash
ros2 service call /qb_arm_vision/handover qb_arm_vision_interfaces/srv/Handover "{action: take, plan_only: false}"
ros2 service call /qb_arm_vision/close std_srvs/srv/Trigger     # during a take: close now
ros2 service call /qb_arm_vision/stop std_srvs/srv/Trigger      # stop where it is
```

Messages: *"No hand to take from / to give to"* — no hand was still in the zone within 15 s (the live view shows whether
the camera sees it). *"Your hand at (…) is out of the arm's reach"* — no natural, collision-free pose in front of it;
the reply says what blocked it. *"Nothing to hold within 30 s"* / *"Nobody took … within 30 s"* — it went home.

## Boundaries: where the camera looks, where the arm may never go

**Editor** — **Config → boundaries** on the control page **http://192.168.1.171:8081**: a top view of the table
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

Normally automatic (`auto_recover`): a fault during a motion is cleared and the step carried on. By hand only after
the emergency stop, when a step faulted 3 times, or with `auto_recover: false`.

Symptoms: a pick fails with `MoveIt error -4`; `/ufactory/robot_states` shows `err` ≠ 0 or `state` 4.

```bash
ros2 topic echo --once /ufactory/robot_states | grep -E "^(state|mode|err):"
```

1. Look at the arm: what did it touch? Is it clear to move?
2. Clear the error: the control page's status bar → **arm · recover** (shows the fault code, C…; click twice), or the
   Status tab → **recover arm** (with the fault's meaning). It clears the error and the warning,
   turns the motors on and sets servo mode, state ready — the arm doesn't move; the driver re-activates the
   trajectory controller once the arm is ready. Or: UFACTORY app / manual mode. If the arm has to be moved clear,
   by hand or in the app — **not** to the zero pose.
3. After the UFACTORY app the arm is typically in mode 0; MoveIt needs servo mode 1. The next pick request sets it back automatically
   (only when there is no arm error); `cell stop && cell start real` works too.
4. Detect again before the next pick (objects may have moved).

Known error codes: **C31** collision caused abnormal joint current; **C16** servo error joint 6 (fixed once by
power-cycling the arm); **C24** speed exceeds limit — travel speed 0.7 over Wi-Fi: a stalled control cycle is caught up in one step, which
the arm sees as 2–3× the planned speed (keep the speeds low until the arm is wired, Config → motion).

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
| Camera / live view dead while the cell runs, log: `Failed to poll cameras: node cannot continue` | a capture timeout ends the Kinect driver; since 2026-10-05 the launch restarts it after 3 s (`respawn` in the driver fork's `driver.launch.py`) — the camera is back within ~10 s. Before: only a cell restart helped |
| Control page: no live view, no hands, arm state stale — but the cell runs and `ros2 topic hz /kinect/rgb/image_raw` shows frames | the control center's ROS thread had died (a subscription destroyed while it waited — after a boundaries preview); fixed 2026-10-05 (it logs and spins on); before: `sudo systemctl restart qb-arm-control` |
| `No Kinect images` / `Failed to open K4A device` | the camera was still held by a previous driver; `cell stop`, wait a few seconds, start again |
| `No reachable grasp for <object>` | object too far (> ~33 cm from the base for top-down grasps), or the start state is invalid (check the log for `CheckStartStateCollision`) |
| Every pick/place fails ("no reachable …"), log: `Joint 'jointN' from the starting state is outside bounds` | the arm stopped a hair past a ±2π limit (float rounding); the executor clamps values within 1 mrad of a limit (since 3edbb84); further out: jog that joint inward by hand |
| `CheckStartStateCollision ... claw_* - link_base` | the arm is at/near the zero pose (sim: `sim_ready_pose` should have moved it) |
| `trajectory controller lite6_traj_controller is unconfigured` (before: a bare `MoveIt error -4`, log: *Action client not connected … follow_joint_trajectory*) | rare since 2026-10-02: the pick executor configures / activates the controller itself before a motion when the arm is ready, and the spawner starts 8 s late. Still there: the message says why — *start-up did not finish*: `cell stop`, `cell start real`; *arm not ready*: recover the arm (Cell tab) |
| `Invalid Trajectory: start point deviates` | the arm moved between planning and execution, or two executors are running: `cell status` |
| `Claw did not reach X rad` after closing | expected when gripping (the object stops the fingers) |
| Claw topics missing | ESP32 not powered / not on Wi-Fi: `ping 10.42.0.10`, `iw dev wlxec750c316d15 station dump` (is it connected to `qbarm-claw`?), `nmcli con show --active` (is `qbarm-claw` up?); agent: `systemctl status ros2-microros-agent` |
| `qbarm-claw` won't start: dnsmasq "address in use" | an orphaned dnsmasq from a crashed NetworkManager: `pgrep -a dnsmasq`, kill the one with `10.42.0.1`, `sudo nmcli con up qbarm-claw` |
| "Nothing grasped" although the object was held | the reply gives the stop angle and holding current; compare with the measured table (pick execution → grip check) and adjust `grip_empty_angle` / `grip_empty_current` (`ros2 param set`, live), or `check_grip false` |
| Claw doesn't move, but answers | servo supply off (voltage ~3.3 V instead of ~7.3 V) |
| Detection fails / slow | GPU server: `curl http://hbh-ai.local:8770/health`; on hbh-ai `docker compose logs -f` |
| `Overrun detected!` in the log | controller loop timing under CPU load (no real-time kernel); harmless unless constant |
| move_group crashed during octomap use | known MoveIt race (planning scene request with the octomap component reads it without a lock); never request the `OCTOMAP` component of `/get_planning_scene` in own code |
