# Pick execution

How `pick_executor` turns a detected object and its grasps into arm and claw motion, and how `release` ends a pick.
Source: `qb_arm_vision/qb_arm_vision/pick_executor.py`, parameters in `config/pick_executor.yaml`.

## Interfaces

| Interface | Type | Purpose |
|---|---|---|
| `/qb_arm_vision/pick` | service `Pick` | `{object_id, plan_only}` → `{success, message, grasp}` |
| `/qb_arm_vision/release` | service `std_srvs/Trigger` | open the claw, detach and remove the held object |
| `/qb_arm_vision/objects` | subscription (latched) | the latest detection: objects by id |
| `/joint_states` | subscription | current `claw_joint` (to wait for the claw) |
| `/claw/command` | publisher | claw target angle |
| MoveIt | clients | `/compute_ik`, `/move_action`, `/compute_cartesian_path`, `/execute_trajectory`, `/get_planning_scene`, `/apply_planning_scene` |

The node runs a multi-threaded executor with a re-entrant callback group, so a service callback can wait for
MoveIt replies (futures) while the executor keeps processing them. A lock rejects a second pick while one is running.

## The pick, step by step

```mermaid
flowchart TB
    S(["pick(object_id, plan_only)"]) --> O{"object in the<br/>latest detection<br/>and has grasps?"}
    O -- no --> F1(["fail: unknown object / no grasp"])
    O -- yes --> ACM["allow claw <-> object contact<br/>(allowed collision matrix)"]
    ACM --> LOOP["for each grasp (best first)<br/>x turn 0 / 180 deg<br/>x pre-grasp 10 cm / 5 cm"]
    LOOP --> IK{"IK collision-free for<br/>grasp and pre-grasp?"}
    IK -- no --> NEXT["next candidate"]
    IK -- yes --> PL{"plan to pre-grasp<br/>(OMPL)"}
    PL -- fail --> NEXT
    PL -- ok --> CA{"straight approach<br/>>= 99% of the line?"}
    CA -- no --> NEXT
    CA -- yes --> CH["chosen"]
    NEXT --> LOOP
    LOOP -- "none left" --> F2(["fail: no reachable grasp"])
    CH --> PO{"plan_only?"}
    PO -- yes --> R1(["success: planned<br/>(shown in RViz)"])
    PO -- no --> OPEN["open the claw"]
    OPEN --> M1["execute: move to pre-grasp<br/>(20% speed)"]
    M1 --> M2["execute: approach<br/>(5% speed)"]
    M2 --> CL["close to grip_target 1.2 rad,<br/>wait until the fingers stop"]
    CL --> AT["attach the object to link_tcp"]
    AT --> LI{"straight lift 10 cm<br/>(>= 2 cm possible)?"}
    LI -- no --> F3(["fail: grasped but cannot lift"])
    LI -- yes --> M3["execute: lift"]
    M3 --> R2(["success: picked, lifted N cm"])
```

### 1. Allow contact with the target

The approach ends *inside* the object's collision shape (the fingers go around it, and the extruded hull is solid,
hole included), so the pick edits the planning scene's **allowed collision matrix**: every claw link
(`other_geometry_link`, cranks, fingers, rockers) may touch `obj_<n>`. Everything else — arm links vs. the object,
claw vs. table, claw vs. other objects — is still checked.

### 2. Candidates

For each of the object's grasps (best first), each grasp also **turned 180° about its approach axis** (the claw is
symmetric; the turned version often suits the arm's wrist better), and each **pre-grasp distance** (10 cm, then
5 cm for tall objects at the edge of the reach):

```
T      = grasp pose (possibly turned)
T_pre  = T with its position moved back by d along its z axis (the approach):  t_pre = t − d · z
```

### 3. Quick reachability (IK)

`reachable(T)` and `reachable(T_pre)`: `/compute_ik` for `link_tcp` with collision avoidance, 50 ms timeout. This
rejects out-of-reach candidates in milliseconds, before any planning.

### 4. Plan to the pre-grasp

`/move_action` with `plan_only`: goal = position constraint (sphere of 2 mm around `T_pre`) + orientation constraint
(0.01 rad), 3 attempts, 5 s, velocity/acceleration scaling 0.2. The result is a joint trajectory from the current
state. (The start state is the *current* robot state — which must be valid, i.e. collision-free.)

### 5. Plan the straight approach

`/compute_cartesian_path` from the **end** of the pre-grasp trajectory (its last joint positions are used as the start
state) to `T`, 5 mm steps, collisions checked, velocity scaling 0.05. Needs ≥ 99 % of the line. The first candidate
for which steps 3–5 succeed is chosen; with `plan_only` the pick stops here (MoveIt shows the plan in RViz).

### 6. Execute

1. **Open the claw** (`/claw/command` 0.0) and wait until `claw_joint` is within 0.02 rad (max. 5 s).
2. **Move to the pre-grasp** (`/execute_trajectory`).
3. **Approach** in a straight line, slowly.
4. **Close**: command `grip_target` (1.2 rad, **past** pads-touching at 0.96) and wait until the claw stops moving
   (position unchanged for 0.4 s) — the object stops the fingers, and the position-controlled servo keeps pushing
   with the remaining error: that is the grip force. The firmware's stall guard then reduces the push to 30 servo
   steps (see [claw firmware](10-qb_arm_gripper.md)). The estimated width is **not** used for closing (it once
   was, and a fallback turned a 13.8 mm wall into 81 mm, leaving the fingers 60 mm apart); it only sets the grasp
   height. The stop position is logged as the gripped width. With `check_grip`, a claw that closes to within
   `empty_margin` of fully closed counts as "nothing grasped": the claw opens and the pick ends without lifting.
   **Off by default**, because the servo reads ~1.01 rad both empty and on a tape wall; it needs the INA219.
5. **Attach** the object to `link_tcp` (touch links = the claw links). From now on MoveIt carries it with the arm.
6. **Lift**: straight line 10 cm up from the current TCP pose; at least 2 cm must be possible (edge of the reach).

### Failure handling

Any exception (MoveIt not answering, a rejected goal, execution failure) ends the pick with `success=false` and the
message; nothing is retried automatically and the arm stays where it stopped. If the arm itself faulted (e.g. C31),
the controller aborts the trajectory and MoveIt reports error −4; the arm must be recovered by a person
(see [operations](12-operations.md)).

## Release

`/qb_arm_vision/release` opens the claw (waits for it), then, if an object is attached, **detaches** it and
**removes** it from the planning scene. The object falls from wherever the claw is — there is no *place* motion yet.

While an object is attached, poses where it would collide are invalid, including the start of any plan that begins
with the object inside the table or the robot; release before planning elsewhere.

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `group` | `lite6` | MoveIt planning group |
| `tcp_link` | `link_tcp` | link the grasps refer to |
| `pregrasp_distances` | `[0.10, 0.05]` | m, tried in order |
| `lift_distance` | 0.10 | m |
| `min_lift_distance` | 0.02 | m, a shorter lift is accepted |
| `velocity_scaling` / `acceleration_scaling` | 0.2 / 0.2 | of the joint limits, free motion and lift |
| `approach_velocity_scaling` | 0.05 | final straight approach |
| `planning_time` | 5.0 | s per plan |
| `grip_target` | 1.2 | rad, claw command when gripping (past fully closed; clamped to 1.2) |
| `check_grip`, `empty_margin` | false, 0.03 | fail the pick if the claw closes to within `empty_margin` of 0.96 (never in sim) |
| `claw_joint`, `claw_links` | | names in the URDF |

`grip_target`, `check_grip` and `empty_margin` are read at every pick: `ros2 param set /qb_arm_vision/pick_executor ...`
tunes them without a restart.

## Example session

```bash
cell start real
ros2 service call /qb_arm_vision/detect qb_arm_vision_interfaces/srv/Detect "{prompt: 'tape roll.'}"
ros2 service call /qb_arm_vision/pick qb_arm_vision_interfaces/srv/Pick "{object_id: obj_1, plan_only: true}"
ros2 service call /qb_arm_vision/pick qb_arm_vision_interfaces/srv/Pick "{object_id: obj_1}"
ros2 service call /qb_arm_vision/release std_srvs/srv/Trigger
```
