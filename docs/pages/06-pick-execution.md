# Pick execution

How `pick_executor` turns a detected object and its grasps into arm and claw motion, and how `release` ends a pick.
Source: `qb_arm_vision/qb_arm_vision/pick_executor.py`, parameters in `config/pick_executor.yaml`.

## Interfaces

| Interface | Type | Purpose |
|---|---|---|
| `/qb_arm_vision/pick` | service `Pick` | `{object_id, plan_only}` → `{success, message, grasp}` |
| `/qb_arm_vision/place` | service `Place` | `{position, plan_only}`: set the held object down at a point on the table |
| `/qb_arm_vision/release` | service `std_srvs/Trigger` | open the claw, detach and remove the held object (drop it) |
| `/qb_arm_vision/objects` | subscription (latched) | the latest detection: objects by id |
| `/joint_states` | subscription | current `claw_joint` (to wait for the claw) |
| `/claw/command` | publisher | claw target angle |
| MoveIt | clients | `/compute_ik`, `/move_action`, `/compute_cartesian_path`, `/execute_trajectory`, `/get_planning_scene`, `/apply_planning_scene` |

**Before every pick** (also `plan_only`), on the real arm: the arm's controller state (`/ufactory/robot_states`) is
checked. An arm **error** (`err` ≠ 0) ends the request with the error code — a person recovers the arm. If the arm is
**not in servo mode** (mode ≠ 1, or state 4 stopped / 5 config changed — e.g. after manual/teach mode or the UFACTORY
app), the pick enables the motors and sets mode 1 and state 0 through UFACTORY's service driver (`/uf_api/...`, started
by the cell) and waits for the arm to confirm; the arm does not move. Without this, every trajectory is aborted with
MoveIt error −4 until the cell restarts.

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
(`other_geometry_link`, cranks, fingers, rockers) may touch the target object. Everything else — arm links vs. the object,
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

## Place

`/qb_arm_vision/place` (`qb_arm_vision_interfaces/srv/Place`) puts the object held since the last successful pick down:

| `relation` | Where | Surface its bottom goes to |
|---|---|---|
| `""` | its centre at `position` (x, y in `world`; (0, 0) = back where it was picked) | the table plane |
| `"on"` | centred on `reference` (an object id from the latest detection, e.g. `white_bin`) | the reference's top + 3 mm |
| `"into"` | above `reference` (cup, box, …): the centre, or — if that is out of reach — shifted towards the robot inside the opening in 2 cm steps as long as it still fits | the reference's rim + 1 cm, then released (the camera can't see how deep it is) |
| `"next_to"` | beside `reference` on `side` (`left` +y, `right` −y, `front` +x away from the robot, `back`; empty = every side, nearest to the robot first), `gap` apart (default 2 cm) | the table plane |

```bash
ros2 service call /qb_arm_vision/place qb_arm_vision_interfaces/srv/Place "{relation: next_to, reference: white_bin}"
ros2 service call /qb_arm_vision/place qb_arm_vision_interfaces/srv/Place "{relation: on, reference: white_bin, plan_only: true}"
ros2 service call /qb_arm_vision/place qb_arm_vision_interfaces/srv/Place "{relation: into, reference: white bin}"
```

- **next_to spacing**: the reference's outline and the held object's outline (the detector's convex hulls, not their
  bounding boxes — a round object's box overestimated by ~40 %) plus the gap — and at least the **open claw's reach**:
  when the claw lets go its pads swing out to 46 mm from the TCP along the closing axis, which can be more than the object.
  The reply states the real distance between the objects.
- **into fit check**: the held object goes down centred on the container, in its pick orientation or turned 180°; its
  outline must stay inside the container's outline minus a 5 mm wall **in every direction** (72 directions checked), else
  the place is refused with how much too wide it is. After an "into" the object is removed from the scene (where it fell
  is unknown; the next detection sees it).
- **The opening claw is collision-checked**: the claw opens without a plan, so before a candidate is accepted MoveIt
  checks the final pose at claw angles from the gripped one down to fully open in 0.12 rad steps
  (`/check_state_validity` with the claw's group `qbag`: only the moving claw links; a whole-robot check also failed on
  unrelated contacts, e.g. a bin's outline touching the robot base). Part-way the fingers are already outside the object
  but still low. A candidate whose fingers would hit the reference, another object or the
  table is skipped.
- **next_to** uses the whole open claw's reach (pads 46 mm, finger knuckles 64 mm along the closing axis, palm 35 mm
  across) and refuses a reference that stands on something unmodelled (its support would be under the held object).
- **into, off centre**: a bin 44 cm from the base was out of reach at its centre (the Lite6 reaches ~40 cm at that
  height); 2–3 cm towards the robot everything was reachable.
- After a place (not into) the object's new pose (`T_place · T_grasp⁻¹ · T_object`, down by the clearance) replaces its old
  one in the executor's detection, so it can serve as a reference right away.
- Detect again while holding is fine: the detector leaves out the object in the claw (near the TCP and floating above
  the table) and numbers new objects after the held one, so ids don't clash.

```mermaid
flowchart TB
    S(["place(relation, reference / x, y)"]) --> H{"holding an object<br/>from a pick?"}
    H -- no --> F1(["fail: pick first"])
    H -- yes --> G["TCP height above the object's bottom at the grasp:<br/>h = z_tcp(grasp) - bottom(object)"]
    G --> T["candidate targets: at (x, y) / on / into / next to the reference<br/>(each side), same orientation or turned 180 deg,<br/>z = surface + h + clearance"]
    T --> C["for 10 cm / 5 cm above: IK check, plan there,<br/>straight way down (MoveIt carries the object),<br/>open claw at the end collision-free?"]
    C -- none --> F2(["fail: no reachable, collision-free way down"])
    C -- ok --> PO{"plan_only?"}
    PO -- yes --> R1(["success: planned"])
    PO -- no --> M1["execute: above the place"] --> M2["execute: straight down (5% speed)"]
    M2 --> O["open the claw"] --> D["detach: the object stays in the scene where it stands"]
    D --> U["straight up 10 cm"] --> R2(["success: placed"])
```

The object moves rigidly with the claw, so its pose after placing is `T_place · T_grasp⁻¹ · T_object`, with `T_grasp` the
TCP pose recorded when the object was attached. The height is **computed, not felt**: the claw lowers to where the
object's bottom should be 3 mm above the measured table plane (`place_clearance`) and opens. If the object slipped in
the claw it ends up that much off (no force/current sensing yet). The servo-mode check runs first, as for the pick.

| Parameter | Default | Meaning |
|---|---|---|
| `place_clearance` | 0.003 m | object bottom above the table when the claw opens |
| `place_distances` | `[0.10, 0.05]` | m, above the place: the straight way down starts here |
| `retreat_distance` | 0.10 m | straight up afterwards |
| `into_clearance`, `into_margin` | 0.01 m, 0.01 m | release height above a container's rim; both walls together |
| `next_to_gap` | 0.02 m | default gap between the outlines |
| `table_plane` | from qb_arm `config/table.yaml` | measured table plane |

First real run (2026-09-30): tape roll picked 40 cm from the base, placed at (0.25, 0.10); the camera found it at
(0.252, 0.112) afterwards.

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
ros2 service call /qb_arm_vision/pick qb_arm_vision_interfaces/srv/Pick "{object_id: tape, plan_only: true}"
ros2 service call /qb_arm_vision/pick qb_arm_vision_interfaces/srv/Pick "{object_id: tape}"
ros2 service call /qb_arm_vision/place qb_arm_vision_interfaces/srv/Place "{position: {x: 0.25, y: 0.10}}"
# or drop it where it is:
ros2 service call /qb_arm_vision/release std_srvs/srv/Trigger
```
