# Pick execution

How `pick_executor` turns a detected object and its grasps into arm and claw motion, how `place` sets it down
(after checking the spot in a fresh height map), how `release` ends a pick and how the arm hands objects to the user's
hand and takes them from it ([handover](#handover-give-and-take)).
Source: `qb_arm_vision/qb_arm_vision/pick_executor.py`, parameters in `config/pick_executor.yaml`.

## Interfaces

| Interface | Type | Purpose |
|---|---|---|
| `/qb_arm_vision/pick` | service `Pick` | `{object_id, plan_only}` → `{success, message, grasp}` |
| `/qb_arm_vision/place` | service `Place` | `{position, relation, reference, side, gap, plan_only}`: set the held object down at a point, on, into or next to another object |
| `/qb_arm_vision/surface_map` | client (`SurfaceMap`, object_detector) | fresh height map of the place, to check it's free / how full a container is |
| `/qb_arm_vision/release` | service `std_srvs/Trigger` | open the claw, detach and remove the held object (drop it) |
| `/qb_arm_vision/home` | service `qb_arm_vision_interfaces/Home` | move the arm to its home pose (`plan_only` to only plan) |
| `/qb_arm_vision/jog` | service `qb_arm_vision_interfaces/Jog` | a small straight step of the TCP (micro adjustment), see [Jog and go-to](#jog-and-go-to) |
| `/qb_arm_vision/go_to` | service `qb_arm_vision_interfaces/GoTo` | move to a named spot / pose of qb_arm `config/spots.yaml` |
| `/qb_arm_vision/save_home` | service `std_srvs/Trigger` | the arm's current pose becomes the home pose |
| `/qb_arm_vision/handover` | service `Handover` | `{action: give \| take, plan_only}`: hand the held object to the user's hand / take one from it |
| `/qb_arm_vision/stop` | service `std_srvs/Trigger` | stop a running handover where it is (cancels the trajectory controller's goals) |
| `/qb_arm_vision/close` | service `std_srvs/Trigger` | during a take: close the claw now |
| `/qb_arm_vision/hands` | subscription | the tracked hands (`hand_tracker`) |
| `/ufactory/joint_states` | subscription (during a give) | the arm's joint torques: a pull on the held object |
| `/kinect/depth/image_raw` + `camera_info` | subscription (during a take) | what is between the open fingers; the taken object's size |
| `/qb_arm_vision/objects` | subscription (latched) | the latest detection: objects by id |
| `/joint_states` | subscription | current `claw_joint` (to wait for the claw) |
| `/claw/command` | publisher | claw target angle |
| MoveIt | clients | `/compute_ik`, `/move_action`, `/compute_cartesian_path`, `/execute_trajectory`, `/get_planning_scene`, `/apply_planning_scene`, `/check_state_validity` |

**Before every pick** (also `plan_only`), on the real arm: the arm's controller state (`/ufactory/robot_states`) is
checked. An arm **error** (`err` ≠ 0) is cleared automatically (`auto_recover`, see *Automatic recovery* below; with it
off, or for an e-stop code, the request ends with the error code and a person recovers the arm). If the arm is
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
   height. **Grip check** (`check_grip`, on; never in sim): once the fingers have settled (within 0.01 rad over
   0.5 s, at most 2.5 s — a sponge gives way for ~2 s), the claw counts as **empty** only if the fingers went on to
   `grip_empty_angle` (1.045 rad) or further **and** the mean servo current (INA219, `/claw/servo_current`, over
   0.5 s) is at most `grip_empty_current` (540 mA). Then the claw opens and the pick fails with the numbers
   (*"Nothing grasped …: fingers stopped at 1.066 rad …, holding 431 mA -> empty"*); otherwise the stop position
   and current are logged with the grip. Without current data, position alone decides (with a warning).

   Measured on the real claw (2026-10-01, closing to 1.2 rad, 3 trials each, servo 41–44 °C):

   | | fingers stop | current passes 300 mA at | holding current | peak |
   |---|---|---|---|---|
   | empty | 1.066–1.072 rad | 1.068 rad (pads meeting) | 418–482 mA | 439–608 mA |
   | tape roll (wall) | 0.960–0.966 rad | 0.89–0.94 rad | 610–633 mA | 1.06–1.63 A |
   | thin cardboard | 1.005–1.027 rad | 0.87–0.99 rad | 592–712 mA | 756–877 mA |
   | sponge | 0.904–0.921 rad | 0.78–0.80 rad | 567–580 mA | 1.49–1.88 A |

   Holding current follows how far the fingers stop short of 1.2 rad (the servo pushes with the error), except on
   soft objects, which give way. Position alone gets tight on thin objects (0.04 rad), current alone on soft ones
   (574 vs ≤ 482 mA); together they separate all four. The peak (the impact) is too noisy to use.
5. **Attach** the object to `link_tcp` (touch links = the claw links). From now on MoveIt carries it with the arm.
   Its contact with the `table` is allowed while it is held: standing on the tilted table, its level bottom can
   touch the table box by a fraction of a millimetre, which made the lift and every later plan start "in collision".
6. **Lift**: straight line 10 cm up from the current TCP pose; at least 2 cm must be possible (edge of the reach).

### Failure handling

Any exception (MoveIt not answering, a rejected goal, a planning failure) ends the pick with `success=false` and the
message; the arm stays where it stopped. The one exception is an **arm fault** during a motion — next section.

### Automatic recovery

(2026-10-04, the user's choice: any error, the step continued as it was; the e-stop is the safety.) When the arm
itself faults during a motion (e.g. C31 collision, C22 self-collision, C24 speed, also servo errors like C16), the
driver deactivates the trajectory controller and MoveIt reports error −4. Every motion of the executor — pick, place,
home, jog, go-to, and the watched handover moves — then:

```mermaid
flowchart LR
    F[motion fails] --> E{arm error?}
    E -- no --> X[task fails]
    E -- "C1 / C2 e-stop, auto_recover off,<br/>stop pressed, retries used up" --> X
    E -- yes --> R["clear error + warning, motors on,<br/>servo mode, state ready"]
    R --> C[wait: controller active again]
    C --> O[refresh octomap]
    O --> P["re-plan the SAME step from where the arm stopped<br/>straight step: straight to the same pose<br/>free move: to the same joint values, same speed<br/>(pose in collision: unchecked / 3 cm up first)"]
    P --> M[execute]
    M -- fails again --> E
```

- No back-off and no slow-down: the step is re-planned from the current state at its own speed (a straight step —
  approach, lift, lower, retreat, jog, last stretch to the hand, back-off from the hand — stays straight).
- At most `auto_recover_retries` (3) recoveries per step; then the task bails out, the arm left in error.
- **Stuck in contact**: after a real collision the arm usually still touches what it hit, so MoveIt sees its pose as
  in collision and would refuse every plan. Then a straight step carries on **without the collision check**, and a
  free move first goes `auto_recover_escape` (3 cm) straight up, unchecked, and is re-planned from there. A joint past
  its limit is not handled (task fails).
- **Never** cleared automatically: `auto_recover_never` = C1 (e-stop button), C2 (emergency IO) — otherwise the arm
  would drive on by itself once the e-stop is released. The page's **stop** during a fault also ends the task.
- An error left from before is cleared the same way at the start of the next motion request.
- The result message lists the recoveries: `… (recovered automatically from C31 during approach)`; the log has one
  `arm error C… - clearing it automatically` warning per recovery.
- During a handover the hand watch stays on through the retry (hand moved / gone → the usual pause and re-plan).
- Not testable in sim (no arm errors there); logic checked offline with a stubbed arm.

## Place

`/qb_arm_vision/place` (`qb_arm_vision_interfaces/srv/Place`) puts the object held since the last successful pick down:

| `relation` | Where | Surface its bottom goes to |
|---|---|---|
| `""` | its centre at `position` (x, y in `world`; (0, 0) = back where it was picked) | the table plane |
| `"on"` | centred on `reference` (an object id from the latest detection, e.g. `white_bin`) | the reference's top (or the highest point measured under the object, if slightly higher) + 3 mm |
| `"into"` | into `reference` (cup, box, bin, …): the **emptiest** spot inside the opening where it fits (see [How full is a container?](#how-full-is-a-container)) | the reference's rim + 1 cm, then released |
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
- **into fit check**: the held object goes down in its pick orientation or turned 180°; its outline must stay inside
  the container's outline minus a 5 mm wall **in every direction** (72 directions checked). Drop spots lie on a 2 cm
  grid over the opening; all of them are checked, the 40 emptiest go on to planning. If the object fits nowhere the place
  is refused with how much too wide it is. After an "into" the object is removed from the scene (where it fell is unknown; the next
  detection sees it).
- **The opening claw is collision-checked**: the claw opens without a plan, so before a candidate is accepted MoveIt
  checks the final pose at claw angles from the gripped one down to fully open in 0.12 rad steps
  (`/check_state_validity` with the claw's group `qbag`: only the moving claw links; a whole-robot check also failed on
  unrelated contacts, e.g. a bin's outline touching the robot base). Part-way the fingers are already outside the object
  but still low. A candidate whose fingers would hit the reference, another object or the
  table is skipped.
- **next_to** uses the whole open claw's reach (pads 46 mm, finger knuckles 64 mm along the closing axis, palm 35 mm
  across) and refuses a reference that stands on something unmodelled (its support would be under the held object).
- **into, off centre**: a bin 44 cm from the base was out of reach at its centre (the Lite6 reaches ~40 cm at that
  height); spots nearer the robot were reachable. Unreachable spots cost little: the IK check rejects them in ~50 ms.
- After a place (not into) the object's new pose (`T_place · T_grasp⁻¹ · T_object`, down by the clearance) replaces its old
  one in the executor's detection — its grasps moved the same way — so it can serve as a reference or be picked again
  right away.
- Detect again while holding is fine: the detector leaves out the object in the claw (near the TCP and floating above
  the table) and never gives a new object the held one's id.

```mermaid
flowchart TB
    S(["place(relation, reference / x, y)"]) --> H{"holding an object<br/>from a pick?"}
    H -- no --> F1(["fail: pick first"])
    H -- yes --> G["TCP height above the object's bottom at the grasp:<br/>h = z_tcp(grasp) - bottom(object)"]
    G --> T["candidate targets: at (x, y) / on / into (grid of drop spots) /<br/>next to the reference (each side),<br/>same orientation or turned 180 deg"]
    T --> HM["fresh height map around all targets<br/>(object_detector surface_map, 7 depth frames)"]
    HM --> CK["per target and turn: free under the object and the open fingers?<br/>seen by the camera? into: room below the rim?"]
    CK -- none left --> F3(["fail: no free spot (reasons per spot)"])
    CK --> SO["sort: into = emptiest spot first"]
    SO --> C["for 10 cm / 5 cm above: IK check, plan there,<br/>straight way down (MoveIt carries the object),<br/>open claw at the end collision-free?"]
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

### Is the spot free? The height map

MoveIt only knows the objects of the **last detection**. Anything put down since, moved, or never asked for in the
prompt is invisible to it. So before planning, the executor asks the detector for a **height map** of the place: a grid
of 1 cm cells over the region around all candidate spots, each holding the height of whatever stands there right now.

```mermaid
flowchart LR
    F["7 depth frames<br/>(~0.25 s)"] --> MED["median per pixel<br/>(noise +-10 mm -> +-3 mm)"]
    MED --> FLT["drop steep surfaces<br/>(> 60 deg to the view: walls)"]
    FLT --> PTS["3D points in world<br/>(only the image part that sees the region)"]
    PTS --> ROB["leave out the robot:<br/>URDF collision boxes of every link<br/>+ the held object's outline as it hangs now<br/>(those cells are flagged 'robot')"]
    ROB --> CELL["per 1 cm cell: 75th percentile<br/>of its points' heights<br/>(< 3 points: not seen)"]
    CELL --> VEIL["cells just behind a taller edge<br/>(seen from the camera): not seen"]
    VEIL --> MAP["heights[iy, ix]<br/>NaN = not seen"]
```

For each candidate spot (and each of its two turns) the executor lays three footprints over the map, each grown by 1 cm:
the **held object's outline**, the **band the open fingers sweep** (±43 mm along the closing axis, 16 mm wide) and the
**claw body** (knuckles and cranks ±64 mm, palm ±35 mm, from 28 mm above the TCP):

- **Outside the boundaries**: no footprint may touch a cell outside the vision workspace or in a keep-out zone
  (*"outside the vision workspace or in a keep-out zone"*) — the camera doesn't look there.
- **Outside the map**: the footprints must lie completely inside the map (*"partly outside the checked region"*).
- **Occupied**: a seen cell is higher than what will be above it — the object's bottom under the object, the fingertips
  under the fingers, the claw body under the body — and at least 8 mm (`free_tolerance`) above the surface. Two such
  cells (`min_blocked_cells`) make the spot occupied: *"something 23 mm high at (0.215, 0.005) under the open fingers"*.
- **Could something be hidden there?** For every cell it can't see, the detector says how high something could stand
  there unseen (`hidden_top`): in the shadow of something taller — a bin, the arm itself — it is the line of sight over
  that occluder; where nothing explains the gap (no depth return from a dark or glossy surface, out of view, right under
  the arm) it is unknown. Only unseen cells where something hidden could reach what comes down there matter:
  - in a shadow, **two such cells** refuse the spot: *"something up to 220 mm high could hide behind a taller object or
    the arm at (0.285, 0.062)"*;
  - unknown ones and mixed-pixel veils are tolerated up to 40 % (`max_hidden`) **of each part** — the object's
    footprint, the fingers' band, the claw body — so a small object can't slip onto an unseen patch because the large
    claw body is well seen: *"70 % of the area under the object not seen by the camera"*.

  A cup's inside, hidden by its own walls, can hide nothing above the rim, so it never stops an "into" whose object is
  released above the rim; a tall item in a bin that hides something reaching above the rim does.
- **"on"**: the object comes to rest on the highest part of the reference's top under it; if the map measures that a
  little higher than the detection did (up to 8 mm), the set-down height follows the map.
- **"back where it was picked"** is not checked: the object hangs above that spot and hides it, and it was just
  picked from there.

Before any pick or place the executor clears MoveIt's octomap and lets the camera refill it (1.2 s, so objects that
are known now or have moved leave no ghost voxels), lets the detected objects touch the fixed robot base (`link_base`:
a bin outline reaching the base otherwise made every arm pose a collision), and checks that MoveIt's scene has the
table and every keep-out zone of
`boundaries.yaml` (*"MoveIt's planning scene lacks keepout_desk ...: not moving"*), plans picks with the claw
**open** (it opens before moving, whatever it was before), checks that the claw can open where the arm is (pick
start, `release`), and straight-line paths are cut at a joint jump of more than 0.5 rad between two 5 mm steps
(an IK branch switch would sweep the arm through unchecked space).

**Why so much filtering** — measured on the real cell (2026-09-30):

| Effect | Seen as | Fix |
|---|---|---|
| Depth noise on the dark table | one frame: cells up to 10 mm, 7 cells above 8 mm in a 16 × 16 cm patch | median of 7 frames: ±3 mm; 75th percentile per cell (instead of the maximum) |
| The Lite6's forearm lies up to 10 cm beside the line between its joint frames | the arm, parked above the bin, showed up as 41–56 cm "obstacles" | cut out every link's collision-mesh box (from `/robot_description`, + 3 cm) |
| **Mixed pixels past an edge** ("veil"): a depth pixel that sees part rim and part table reads a depth in between | a 10–25 mm ramp beside the bin's rim, on the side away from the camera | cells just behind a ≥ 3 cm taller edge, seen from the camera, within the edge's shadow length + 4 cm, count as not seen |

### How full is a container?

"Is the centre hole at least as deep as the rim is high?" would give one yes/no for the whole bin. The height map allows
something better, because the ceiling camera looks **into** the container:

```mermaid
flowchart TB
    IN["cells inside the container's outline<br/>minus 1.5 cm (walls, rim)"] --> FILL["fill = mean over the seen cells of<br/>(height - floor) / (rim - floor)<br/>floor = bottom + 5 mm"]
    SPOT["each drop spot where the object fits<br/>(outline inside the opening)"] --> PILE["contents under the object:<br/>95th percentile of those cells"]
    PILE --> ROOM{"rim - contents<br/>>= min(object height, rim - floor)<br/>- 8 mm ?"}
    ROOM -- no --> FULL["full there: skip"]
    ROOM -- yes --> KEY["candidate, sorted by<br/>contents height (whole cm)"]
    KEY --> TRY["try the emptiest first,<br/>then nearest to the centre"]
```

- A spot is usable when the object, resting on the contents under it, fits **completely below the rim** (8 mm
  tolerance). An object taller than the container only goes onto an (almost) empty part of the floor.
- **Cups and deep boxes**: the camera can't see their inside at all (walls). Such spots are still allowed — dropped
  above the rim — if nothing unseen under the object could reach above the rim, but only after every spot whose
  contents were seen; the reply says *"its contents not seen"*.
- The container is "full" **for this object** when no spot is usable: a pen may still fit where a bottle doesn't. The
  reply then lists the spots and why, e.g. *"full there: contents 1.9 cm high leave 7.6 cm below the rim, it needs
  8.7 cm"*.
- The claw drops where the container is **emptiest**, so it fills evenly instead of piling up in the middle.
- Every reply for "into" reports the fill: *"white_bin 7% full, 35% of the inside seen"*.

**Limits**: the camera looks in at ~20°, so the strip along the wall nearest to the camera is hidden (about 0.4 × the
wall height, plus the veil band): spots there count as not seen. Very dark or transparent contents return little or no
depth (not seen), a shiny white bin's floor reads ~1 cm too low (light bouncing between the walls). With the arm
parked right above the container most of the inside is hidden — the map is taken from wherever the arm is when place
is called.

| Parameter | Default | Meaning |
|---|---|---|
| `place_clearance` | 0.003 m | object bottom above the table when the claw opens |
| `place_distances` | `[0.10, 0.05]` | m, above the place: the straight way down starts here |
| `retreat_distance` | 0.10 m | straight up afterwards |
| `home_after_place` | true | then to the home pose (`home_file`: qb_arm `config/home.yaml`) |
| `jog_velocity_scaling` | 0.05 | jog steps (a motion parameter, at most 0.3) |
| `jog_max_step`, `jog_max_angle` | 0.05 m, 15° | largest jog per axis and request |
| `spots_file` | qb_arm `config/spots.yaml` | named spots and poses for go-to |
| `into_clearance`, `into_margin` | 0.01 m, 0.01 m | release height above a container's rim; both walls together |
| `into_step`, `into_max_spots` | 0.02 m, 40 | grid of drop spots over the opening; all are checked, this many (emptiest first) go on to planning |
| `next_to_gap` | 0.02 m | default gap between the outlines |
| `check_place` | true | check the spots in a fresh height map first |
| `free_tolerance` | 0.008 m | higher above the surface under the object / fingers = something there |
| `free_margin` | 0.01 m | around the object's outline and the fingers' band |
| `min_blocked_cells` | 2 | 1 cm cells that must be too high before a spot is occupied |
| `max_hidden` | 0.4 | a spot with more of its area not seen is refused |
| `container_wall`, `container_floor` | 0.015 m, 0.005 m | inside = outline minus this; floor above the container's bottom |
| `max_survey_half_size` | 0.5 m | largest height map (± this), as object_detector's `max_map_half_size` |

The place-check parameters are read at every place, so `ros2 param set /qb_arm_vision/pick_executor check_place false`
(or any of the others) applies to the next one.
| `table_plane` | from qb_arm `config/table.yaml` | measured table plane |

First real run (2026-09-30): tape roll picked 40 cm from the base, placed at (0.25, 0.10); the camera found it at
(0.252, 0.112) afterwards.

## Release

`/qb_arm_vision/release` opens the claw (waits for it), then, if an object is attached, **detaches** it and
**removes** it from the planning scene. The object falls from wherever the claw is; `place` is the controlled way down.

While an object is attached, poses where it would collide are invalid, including the start of any plan that begins
with the object inside the table or the robot; release before planning elsewhere.

## Jog and go-to

The helping-hand basics: hold something where you want it, and nudge it.

**Jog** (`/qb_arm_vision/jog`): `translation` (m, world frame) and `rotation` (rad, about the world x / y / z axes
through the TCP: roll, pitch, yaw), each at most `jog_max_step` (5 cm) / `jog_max_angle` (15°) per axis. One
straight Cartesian path from the current TCP pose (IK every 5 mm, cut at a joint jump), at
`jog_velocity_scaling` (0.05, at most 0.3; Config → motion), carrying whatever the claw holds. **Not checked for
collisions** (since 2026-10-04, the user's choice: micro adjustments must be able to nudge the claw against objects,
the table or into tight spots) — objects, octomap, table and the arm itself are all ignored; contact is caught only by
the arm's own collision detection (C31, then [automatic recovery](#automatic-recovery)). All or nothing: a step that
can't be done completely (reach, joint limits) is refused with how far it would get.

**Go-to** (`/qb_arm_vision/go_to`): the spot or pose `name` (case-insensitive) from qb_arm's `config/spots.yaml`
(launch argument `spots_file`, read at every call): a **spot** is a position — the TCP goes there and the claw keeps
its current orientation; a **pose** is a position and an orientation (roll / pitch / yaw in degrees, extrinsic x-y-z in
the world frame; claw straight down = roll 180, pitch 0). IK check first (*"out of reach"*), octomap refresh, a
collision-free plan at the travel speed. Reached within ~1 mm / 0.3° in sim.

Both, like pick / place / home: one motion at a time, servo mode and an active trajectory controller before a real
motion, `plan_only` to only plan.

## Home

The home pose is a set of joint values in qb_arm's `config/home.yaml` (launch argument `home_file`), read at every
move home and checked against the joint limits. Default: the ready pose, TCP at (0.200, 0.000, 0.088), claw pointing
down. After every successful place and retreat the arm goes home (`home_after_place`, default true, live); if no
collision-free way home is found the place still succeeds and the reply says *"not home: …"*. The ±2π joints (1, 4, 6)
go to the equivalent angle nearest to where they are, so the arm doesn't unwind a full turn to get home — but always at
least 5° inside their limits (`LIMIT_MARGIN`): aimed at exactly +360°, joint 4 stopped there with C23 (*joint angle
exceed limit*). A joint up to 0.5° past its limit (`JOINT_LIMIT_SLACK`: where such a stop leaves it, or rounding) is
planned from the limit; further out, home / go-to / jog say which joint and by how much instead of "no collision-free
way".

`/qb_arm_vision/save_home` (control page: **save pose as home**) writes the arm's current pose to `home.yaml`: jog the
arm to where it should rest (out of the camera's way), then save.
A save is refused while the arm's joint states aren't in yet (right after a cell start they are all exactly 0) and for
any pose MoveIt finds in collision; a home pose in collision is reported (*"The home pose … is in collision …: save a
new one"*) instead of planned.

## Handover: give and take

The helping hand passes objects to the user's hand (**give**) and takes them from it (**take**, *hold this*). The hands
come from the camera: `hand_tracker` publishes every tracked hand on `/qb_arm_vision/hands` (21 landmarks in the
world frame, ~10 Hz, ~0.1 s late). Both directions share the way to the hand; source `pick_executor.py` (`give`,
`take`, `approach_hand`, `plan_approach`), the geometry in `handover.py` and `claw.py`.

```mermaid
sequenceDiagram
    participant U as User
    participant C as Camera (hands)
    participant E as pick_executor
    participant A as Arm + claw
    U->>C: holds a hand still in front of the arm
    E->>C: a still hand in the handover zone? (nearest to the robot)
    E->>E: leave the hand out of the octomap, find a pose 10 cm in front of the palm, plan
    E->>A: move there (last 10 cm straight), watched every 30 ms
    alt give
        U->>A: hand at the object for 1 s, or a pull on it
        E->>A: open, back off, home
    else take
        U->>A: puts the object between the open fingers, hand clear of them
        E->>A: close after 0.5 s (or "close now"), measure the object, hold it right there
    end
```

**Finding the hand.** A hand held still (under 10 cm/s for 0.5 s, within `handover_find_time`, 15 s) in the
**handover zone**: 34–75 cm from the base axis, −3 to 60 cm high, not over a keep-out zone (the desk with the mouse
hand). With several, the one nearest to the robot.

**Where the claw goes.** The held object's centre (give) or the open claw's centre (take) goes `handover_standoff`
(10 cm) in front of the palm, towards the robot — at most `handover_max_reach` (50 cm) and at least 30 cm from the
base, at the palm's height (above the table by the object's size). Tried in order: the claw's tilt
(`handover_pitch` 34° below the horizontal, then 52°, 69°, 90°, 17°, level), five distances, two wrist sides, the last
`handover_approach` (10 cm) straight along the claw or from above. Only **natural arm configurations** (the shoulder
not leaning back more than 10°, the forearm roll within ±120°): MoveIt's model found turned-over poses near the base
collision-free, the arm's controller stopped them with C22 (self-collision). The first leg goes to exactly the checked
joint values; the straight leg is planned from where the first one really ends.

**The hand in the octomap.** Without care, the user's own hand blocks every pose near it. Before planning, the hand,
the spots in front of it and the held object are published as zones that the obstacle cloud leaves out; 0.8 s later
(`octomap_mask_delay`: camera frames already on their way still contain the hand) the octomap is cleared and rebuilt.

**Watched motion.** Every 30 ms during the approach:

| Seen | Then |
|---|---|
| any hand within 5 cm of the arm (`link_base`…`link6` as 4 cm capsules) | stop |
| the stop service (control page **stop**) | stop |
| the hand moved 8 cm, left, or no hand data for 0.5 s | stop; pause until a hand is still again and re-plan towards it (at most 3 times) — unless the hand is already at the object (give) / at the claw (take): then carry on from where the arm stopped |

A stop cancels the trajectory controller's goals directly (MoveIt's own cancel did not stop the execution): the arm
stands within ~0.1 s, where it is.

### Give

**Release** when a hand has been within `handover_release_distance` (5 cm) of the held object's box for
`handover_release_delay` (1 s) — or on a **pull**: the arm's joint torques (joints 2–6, `/ufactory/joint_states`,
150 Hz) off their resting values by `handover_pull_torque` (0.8 Nm) for `handover_pull_time` (0.15 s). Measured: noise
~0.01 Nm, gentle pulls 1.8–3.2 Nm; the pull is the backup when the claw hides the fingers from the camera. Then: open,
detach, back off along the approach, home. Nobody takes it within `handover_wait` (30 s): back off, home, still holding.

### Take (hold this)

The claw opens, comes to the hand open and waits. It **closes** when, for `take_close_delay` (0.5 s) without a break:

- the depth image shows **something between the open fingers**: at least `take_min_points` (25) points in a box between
  the pads (link_tcp frame: x ±2 cm, y ±2.4 cm, z −1.2…+3 cm), the claw's own parts left out, in 3 checks in a row; and
- **every hand the camera sees is clear of the claw**: all landmarks at least `take_hand_clear` (4 cm) from where the
  closing claw can pinch (x ±1 cm, y ±4.6 cm, z −3…+3.1 cm). A hand the camera doesn't see does not count as clear.

— or on **close now** (`/qb_arm_vision/close`). So: hold the object by its far end and keep holding it until the claw
closes. The grip check is the pick's: closed on nothing → open again and wait on. Then the arm **holds it right there**
— *hold this* is for working on it (soldering): no retract, no move home; jog it where you need it, home or give it
back when done. Nothing put in within `handover_wait`: the claw stays open there.

The **object is measured** once your hand has left it (every seen hand `take_hand_clear` + 2 cm from where the claw
pinches for 0.5 s, at most `take_measure_wait`, 3 s — a hand at the object is left out of the depth points together
with what it holds), over 3 new depth frames: the points connected (5 mm voxels) to what is on the grip line — between
the pads, or where a thin object sticks out of them — leaving out the claw's own parts (its fingers and linkage from the
meshes, at the claw's angle, +8 mm), the table, anything within 3.5 cm of a hand and every pixel at a depth jump (the
Kinect's "flying pixels" around the claw's edges made an empty claw look like a 4 × 10 × 7 cm object) give a box in
the claw's frame (+8 mm: the camera sees one side). Not seen: a 4 cm box — the reply says so. The box is added to
MoveIt and attached as `handed_N`: later moves (jog, give, place) are checked with it. Like a picked target, the claw
may touch it — set down, the opened fingers stand inside its box (without that the retreat after a place, and every
move after it, failed as "in collision (claw_left_finger–handed_1)").

A taken object is held like a picked one: **give** hands it back (its centre, not the TCP, 10 cm in front of the
palm); **place** sets it down as if it had been picked from above (the claw straight down, the box upright under it) —
"back where it was picked" is refused, it has no such place. Nothing put in within `handover_wait`: back off, home,
claw open.

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `group` | `lite6` | MoveIt planning group |
| `tcp_link` | `link_tcp` | link the grasps refer to |
| `pregrasp_distances` | `[0.10, 0.05]` | m, tried in order |
| `lift_distance` | 0.10 | m |
| `min_lift_distance` | 0.02 | m, a shorter lift is accepted |
| `velocity_scaling` / `acceleration_scaling` | 0.2 / 0.2 | of the joint limits: free motion (pre-grasp, place, home) / every motion |
| `approach_velocity_scaling` | 0.05 | straight approach to the grasp and way down to the place |
| `lift_velocity_scaling` | 0.2 | straight lift after gripping, retreat after releasing |
| `pause_before_grip`, `pause_after_grip` | 0, 0 | s at the grasp pose before closing; after closing, before the lift |
| `pause_before_release`, `pause_after_release` | 0, 0 | s at the place before opening; after opening, before the retreat |

The eight motion parameters above are read at every motion (`ros2 param set` works live; ranges 0.01–1 and 0–10 s are enforced) and set from qb_arm's `config/motion.yaml` (launch argument `motion_file`), which the control page's Config → motion writes.

| Parameter | Default | Meaning |
|---|---|---|
| `planning_time` | 5.0 | s per plan |
| `grip_target` | 1.2 | rad, claw command when gripping (past fully closed; clamped to 1.2) |
| `check_grip` | true | after closing, fail the pick (claw opened) if it looks empty (never in sim) |
| `grip_empty_angle`, `grip_empty_current` | 1.045 rad, 0.54 A | empty = the fingers at this angle or further **and** holding current at most this |
| `grip_settle_timeout`, `grip_current_window` | 2.5 s, 0.5 s | wait for the fingers to settle; current averaged over the window |
| `claw_current_topic` | `/claw/servo_current` | the claw's INA219 servo current (A) |
| `claw_joint`, `claw_links` | | names in the URDF |

`grip_target` and the grip-check parameters are read at every pick: `ros2 param set /qb_arm_vision/pick_executor ...`
tunes them without a restart.

Handover parameters, read at every handover (`ros2 param set` applies to the next one; ranges enforced):

| Parameter | Default | Meaning |
|---|---|---|
| `handover_velocity_scaling` | 0.15 | of the joint velocity limits: the moves towards the hand and back (at most 0.3) |
| `handover_approach_velocity_scaling` | 0.04 | the last straight bit to the hand and the retreat from it |
| `handover_standoff` | 0.10 | m: the object (give) / the open claw (take) stops this far in front of the palm |
| `handover_approach` | 0.10 | m of straight line at the end of the move to the hand |
| `handover_pitch` | 0.6 | rad: the claw's tilt below the horizontal tried first |
| `handover_max_reach` | 0.50 | m from the base axis |
| `handover_release_distance`, `handover_release_delay` | 0.05, 1.0 | give: a hand this close to the object's box this long → open |
| `handover_pull_torque`, `handover_pull_time` | 0.8, 0.15 | give: a pull of this many Nm (joints 2–6) this long → open (0 = off) |
| `handover_wait`, `handover_find_time` | 30, 15 | s at the hand; s to find a still hand |
| `handover_stop_distance`, `handover_hand_moved` | 0.05, 0.08 | m: a hand this close to the arm → stop; the target hand moved this far → pause |
| `take_hand_clear` | 0.04 | m: every seen hand this far from where the claw pinches = clear |
| `take_close_delay`, `take_min_points` | 0.5, 25 | take: object seen + hands clear this long → close; depth points that count as an object |
| `take_measure_wait` | 3.0 | s: after closing, at most this long for your hand to leave the object before it is measured |
| `octomap_mask_delay` | 0.8 | s between leaving the hand out of the octomap and clearing it |

## Example session

```bash
cell start real
ros2 service call /qb_arm_vision/detect qb_arm_vision_interfaces/srv/Detect "{prompt: 'tape roll.'}"
ros2 service call /qb_arm_vision/pick qb_arm_vision_interfaces/srv/Pick "{object_id: tape, plan_only: true}"
ros2 service call /qb_arm_vision/pick qb_arm_vision_interfaces/srv/Pick "{object_id: tape}"
ros2 service call /qb_arm_vision/place qb_arm_vision_interfaces/srv/Place "{position: {x: 0.25, y: 0.10}}"
# or drop it where it is:
ros2 service call /qb_arm_vision/release std_srvs/srv/Trigger
ros2 service call /qb_arm_vision/home qb_arm_vision_interfaces/srv/Home "{plan_only: true}"
```
