# Module: camera calibration

Every 3D position the perception pipeline computes is only as good as the **camera's pose in `world`** (the
*extrinsic calibration*): an error of 1° at 1.5 m distance moves everything on the table by 2.6 cm. This page
describes how the pose is represented and the three tools that measured it.

Result in use (2026-10-02, camera moved along the wall towards the user): `x 0.414, y 0.561, z 1.471,
yaw −1.7153 rad`, tilt 19.3° from straight down; ICP 4.4 mm RMS against the robot model, then corrected
(roll 0.18°, pitch 0.25°, z +5.9 mm) so that the camera's table matches three claw touches (see *Touch check* below).
Before (2026-09-27): `x −0.380, y 0.571, z 1.475, yaw −0.6515 rad`, 4.0 mm RMS.

**Why the camera moved (2026-10-02).** An offline occlusion study (arm + claw model rendered into a virtual Kinect,
14 arm poses plus the user's working pose; validated against a real depth frame: silhouette IoU 0.64, depth axis and
position matching the driver within 0.0° / 2 mm) compared positions along the wall (y ≈ 0.57, z ≈ 1.5):

| Camera x | Table visible | Under the claw at pre-grasp | User's hand zone | Around the claw, user's pose |
|---|---|---|---|---|
| −0.38 (old) | 88 % | 86 % | 79 % (95 % re-aimed) | 80 % |
| 0.00 | 89 % | 55 % | 97 % | 88 % |
| 0.20 | 89 % | 41 % | 98 % | 90 % |
| 0.40 | 88 % | 70 % | 99 % | 92 % |
| 0.50 | 87 % | 86 % | 99 % | 92 % |

Positions beside the work area are the worst for seeing under a top-down claw; from either end the camera looks in at
a slant. Lowering the camera (to ~1.0 m) helps under the claw but costs the hand zone and raises the incidence angle
on the dark table past 45°.

## Representation (`config/camera_pose.yaml`, `qb_arm/camera_pose.py`)

The pose of `camera_base` in `world` is stored as

| Key | Meaning | Source |
|---|---|---|
| `x`, `y`, `z` | position (m) | tape measure, then fitted |
| `up_in_camera` | the world's "up" direction expressed in camera coordinates (unit vector) | the camera's accelerometer |
| `yaw` | rotation about the world vertical | fitted against the robot |

Why this split: an accelerometer at rest measures gravity, so it tells the tilt (2 degrees of freedom) exactly, but
it cannot see the rotation *about* the vertical (yaw). Storing tilt and yaw separately lets each come from the right
source.

`camera_quaternion(up_in_camera, yaw)` builds the orientation in two steps:

1. **Tilt**: the shortest rotation that maps `up_in_camera` onto world +Z:
   axis = `up × Z`, angle = `acos(up · Z)` (handling the parallel/antiparallel cases).
2. **Yaw**: then rotate about world +Z by `yaw` — which leaves the tilt unchanged.

`q = q_yaw ⊗ q_tilt`. `kinect.launch.py` publishes it as the static transform `world → camera_base`.

## Procedure

```mermaid
flowchart LR
    M["Measure x, y, z<br/>by hand"] --> T["measure_camera_tilt<br/>IMU -> up_in_camera"]
    T --> Y["fit_camera_yaw<br/>coarse: capsule robot model,<br/>full yaw search"]
    Y --> R["refine_camera_pose<br/>ICP against the Lite6 meshes<br/>4-DOF (or 6-DOF --full)"]
    R --> A["--apply writes camera_pose.yaml<br/>restart the cell"]
```

All three tools are read-only with respect to the robot (they never move the arm) and write the YAML only with
`--apply` (the tilt tool unless `--dry-run`). `config_path()` resolves the installed YAML back to the source file
(symlink install), so results land in the repository.

### `measure_camera_tilt`

Averages `/kinect/imu` linear acceleration for 3 s (camera still), rotates the mean vector from the IMU frame into
`camera_base` (via TF), normalises it → `up_in_camera`. At rest the accelerometer reads +g **upwards** (it measures the
reaction to gravity). Prints |g| (sanity check ≈ 9.8) and the tilt of the lens axis from straight down (19.3° here).

### `fit_camera_yaw` — coarse, global

The yaw is unknown and can be anything, so a local method (ICP) would get stuck. This tool does an exhaustive search
against a crude robot model:

1. **Robot as capsules**: the live TF positions of `link_base … link_eef` form a chain; each consecutive pair is a
   capsule of radius 6 cm, plus a 12 cm column for the base.
2. Take one point cloud, 150 000 random points, transformed into a "tilt-only" frame (`camera_base` rotated by the
   known tilt, yaw 0).
3. **Score** of a candidate (yaw, dx, dy) = number of points (above 3 cm) within a capsule after rotating by yaw and
   translating to (x + dx, y + dy, z).
4. Search: yaw from −180° to 180° in 2° steps → best ± 3° in 0.25° steps → joint refinement of yaw/x/y by
   coordinate descent at 2, 1, 0.5 cm steps.
5. Reports the best yaw and the best *distinct* runner-up (> 20° away) to judge ambiguity.

### `refine_camera_pose` — fine, local (ICP)

**Iterative Closest Point** against the real robot surface:

1. **Model**: the Lite6 visual meshes (`link_base … link6`) sampled uniformly by area (one point per ~4 mm², from the
   binary STL triangles), placed at the live joint state → a KD-tree.
2. **Data**: 5 accumulated point clouds, in `camera_base`, kept only where they could be the robot (z > 2 cm, within
   0.6 m of the model's centre).
3. **Loop**, with a shrinking inlier distance 6 → 4 → 3 → 2 → 1.5 → 1.2 → 1 cm, 8 iterations each:
   - transform the data with the current pose, find each point's nearest model point (KD-tree);
   - keep pairs closer than the threshold (inliers);
   - solve the best rigid transform between inliers and their partners:
     - **4-DOF** (default): rotation about z only, closed form:
       `a = atan2(Σ(sx·dy − sy·dx), Σ(sx·dx + sy·dy))` on centred coordinates;
     - **6-DOF** (`--full`): Kabsch/SVD, `R = V · diag(1, 1, det) · Uᵀ`;
     - translation `t = mean(d) − R · mean(s)`;
   - compose it onto the pose.
4. Convert back to YAML form: `up = Rᵀ · Z`, yaw from the remaining z-rotation.
5. Report inliers within 2 cm and RMS before/after.

The 6-DOF run confirmed the IMU tilt within 0.2°, so the default keeps the tilt fixed (fewer parameters, more
robust). Note: the user's first hand measurement had x and y swapped (robot X points to the front).

### `measure_table` — the table plane

Averages 10 depth frames (median per pixel), deprojects them into `world`, keeps points 12–55 cm from the base within
3 cm of z = 0, and fits `z = a·x + b·y + c` by robust least squares (four rounds, dropping points more than 3× the
median residual off). Result (2026-09-29): `a −0.01468, b −0.00354, c −0.00170`, 1.7 mm RMS: the table looked tilted 0.87°
against the robot base — which turned out to be a camera calibration error (see the touch check). `--apply` writes `config/table.yaml` (keyed `/**`), which `planning_scene_setup` (MoveIt's table
box, tilted to the plane) and the object detector (heights, fingertip clearance) read.

### Touch check — the table from the arm itself (2026-10-02)

The camera-based table depends on the camera calibration, and the ICP against the arm cannot separate a small tilt
from a few cm of position when the arm is in one pose. Ground truth: jog the closed claw (pointing straight down)
until the fingertips just touch the table, at three spots, and compute the lowest claw point from the joint states
and the claw model:

| Spot | Fingertip z (touch) | Old camera | New camera before correction |
|---|---|---|---|
| (0.18, 0.00) | +1.4 mm | −4.4 mm | −3.7 mm |
| (0.41, 0.00) | +1.2 mm | −7.8 mm | −2.8 mm |
| (0.19, 0.30) | +0.6 mm | −5.6 mm | −5.4 mm |

The table is flat against the base within 0.16° (`table.yaml`: `a −0.00076, b −0.00273, c +0.00155`). Both camera
calibrations put it 4–9 mm too low; the camera pose was rotated/raised so its table matches the touches
(`measure_table` afterwards: within ~1.5 mm). The arm-mesh ICP alone still prefers the camera ~6–8 mm lower — most
likely a time-of-flight bias between the shiny arm and the dark table; for picks the table is the reference that counts.

### Limits

- The camera sees the robot mostly from above; poses where the arm spans a large volume give the best fits.
- An attempt to fit the **claw's mounting yaw** the same way (claw meshes vs. cloud) was not usable: too few points
  (link6 hides the claw from above) and unmodelled parts (ESP32, cables). The claw yaw was confirmed visually in RViz.
