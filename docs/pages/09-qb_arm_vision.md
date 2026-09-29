# Module: qb_arm_vision

Perception and picking. Repository `whoobee/qb_arm_vision`:

```
qb_arm_vision/
├── server/                       GPU inference server (Docker, runs on hbh-ai)
│   ├── app/server.py             FastAPI: /pipeline, /health
│   ├── Dockerfile, compose.yaml, requirements.txt
├── qb_arm_vision/                ROS 2 package (ament_python)
│   ├── qb_arm_vision/object_detector.py
│   ├── qb_arm_vision/pick_executor.py
│   ├── qb_arm_vision/claw.py     claw geometry shared by both nodes
│   ├── config/object_detector.yaml, pick_executor.yaml
│   └── launch/object_detector.launch.py   (both nodes, namespace qb_arm_vision)
└── qb_arm_vision_interfaces/     messages and services (ament_cmake)
```

The algorithms are described on [perception pipeline](05-perception-pipeline.md) and
[pick execution](06-pick-execution.md); this page documents the module structure, interfaces and deployment.

## Class view

```mermaid
classDiagram
    class ObjectDetector {
        <<ROS node>>
        -server_url
        -thresholds and grasp parameters
        +on_detect(request) response
        -snapshot() images_K_pose
        -make_object(det, depth, K, T) Object
        -claw_grasps(cgn_grasps, T) list
        -ring_grasps(world, top, height) list
        -top_down_grasp(world, top) Grasp
        -closing_drop(width) float
        -publish_markers(objects)
        -publish_debug_image(img, K, T, results)
        -update_scene(objects)
    }
    class PickExecutor {
        <<ROS node>>
        -objects
        -claw_position
        +on_pick(request) response
        +on_release(request) response
        -pick(request) result
        -reachable(pose) bool
        -plan_to(pose) trajectory
        -straight_line(target, start) trajectory
        -run(trajectory)
        -move_claw(angle)
        -allow_claw_contact(object_id)
        -attach(object_id)
    }
    class ClawGeometry {
        <<module claw.py>>
        CRANK_Y = -0.0346
        CRANK_Z = 0.0225
        OPEN_GAP = 0.070
        CLOSED = 0.96
        TIP_OPEN = 0.012
        DROP_CLOSED = 0.0188
        +claw_gap(angle) float
        +claw_angle(gap) float
        +finger_drop(angle) float
    }
    class GpuServer {
        <<FastAPI on hbh-ai port 8770>>
        +pipeline(rgb, depth, K, prompt) detections
        +health() status
        -detect(image, prompt) boxes
        -segment(image, boxes) masks
        -non_max_suppression(boxes, scores) keep
    }
    class Object {
        id
        label
        score
        pose
        size
        shape
        grasps
    }
    class Grasp {
        pose
        score
        width
    }
    ObjectDetector ..> GpuServer : HTTP
    ObjectDetector ..> ClawGeometry
    PickExecutor ..> ClawGeometry
    ObjectDetector --> Object : produces
    Object *-- Grasp
    PickExecutor ..> Object : consumes
```

## Interfaces (`qb_arm_vision_interfaces`)

```
# msg/Grasp.msg — a grasp for the claw: target pose of link_tcp, fingers closing along its y axis
geometry_msgs/PoseStamped pose
float32 score        # 0..1 (Contact-GraspNet confidence, or the geometric grasps' fixed scores)
float32 width        # m, opening needed

# msg/Object.msg — an object found on the table
string id                  # collision object id in MoveIt's planning scene, e.g. "obj_3"
string label               # detector label, e.g. "tape roll"
float32 score              # detector confidence
geometry_msgs/PoseStamped pose   # centre of the bounding box, z up, x along the long side
geometry_msgs/Vector3 size       # long side, short side, height (m)
shape_msgs/Mesh shape            # top-view outline extruded to the table, relative to pose
Grasp[] grasps             # best first

# msg/ObjectArray.msg — latched on /qb_arm_vision/objects
std_msgs/Header header
string prompt
Object[] objects

# srv/Detect.srv
string prompt        # "tape roll. bottle. cup." ; empty = the node's default
---
bool success
string message
Object[] objects

# srv/Pick.srv
string object_id     # from the latest detection
bool plan_only       # only plan (shown in RViz), don't move
---
bool success
string message
Grasp grasp          # the grasp used
```

## `claw.py`

The claw's parallelogram geometry, shared by the detector (grasp heights) and the executor (closing angle), so they
can't disagree. Values from `qb_arm/urdf/qbag.xacro`: the crank vector from the crank pivot to the finger pivot is
(y, z) = (−34.645 mm, 22.497 mm).

| Function | Formula / method |
|---|---|
| `claw_gap(a)` | `0.070 − 2·(CY·cos a + CZ·sin a − CY)` — gap between the pads at `claw_joint = a` |
| `claw_angle(gap)` | inverse of `claw_gap` by bisection over 0..0.96 (40 iterations; the gap shrinks monotonically) |
| `finger_drop(a)` | `−CY·sin a + CZ·(cos a − 1)` — how much further along the approach the fingers are than when open |
| `DROP_CLOSED` | `finger_drop(0.96)` = 18.8 mm |

## `object_detector` parameters (`config/object_detector.yaml`)

| Parameter | Default | Meaning |
|---|---|---|
| `server_url` | `http://hbh-ai.local:8770` | GPU server |
| `prompt` | `bottle. cup. box. tape roll. tool. can. container. toy.` | used when the request's prompt is empty |
| `box_threshold`, `text_threshold` | 0.3, 0.25 | Grounding DINO thresholds |
| `max_box_fraction` | 0.25 | ignore boxes covering more of the image |
| `max_tilt_deg` | 30 | network grasps: max angle between approach and straight down |
| `max_grasp_width` | 0.065 m | the claw opens 70 mm |
| `gripper_depth` | 0.1034 m | Panda hand frame → finger contacts |
| `grasps_per_object` | 5 | |
| `table_z` | 0.0 m | table surface in `world` |
| `min_object_height`, `max_object_height` | 0.005, 0.4 m | above the table plane |
| `workspace_radius` | 0.8 m | around the robot base |
| `reach` | 0.44 m | only for the "out of reach" hint in the debug image |
| `edge_threshold` | 0.02 m | flying-pixel filter |
| `fingertip_clearance` | 0.003 m | fully closed claw's fingertips above the table plane |
| `table_plane` | from qb_arm `config/table.yaml` | measured table plane a, b, c (`z = a·x + b·y + c`) |
| `slice_grasp_score`, `slice_step` | 0.2, 0.01 m | narrow-slice grasps (flat/long objects) |
| `fallback_grasp_score`, `top_slice` | 0.1, 0.03 m | top-slice fallback grasp |
| `max_grasp_depth` | 0.025 m | network grasps: max TCP depth below the object top |
| `finger_margin`, `max_finger_hits` | 0.01 m, 5 | finger-landing check |
| `ring_grasp_score`, `ring_grasp_directions`, `min_ring_hole_radius` | 0.3, 12, 0.02 m | ring grasps |
| `add_collision_objects` | true | put detected objects into the planning scene |
| `timeout` | 60 s | server request |

`pick_executor` parameters: see [pick execution](06-pick-execution.md#parameters).

**Parameter files** are keyed `/**/object_detector:` and `/**/pick_executor:`: the nodes run in the namespace
`/qb_arm_vision`, and a plain `object_detector:` key matches nothing (until 2026-09-29 the files were silently ignored).
The config files are copied at build time (`colcon build`), not symlinked.

## The GPU server

| | |
|---|---|
| Host | hbh-ai (`192.168.1.220`), user `whoobee`, docker group (no sudo) |
| Code | `~/prj/qb_arm_vision/server` (a copy of the repo's `server/`), `app/` mounted into the container for live edits |
| Container | `qb_arm_vision`, `restart: unless-stopped`, port 8770 (8765 is taken by speech-to-speech) |
| GPU | device 0, RTX 3060 12 GB (~9.7 GB in use incl. other services); the 3060 Ti runs speech-to-speech |
| Image | `pytorch/pytorch:2.8.0-cuda12.8-cudnn9-runtime` + transformers, FastAPI, OpenCV; Contact-GraspNet PyTorch port (elchun, pinned commit) |
| Models | `IDEA-Research/grounding-dino-base`, `facebook/sam2.1-hiera-base-plus`, Contact-GraspNet (NVIDIA, non-commercial licence) |
| Model cache | `~/.cache/qb_arm_vision` → `/models` (Hugging Face downloads survive rebuilds) |
| Concurrency | one request at a time on the GPU (a lock) |

Operations:

```bash
ssh hbh-ai.local
cd ~/prj/qb_arm_vision/server
docker compose up -d --build      # (re)build and start
docker compose logs -f            # logs
curl http://hbh-ai.local:8770/health
```

Contact-GraspNet's checkpoint pickles NumPy scalars, which PyTorch ≥ 2.6 refuses by default, so the server loads it with
`weights_only=False` (the checkpoint comes with the pinned upstream commit).

### `POST /pipeline` — request and response

Request (multipart form): `rgb` (JPEG/PNG), `depth` (16-bit PNG, mm, registered to rgb), `K` (JSON 3x3), `prompt`,
`box_threshold`, `text_threshold`, `max_box_fraction`, `z_min` (0.2), `z_max` (2.0), `forward_passes` (1), `grasps`
(true).

Response:

```json
{
  "frame": "camera_optical",
  "detections": [
    {"id": 1, "label": "tape roll", "score": 0.44, "box": [x0, y0, x1, y1], "pixels": 5210,
     "mask_png": "<base64 PNG>",
     "grasps": [{"T": [[...4x4...]], "score": 0.19, "width": 0.009, "contact": [x, y, z]}]}
  ],
  "timing": {"detect": 0.43, "segment": 0.62, "grasps": 3.1}
}
```

`timing` values are cumulative seconds since the request started.
