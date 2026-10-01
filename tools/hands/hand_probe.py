#!/usr/bin/env python3
"""Hand tracking probe (feasibility): MediaPipe HandLandmarker on the ceiling Kinect's colour image + the aligned
depth -> palm centre / wrist / index fingertip in the robot's world frame. Prints rate, detections and positions,
saves an annotated image. Read-only (subscribes only).

    source ~/prj/ros2_ws/ros_env.sh && ~/prj/venvs/hands/bin/python hand_probe.py [seconds] [out.jpg]
"""
import sys, time, threading, collections
import numpy as np, cv2, rclpy
import mediapipe as mp
from mediapipe.tasks.python import vision, BaseOptions
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros
from scipy.spatial.transform import Rotation

SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
OUT = sys.argv[2] if len(sys.argv) > 2 else '/tmp/hand_probe.jpg'
MODEL = '/home/whoobee/prj/venvs/hands/models/hand_landmarker.task'
PALM = (0, 5, 9, 13, 17)                 # wrist + finger bases: the palm centre
WRIST, INDEX_TIP = 0, 8

rclpy.init(); node = rclpy.create_node('hand_probe')
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, node)
latest = {}
node.create_subscription(Image, '/kinect/rgb/image_raw', lambda m: latest.__setitem__('rgb', m), qos_profile_sensor_data)
node.create_subscription(Image, '/kinect/depth_to_rgb/image_raw', lambda m: latest.__setitem__('depth', m), qos_profile_sensor_data)
node.create_subscription(CameraInfo, '/kinect/rgb/camera_info', lambda m: latest.__setitem__('info', m), qos_profile_sensor_data)
threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

def to_np(msg):
    if msg.encoding in ('bgra8', 'rgba8'):
        a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 4)
        return cv2.cvtColor(a, cv2.COLOR_BGRA2RGB if msg.encoding == 'bgra8' else cv2.COLOR_RGBA2RGB)
    if msg.encoding in ('bgr8', 'rgb8'):
        a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(a, cv2.COLOR_BGR2RGB) if msg.encoding == 'bgr8' else a.copy()
    if msg.encoding == '16UC1':
        return np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width).astype(np.float32) / 1000.0
    if msg.encoding == '32FC1':
        return np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width).copy()
    raise ValueError(msg.encoding)

def depth_at(depth, u, v, r=4):
    """Median of the valid depths in a (2r+1)^2 window (m), or None."""
    h, w = depth.shape
    u, v = int(round(u)), int(round(v))
    win = depth[max(v - r, 0):min(v + r + 1, h), max(u - r, 0):min(u + r + 1, w)]
    win = win[(win > 0.2) & (win < 3.0) & np.isfinite(win)]
    return float(np.median(win)) if win.size >= 5 else None

opts = vision.HandLandmarkerOptions(base_options=BaseOptions(model_asset_path=MODEL), running_mode=vision.RunningMode.VIDEO,
                                    num_hands=2, min_hand_detection_confidence=0.5, min_hand_presence_confidence=0.5,
                                    min_tracking_confidence=0.5)
lm = vision.HandLandmarker.create_from_options(opts)
end = time.time() + 60
while not all(k in latest for k in ('rgb', 'depth', 'info')) and time.time() < end:
    time.sleep(0.1)
if 'rgb' not in latest:
    print('no camera images (is the cell running?)'); sys.exit(1)
info = latest['info']; K = np.array(info.k).reshape(3, 3)
print(f'camera: {latest["rgb"].width}x{latest["rgb"].height} {latest["rgb"].encoding}, depth {latest["depth"].encoding}')

frames = det = 0; t_inf = []; tracks = collections.defaultdict(list); last_seen_stamp = None
best = None; t0 = time.time(); last_print = 0
while time.time() - t0 < SECONDS:
    rgb_msg, depth_msg = latest['rgb'], latest['depth']
    stamp = rgb_msg.header.stamp.sec * 1000 + rgb_msg.header.stamp.nanosec // 1000000
    if stamp == last_seen_stamp:
        time.sleep(0.005); continue
    last_seen_stamp = stamp
    rgb, depth = to_np(rgb_msg), to_np(depth_msg)
    t = time.time(); res = lm.detect_for_video(mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), int((t - t0) * 1000) + 1)
    t_inf.append(time.time() - t); frames += 1
    try:
        tf = buf.lookup_transform('world', rgb_msg.header.frame_id, Time()).transform
    except tf2_ros.TransformException:
        continue
    T = np.eye(4); T[:3, :3] = Rotation.from_quat([tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w]).as_matrix()
    T[:3, 3] = [tf.translation.x, tf.translation.y, tf.translation.z]
    if res.hand_landmarks:
        det += 1
    for i, hand in enumerate(res.hand_landmarks):
        side = res.handedness[i][0].category_name if res.handedness else '?'
        h, w = rgb.shape[:2]
        world = res.hand_world_landmarks[i]          # metres, hand-centred, ~camera-aligned axes
        px = np.array([[p.x * w, p.y * h] for p in hand]); wl = np.array([[p.x, p.y, p.z] for p in world])
        # palm depth from the sensor: a large window over the palm centre (nearly always visible)
        up, vp = px[list(PALM)].mean(0)
        z_palm = depth_at(depth, up, vp, 8)
        if z_palm is None:
            continue
        wl_palm = wl[list(PALM)].mean(0)
        pts = {}
        for name, idx in (('palm', PALM), ('wrist', (WRIST,)), ('index', (INDEX_TIP,))):
            u, v = px[list(idx)].mean(0)
            z_pred = z_palm + (wl[list(idx)].mean(0)[2] - wl_palm[2])     # hand shape: depth relative to the palm
            z_meas = depth_at(depth, u, v, 6 if name == 'palm' else 3)
            # a landmark's own depth unless the depth camera sees something else there (the arm in front of a
            # fingertip): then the palm depth + the hand shape
            z = z_meas if z_meas is not None and abs(z_meas - z_pred) < 0.03 else z_pred
            if name != 'palm':
                tracks[f'{side}:{name} fallback'].append(np.array([float(z is z_pred), 0, 0]))
            cam = np.array([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z, 1.0])
            pts[name] = (T @ cam)[:3]
            if z_meas is not None:            # the old way, for comparison
                cam = np.array([(u - K[0, 2]) * z_meas / K[0, 0], (v - K[1, 2]) * z_meas / K[1, 1], z_meas, 1.0])
                tracks[f'{side}:{name} (raw)'].append((T @ cam)[:3])
        for name, p in pts.items():
            tracks[f'{side}:{name}'].append(p)
        if 'palm' in pts:
            best = (rgb.copy(), res, pts)
    if time.time() - last_print > 2.0 and tracks:
        last_print = time.time()
        line = ' | '.join(f'{k} ({", ".join(f"{v * 1000:.0f}" for v in ps[-1])}) mm' for k, ps in tracks.items() if ps and k.endswith('palm'))
        print(f'{time.time() - t0:5.1f}s {line}', flush=True)

dt = time.time() - t0
print(f'\nframes {frames} in {dt:.1f} s = {frames / dt:.1f} fps processed; inference {np.mean(t_inf) * 1000:.1f} ms '
      f'(max {np.max(t_inf) * 1000:.0f}); frames with a hand {det} ({det / max(frames, 1) * 100:.0f}%)')
for k, ps in sorted(tracks.items()):
    P = np.array(ps)
    print(f'{k:14} n={len(P):4}  mean ({", ".join(f"{v * 1000:.0f}" for v in P.mean(0))}) mm  '
          f'std ({", ".join(f"{v * 1000:.1f}" for v in P.std(0))}) mm  [last 10 frames std '
          f'{", ".join(f"{v * 1000:.1f}" for v in P[-10:].std(0))}]')
if best:
    img, res, pts = best; img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR); h, w = img.shape[:2]
    for hand in res.hand_landmarks:
        for p in hand:
            cv2.circle(img, (int(p.x * w), int(p.y * h)), 4, (0, 255, 255), -1)
    for name, p in pts.items():
        cv2.putText(img, f'{name} {p[0]*1000:.0f},{p[1]*1000:.0f},{p[2]*1000:.0f}', (20, 40 + 30 * list(pts).index(name)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
    cv2.imwrite(OUT, img); print('annotated:', OUT)
sys.stdout.flush(); import os; os._exit(0)
