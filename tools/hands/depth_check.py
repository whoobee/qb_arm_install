#!/usr/bin/env python3
"""Depth coverage and table noise of the Kinect (read-only): N frames of depth_to_rgb -> valid fraction (all, top band
where the user sits), table flatness / temporal noise inside the vision workspace, a coverage image."""
import sys, time, threading
import numpy as np, cv2, rclpy, tf2_ros, yaml
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from scipy.spatial.transform import Rotation
from qb_arm import boundaries
from qb_arm.camera_pose import config_path
OUT = sys.argv[1]; N = 10
rclpy.init(); n = rclpy.create_node('depth_check'); buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
frames, got = [], {}
n.create_subscription(Image, '/kinect/depth_to_rgb/image_raw', lambda m: frames.append(m) if len(frames) < N else None, qos_profile_sensor_data)
n.create_subscription(Image, '/kinect/rgb/image_raw', lambda m: got.__setitem__('rgb', m), qos_profile_sensor_data)
n.create_subscription(CameraInfo, '/kinect/rgb/camera_info', lambda m: got.__setitem__('info', m), qos_profile_sensor_data)
threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
end = time.time() + 30
while (len(frames) < N or 'info' not in got or 'rgb' not in got) and time.time() < end: time.sleep(0.1)
D = np.stack([np.frombuffer(f.data, np.uint16).reshape(f.height, f.width).astype(np.float32) / 1000 for f in frames])
D[D <= 0] = np.nan
K = np.array(got['info'].k).reshape(3, 3); h, w = D.shape[1:]
tf = buf.lookup_transform('world', frames[0].header.frame_id, Time(), timeout=rclpy.duration.Duration(seconds=3)).transform
T = np.eye(4); T[:3, :3] = Rotation.from_quat([tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w]).as_matrix()
T[:3, 3] = [tf.translation.x, tf.translation.y, tf.translation.z]
valid = np.isfinite(D).mean(0)                         # per pixel: fraction of frames with depth
print(f'valid depth: whole image {np.mean(valid > 0.5) * 100:.0f}%, top band (rows < 200, the user side) '
      f'{np.mean(valid[:200] > 0.5) * 100:.0f}%, rows < 100 {np.mean(valid[:100] > 0.5) * 100:.0f}%')
med = np.nanmedian(D, 0)
v, u = np.mgrid[0:h, 0:w]
cam = np.stack([(u - K[0, 2]) * med / K[0, 0], (v - K[1, 2]) * med / K[1, 1], med], -1).reshape(-1, 3)
world = cam @ T[:3, :3].T + T[:3, 3]
a, b, c = yaml.safe_load(open(config_path('table.yaml')))['/**']['ros__parameters']['table_plane']
ws, boxes, _ = boundaries.load(config_path('boundaries.yaml'))
inside = ws.contains_xy(world[:, :2]) & np.isfinite(world[:, 2])
resid = world[:, 2] - (a * world[:, 0] + b * world[:, 1] + c)
table = inside & (np.abs(resid) < 0.01)
print(f'table pixels in the workspace: {table.sum()}  flatness (std of z - plane): {np.std(resid[table]) * 1000:.2f} mm, '
      f'mean {np.mean(resid[table]) * 1000:+.2f} mm')
tstd = np.nanstd(D, 0).reshape(-1)
print(f'temporal noise on the table (per-pixel std over {N} frames): median {np.nanmedian(tstd[table]) * 1000:.2f} mm, '
      f'p90 {np.nanpercentile(tstd[table], 90) * 1000:.2f} mm')
# user side: the table band 0.45 .. 0.75 m in front of the robot, y -0.3 .. 0.6
band = (world[:, 0] > 0.45) & (world[:, 0] < 0.75) & (world[:, 1] > -0.3) & (world[:, 1] < 0.6)
print(f'pixels seeing x 0.45..0.75 m (user side): {band.sum()}')
rgb = np.frombuffer(got['rgb'].data, np.uint8).reshape(got['rgb'].height, got['rgb'].width, 4)[:, :, :3].copy()
overlay = rgb.copy(); overlay[valid <= 0.5] = (0, 0, 180)
cv2.imwrite(OUT, cv2.addWeighted(rgb, 0.5, overlay, 0.5, 0)); print('coverage image (red = no depth):', OUT)
sys.stdout.flush(); import os; os._exit(0)
