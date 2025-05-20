## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

import time
import numpy as np
import cv2
import pyrealsense2 as rs
import mediapipe as mp
import redis

EE_POS_DESIRED_KEY = "sai::controller::ee_pos_desired"

# --- MediaPipe setup ---
mp_pose = mp.solutions.pose
pose    = mp_pose.Pose()
TORSO_LANDMARKS = [
    mp_pose.PoseLandmark.LEFT_SHOULDER,
    mp_pose.PoseLandmark.RIGHT_SHOULDER,
    mp_pose.PoseLandmark.LEFT_HIP,
    mp_pose.PoseLandmark.RIGHT_HIP
]

def interpolate_grid(top_left, top_right, bottom_left, bottom_right, n, m):
    grid = []
    for i in range(m):
        v = i/(m-1) if m>1 else 0
        row = []
        for j in range(n):
            u = j/(n-1) if n>1 else 0
            top    = np.array(top_left)*(1-u) + np.array(top_right)*u
            bottom = np.array(bottom_left)*(1-u) + np.array(bottom_right)*u
            pt     = (1-v)*top + v*bottom
            row.append((pt[0], pt[1]))
        grid.append(row)
    return grid

def get_torso_grid_with_depth(num_columns=9, num_rows=5, wait_time=3):
    # — RealSense setup & alignment —
    pipeline = rs.pipeline()
    cfg      = rs.config()
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    aligner  = rs.align(rs.stream.color)
    pipeline.start(cfg)

    print(f"Stabilizing... please hold still for {wait_time} seconds")
    time.sleep(wait_time)

    # grab one good frame
    while True:
        frames      = pipeline.wait_for_frames()
        aligned     = aligner.process(frames)
        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()
        if depth_frame and color_frame:
            color_image = np.asanyarray(color_frame.get_data())
            break

    # get intrinsics for deprojection
    depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
    pipeline.stop()

    h, w, _ = color_image.shape

    # — MediaPipe pose on color —
    rgb     = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
    results = pose.process(rgb)
    if not results.pose_landmarks:
        raise RuntimeError("Failed to detect torso landmarks")

    # corner pixel coords
    coords = {}
    for lm in TORSO_LANDMARKS:
        p = results.pose_landmarks.landmark[lm]
        coords[lm.name] = (p.x * w, p.y * h)

    # full grid
    grid = interpolate_grid(
        coords["LEFT_SHOULDER"],
        coords["RIGHT_SHOULDER"],
        coords["LEFT_HIP"],
        coords["RIGHT_HIP"],
        n=num_columns,
        m=num_rows
    )

    # annotate frame
    frame = color_image.copy()
    for (x, y) in coords.values():
        cv2.circle(frame, (int(x), int(y)), 6, (0, 255, 0), -1)
    for row in grid:
        for (x, y) in row:
            cv2.circle(frame, (int(x), int(y)), 2, (255, 0, 0), -1)

    # select subset, deproject, label
    real_points = []
    idx = 1
    for ri in [1, 3]:
        for ci in [1, 3, 5, 7]:
            px, py = grid[ri][ci]
            xi, yi = int(np.clip(px, 0, w-1)), int(np.clip(py, 0, h-1))
            depth = depth_frame.get_distance(xi, yi)
            X, Y, Z = rs.rs2_deproject_pixel_to_point(depth_intrin, [xi, yi], depth)
            real_points.append((X, Y, Z))

            cv2.circle(frame, (xi, yi), 6, (0, 0, 255), -1)
            cv2.putText(
                frame,
                str(idx),
                (xi - 15, yi + 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (255, 255, 255),
                2,
                cv2.LINE_AA
            )
            idx += 1

    # print them immediately
    print("\n--- RAW 3D POINTS (X, Y, Z) ---")
    for i, (X, Y, Z) in enumerate(real_points, start=1):
        print(f"{i}: ({X:.3f}, {Y:.3f}, {Z:.3f})")

    # save annotated image
    img_path = "torso_grid_labeled.png"
    cv2.imwrite(img_path, frame)
    print(f"[get_torso] 🖼️ Saved annotated image to '{img_path}'")

    return real_points

def transform_points(points):
    """
    Rotate each (X,Y,Z) by:
      1) 90° about X
      2) 180° about Z
    """
    θ1 = np.pi/2
    Rx1 = np.array([
        [1,            0,             0],
        [0, np.cos(θ1), -np.sin(θ1)],
        [0, np.sin(θ1),  np.cos(θ1)]
    ])
    θ2 = np.pi
    Rx2 = np.array([
        [np.cos(θ2), -np.sin(θ2),           0],
        [np.sin(θ2),  np.cos(θ2), 0],
        [0, 0, 1]
    ])
    R =  Rx1 @ Rx2

    return [tuple((R @ np.array(p)).tolist()) for p in points]

def publish_all_xyz_as_list(points, host="localhost", port=6379, db=0):
    if not points:
        print("[publish] ⚠️ no points to publish")
        return
    try:
        r = redis.Redis(host=host, port=port, db=db, socket_timeout=2)
        # clear old list
        r.delete(EE_POS_DESIRED_KEY)
        for i, (X, Y, Z) in enumerate(points, start=1):
            val = f"{X:.6f} {Y:.6f} {Z:.6f}"
            print(f"[publish] ▶ pushing {EE_POS_DESIRED_KEY}[{i}] = '{val}'")
            r.rpush(EE_POS_DESIRED_KEY, val)
    except Exception as e:
        print(f"[publish] ❌ ERROR: {e}")

if __name__ == "__main__":
    # 1) Capture raw torso points
    raw_pts = get_torso_grid_with_depth()

    # 2) Rotate them
    rotated_pts = transform_points(raw_pts)

    # 3) Save **all** rotated points to file
    txt_path = "torso_points_transformed.txt"
    with open(txt_path, "w") as f:
        for i, (X, Y, Z) in enumerate(rotated_pts, start=1):
            f.write(f"{i}: {X:.6f} {Y:.6f} {Z:.6f}\n")
    print(f"[main] 💾 Saved all transformed points to '{txt_path}'")

    # 4) Push **all** points into Redis list
    publish_all_xyz_as_list(rotated_pts)
