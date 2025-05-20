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
        if not depth_frame or not color_frame:
            continue
        color_image = np.asanyarray(color_frame.get_data())
        break

    pipeline.stop()

    h, w, _ = color_image.shape

    # — MediaPipe pose on color —
    rgb     = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
    results = pose.process(rgb)
    if not results.pose_landmarks:
        raise RuntimeError("Failed to detect torso landmarks")

    # collect the four corner coords
    coords = {}
    for lm in TORSO_LANDMARKS:
        p = results.pose_landmarks.landmark[lm]
        coords[lm.name] = (p.x * w, p.y * h)

    # build full m×n grid
    grid = interpolate_grid(
        coords["LEFT_SHOULDER"],
        coords["RIGHT_SHOULDER"],
        coords["LEFT_HIP"],
        coords["RIGHT_HIP"],
        n=num_columns,
        m=num_rows
    )

    # annotate exactly like your original:
    frame = color_image.copy()
    # 1) green circles for the 4 landmarks
    for name, (x, y) in coords.items():
        cv2.circle(frame, (int(x), int(y)), 6, (0, 255, 0), -1)

    # 2) blue dots for every grid cell
    for row in grid:
        for (x, y) in row:
            cv2.circle(frame, (int(x), int(y)), 2, (255, 0, 0), -1)

    # 3) select your subset [rows 1,3]×[cols 1,3,5,7], fetch depth, draw red
    selected = []
    for ri in [1, 3]:
        for ci in [1, 3, 5, 7]:
            x, y = grid[ri][ci]
            xi   = int(np.clip(x, 0, w-1))
            yi   = int(np.clip(y, 0, h-1))
            z    = depth_frame.get_distance(xi, yi)
            selected.append((x, y, z))
            cv2.circle(frame, (xi, yi), 4, (0, 0, 255), -1)

    # show the annotated frame
    cv2.imshow("Torso Landmarks and Grid", frame)
    print("\n--- SELECTED GRID POINTS (x, y, z in meters) ---")
    for (x, y, z) in selected:
        print(f"({x:.1f}, {y:.1f}, {z:.3f})")

    # timed wait (3 seconds) so that we always continue
    key = cv2.waitKey(3000)
    if key < 0:
        print("[get_torso] ▶ timeout reached, closing display")
    cv2.destroyAllWindows()

    return selected

def publish_xyz_as_eigen(selected_points,
                         pick_index=0,
                         host="localhost",
                         port=6379,
                         db=0):
    """
    Picks one (x,y,z) from selected_points, converts to an Eigen-style 
    space-separated string "x y z", and SETs it into Redis so that
    C++ RedisClient.getEigen() will parse it as Vector3d.
    """
    if not selected_points:
        print("[publish] ⚠️ no points to publish")
        return

    x, y, z = selected_points[pick_index]
    val = f"{x:.6f} {y:.6f} {z:.6f}"

    try:
        r = redis.Redis(host=host, port=port, db=db, socket_timeout=2)
        print(f"[publish] ▶ connecting to Redis @ {host}:{port}, db={db}")
        ok = r.set(EE_POS_DESIRED_KEY, val)
        print(f"[publish] ✔ r.set returned: {ok!r}")
        keys = r.keys()
        print(f"[publish] 🔑 keys in DB0: {keys}")
        got = r.get(EE_POS_DESIRED_KEY)
        print(f"[publish] 📝 get '{EE_POS_DESIRED_KEY}': {got!r}")
    except Exception as e:
        print(f"[publish] ❌ ERROR talking to Redis: {e}")

if __name__ == "__main__":
    pts_3d = get_torso_grid_with_depth()
    print("Returned points:", pts_3d)

    # Publish the first point as an Eigen::Vector3d
    publish_xyz_as_eigen(pts_3d, pick_index=0)
