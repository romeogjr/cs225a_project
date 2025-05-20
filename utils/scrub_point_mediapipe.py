import cv2
import mediapipe as mp
import time
import numpy as np

# Initialize MediaPipe Pose
mp_pose = mp.solutions.pose
pose = mp_pose.Pose()
TORSO_LANDMARKS = [
    mp_pose.PoseLandmark.LEFT_SHOULDER,
    mp_pose.PoseLandmark.RIGHT_SHOULDER,
    mp_pose.PoseLandmark.LEFT_HIP,
    mp_pose.PoseLandmark.RIGHT_HIP
]

# Bilinear interpolation function for grid creation
def interpolate_grid(top_left, top_right, bottom_left, bottom_right, n, m):
    grid = []
    for i in range(m):
        row = []
        v = i / (m - 1) if m > 1 else 0
        for j in range(n):
            u = j / (n - 1) if n > 1 else 0
            top = np.array(top_left) * (1 - u) + np.array(top_right) * u
            bottom = np.array(bottom_left) * (1 - u) + np.array(bottom_right) * u
            point = top * (1 - v) + bottom * v
            row.append(tuple(point))
        grid.append(row)
    return grid

# Main data collection function
def get_torso_landmarks_and_grid(num_columns=9, num_rows=5, wait_time=3):
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("Unable to open webcam")

    print(f"Stabilizing... Please hold position for {wait_time} seconds.")
    time.sleep(wait_time)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = pose.process(rgb_frame)

        if results.pose_landmarks:
            h, w, _ = frame.shape
            coords = {}

            for landmark in TORSO_LANDMARKS:
                lm = results.pose_landmarks.landmark[landmark]
                coords[landmark.name] = (lm.x * w, lm.y * h)

            if all(name in coords for name in ["LEFT_SHOULDER", "RIGHT_SHOULDER", "LEFT_HIP", "RIGHT_HIP"]):
                grid = interpolate_grid(
                    coords["LEFT_SHOULDER"],
                    coords["RIGHT_SHOULDER"],
                    coords["LEFT_HIP"],
                    coords["RIGHT_HIP"],
                    n=num_columns,
                    m=num_rows
                )
                print(grid)
                cap.release()
                return coords, grid, frame  # Return the captured frame as well

    cap.release()
    raise RuntimeError("Failed to detect pose with all torso landmarks.")

# Main function that uses the above
def main():
    try:
        torso_landmarks, grid, frame = get_torso_landmarks_and_grid()

        print("\n--- TORSO LANDMARKS ---")
        for name, (x, y) in torso_landmarks.items():
            print(f"{name}: x={x:.2f}, y={y:.2f}")
            cv2.circle(frame, (int(x), int(y)), 6, (0, 255, 0), -1)

        for row in grid:
            for (x, y) in row:
                cv2.circle(frame, (int(x), int(y)), 2, (255, 0, 0), -1)

        selected_points = []
        print("\n--- SELECTED GRID POINTS AND HEIGHTS ---")
        for row_idx in [1, 3]:
            for col_idx in [1, 3, 5, 7]:
                x, y = grid[row_idx][col_idx]
                y_top = grid[0][col_idx][1]
                y_bottom = grid[2][col_idx][1]
                # get the value for the amplitude of the sin graph, divide by 2 based on Karim's code
                h = (y_bottom - y_top) / 2.0
                selected_points.append((x, y, h))
                print(f"({x:.2f}, {y:.2f}, {h:.2f})")
                cv2.circle(frame, (int(x), int(y)), 4, (0, 0, 255), -1)

        # Display the frame with annotations
        cv2.imshow("Torso Landmarks and Grid", frame)
        print("\nPress any key on the image window to close.")
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        # Return the tuples 
        return selected_points

    except RuntimeError as e:
        print(f"Error: {e}")
        return []



if __name__ == "__main__":
    main()
