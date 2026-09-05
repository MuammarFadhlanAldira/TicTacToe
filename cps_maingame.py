import csv
import json
import os
import time
import threading
import cv2
import numpy as np

# LeRobot imports
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

CALIBRATION_FILE = "board_corners.json"
WARPED_SIZE = 300  # Size of the warped square board in pixels

# Robot Arm Configuration
FOLLOWER_PORT = "COM5"           # Adjust port as needed
FOLLOWER_ID = "my_follower_arm"  # Must match your calibration ID
PLAYBACK_HZ = 20                 # Execution frame rate

JOINT_ORDER = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper"
]

# UI Color Palette (Sleek Dark Theme)
COLOR_BG = (24, 24, 32)
COLOR_CARD = (38, 38, 50)
COLOR_TEXT = (240, 240, 245)
COLOR_ACCENT = (255, 120, 0)
COLOR_BUTTON = (50, 50, 68)
COLOR_HOVER = (70, 70, 95)


# ---------------------------------------------------------------------------
# Robot Trajectory Playback Helpers
# ---------------------------------------------------------------------------
def load_positions_from_file(filepath):
    """Reads CSV joint values from file into a list of dictionaries."""
    if not os.path.exists(filepath):
        print(f"[ROBOT ERROR] File '{filepath}' not found.")
        return []

    positions = []
    with open(filepath, "r") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or row[0].strip() == "shoulder_pan":
                continue
            if len(row) >= 6:
                try:
                    pos_dict = {
                        f"{joint}.pos": float(val.strip())
                        for joint, val in zip(JOINT_ORDER, row[:6])
                    }
                    positions.append(pos_dict)
                except ValueError:
                    continue
    return positions


def execute_trajectory(filepath):
    """Connects to the arm, replays positions from the file, then disconnects."""
    positions = load_positions_from_file(filepath)
    if not positions:
        print(f"[ROBOT] Skipping execution: No valid data in {filepath}")
        return

    print(f"\n[ROBOT] Connecting to arm to play '{filepath}' ({len(positions)} frames)...")
    try:
        config = SO101FollowerConfig(port=FOLLOWER_PORT, id=FOLLOWER_ID)
        robot = SO101Follower(config)
        robot.connect(calibrate=False)
        robot.bus.enable_torque()

        frame_duration = 1.0 / PLAYBACK_HZ
        for idx, pos_target in enumerate(positions, start=1):
            start_time = time.time()
            robot.send_action(pos_target)

            elapsed = time.time() - start_time
            sleep_time = max(0.0, frame_duration - elapsed)
            time.sleep(sleep_time)

        print("[ROBOT] Trajectory completed successfully.")
    except Exception as e:
        print(f"[ROBOT ERROR] Failed during playback: {e}")
    finally:
        try:
            robot.bus.disable_torque()
            robot.disconnect()
            print("[ROBOT] Torque disabled & disconnected.")
        except Exception:
            pass


# ---------------------------------------------------------------------------
# 1. Calibration: Capture initial frame & click 4 corners (TL, TR, BR, BL)
# ---------------------------------------------------------------------------
def calibrate_corners(camera_index=0):
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError("Could not open camera.")

    print("=== CALIBRATION ===")
    print("Point camera at the board. Click on the video window, then press SPACE/ENTER to capture.")

    frozen_frame = None
    while True:
        ok, frame = cap.read()
        if not ok:
            break

        cv2.imshow("Calibration - Step 1: Capture Frame", frame)
        key = cv2.waitKey(30) & 0xFF
        if key in (32, 13, ord('c')):  # SPACE, ENTER, or 'c'
            frozen_frame = frame.copy()
            break
        elif key == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            return None

    cap.release()
    cv2.destroyAllWindows()

    points = []
    def on_click(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(points) < 4:
            points.append([x, y])

    window_name = "Calibration - Step 2: Click 4 Corners (TL, TR, BR, BL)"
    cv2.namedWindow(window_name)
    cv2.imshow(window_name, frozen_frame)
    cv2.waitKey(100)
    cv2.setMouseCallback(window_name, on_click)

    print("Click 4 corners: 1. Top-Left -> 2. Top-Right -> 3. Bottom-Right -> 4. Bottom-Left.")
    print("Press 'r' to reset points, 'q' when 4 points are placed.")

    while True:
        display = frozen_frame.copy()
        for i, p in enumerate(points):
            cv2.circle(display, tuple(p), 6, (0, 255, 0), -1)
            cv2.putText(display, str(i), (p[0] + 8, p[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        if len(points) == 4:
            pts = np.array(points, dtype=np.int32)
            cv2.polylines(display, [pts], True, (0, 255, 0), 2)

        cv2.imshow(window_name, display)
        key = cv2.waitKey(20) & 0xFF
        if key == ord('r'):
            points.clear()
        elif key == ord('q') and len(points) == 4:
            break

    cv2.destroyAllWindows()

    with open(CALIBRATION_FILE, "w") as f:
        json.dump(points, f)
    print(f"Saved corners to {CALIBRATION_FILE}: {points}\n")
    return points


def load_corners():
    if not os.path.exists(CALIBRATION_FILE):
        return None
    with open(CALIBRATION_FILE) as f:
        return json.load(f)


def get_perspective_transform(corners):
    src = np.array(corners, dtype=np.float32)
    dst = np.array([
        [0, 0],
        [WARPED_SIZE, 0],
        [WARPED_SIZE, WARPED_SIZE],
        [0, WARPED_SIZE]
    ], dtype=np.float32)
    return cv2.getPerspectiveTransform(src, dst)


# ---------------------------------------------------------------------------
# 2. Vision Classification: White = 'O', Black = 'X', Empty
# ---------------------------------------------------------------------------
def classify_cell(cell_bgr):
    hsv = cv2.cvtColor(cell_bgr, cv2.COLOR_BGR2HSV)
    cell_h, cell_w = cell_bgr.shape[:2]
    total_pixels = cell_h * cell_w

    # White Piece Detection ('O')
    lower_white = np.array([0, 0, 180])
    upper_white = np.array([180, 60, 255])
    white_mask = cv2.inRange(hsv, lower_white, upper_white)

    kernel = np.ones((3, 3), np.uint8)
    white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)
    white_pixels = cv2.countNonZero(white_mask)
    white_ratio = white_pixels / float(total_pixels)

    if white_ratio > 0.12:
        contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 40:
                return "O"

    # Black Piece Detection ('X')
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 75])
    black_mask = cv2.inRange(hsv, lower_black, upper_black)

    black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, kernel)
    black_pixels = cv2.countNonZero(black_mask)
    black_ratio = black_pixels / float(total_pixels)

    if black_ratio > 0.12:
        contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 40:
                return "X"

    return "empty"


def read_board(frame, corners):
    transform = get_perspective_transform(corners)
    warped = cv2.warpPerspective(frame, transform, (WARPED_SIZE, WARPED_SIZE))

    cell_size = WARPED_SIZE // 3
    board = []

    for row in range(3):
        for col in range(3):
            y0, y1 = row * cell_size, (row + 1) * cell_size
            x0, x1 = col * cell_size, (col + 1) * cell_size

            pad = cell_size // 5
            cell = warped[y0 + pad:y1 - pad, x0 + pad:x1 - pad]
            board.append(classify_cell(cell))

    return board, warped


# ---------------------------------------------------------------------------
# 3. Game Logic: Win / Draw Checking
# ---------------------------------------------------------------------------
def check_game_status(board):
    win_conditions = [
        (0, 1, 2), (3, 4, 5), (6, 7, 8),  # Rows
        (0, 3, 6), (1, 4, 7), (2, 5, 8),  # Columns
        (0, 4, 8), (2, 4, 6)               # Diagonals
    ]

    for condition in win_conditions:
        a, b, c = condition
        if board[a] != "empty" and board[a] == board[b] == board[c]:
            return "WIN", board[a], condition

    if "empty" not in board:
        return "DRAW", None, None

    return "ONGOING", None, None


# ---------------------------------------------------------------------------
# 4. Sleek Modern Main Menu UI
# ---------------------------------------------------------------------------
def draw_button(img, text, pos, size, is_hovered):
    x, y = pos
    w, h = size
    color = COLOR_HOVER if is_hovered else COLOR_BUTTON
    
    cv2.rectangle(img, (x, y), (x + w, y + h), color, -1)
    cv2.rectangle(img, (x, y), (x + w, y + h), COLOR_ACCENT if is_hovered else (80, 80, 100), 2)
    
    text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
    tx = x + (w - text_size[0]) // 2
    ty = y + (h + text_size[1]) // 2
    cv2.putText(img, text, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_TEXT, 2)


def main_menu(camera_index=0):
    menu_win = "Tic-Tac-Toe Live Vision - Main Menu"
    cv2.namedWindow(menu_win)

    mouse_pos = (-1, -1)
    clicked = False

    def on_mouse(event, x, y, flags, param):
        nonlocal mouse_pos, clicked
        mouse_pos = (x, y)
        if event == cv2.EVENT_LBUTTONDOWN:
            clicked = True

    cv2.setMouseCallback(menu_win, on_mouse)

    width, height = 700, 500

    btn_w, btn_h = 320, 50
    btn_x = (width - btn_w) // 2
    btn_start = (btn_x, 220)
    btn_calib = (btn_x, 290)
    btn_exit = (btn_x, 360)

    while True:
        menu_img = np.full((height, width, 3), COLOR_BG, dtype=np.uint8)

        cv2.putText(menu_img, "TIC-TAC-TOE VISION", (width // 2 - 220, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.3, COLOR_TEXT, 3)
        cv2.putText(menu_img, "White = O  |  Black = X", (width // 2 - 130, 130),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLOR_ACCENT, 2)

        mx, my = mouse_pos
        hover_start = (btn_start[0] <= mx <= btn_start[0] + btn_w) and (btn_start[1] <= my <= btn_start[1] + btn_h)
        hover_calib = (btn_calib[0] <= mx <= btn_calib[0] + btn_w) and (btn_calib[1] <= my <= btn_calib[1] + btn_h)
        hover_exit = (btn_exit[0] <= mx <= btn_exit[0] + btn_w) and (btn_exit[1] <= my <= btn_exit[1] + btn_h)

        draw_button(menu_img, "START GAME", btn_start, (btn_w, btn_h), hover_start)
        draw_button(menu_img, "RECALIBRATE BOARD", btn_calib, (btn_w, btn_h), hover_calib)
        draw_button(menu_img, "EXIT", btn_exit, (btn_w, btn_h), hover_exit)

        cv2.imshow(menu_win, menu_img)
        key = cv2.waitKey(20) & 0xFF

        if clicked:
            clicked = False
            if hover_start:
                cv2.destroyWindow(menu_win)
                return "START"
            elif hover_calib:
                cv2.destroyWindow(menu_win)
                return "CALIBRATE"
            elif hover_exit:
                cv2.destroyWindow(menu_win)
                return "EXIT"

        if key == ord('q'):
            cv2.destroyWindow(menu_win)
            return "EXIT"


# ---------------------------------------------------------------------------
# 5. Main Game Loop with Live Camera Rendering & Timed Sequence
# ---------------------------------------------------------------------------
def play_game(camera_index=0):
    corners = load_corners()
    if corners is None:
        corners = calibrate_corners(camera_index)
        if corners is None:
            print("Calibration aborted.")
            return

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError("Could not open camera")

    while True:  # Main replay loop
        # Define state order matching steps 1 to 10:
        sequence = [
            {"step": 1, "type": "PLAYER", "duration": 10},
            {"step": 2, "type": "CHECK", "duration": 3},
            {"step": 3, "type": "ROBOT", "duration": 20, "file": "cps_o_pos4.txt"},
            {"step": 4, "type": "CHECK", "duration": 3},
            {"step": 5, "type": "PLAYER", "duration": 10},
            {"step": 6, "type": "CHECK", "duration": 3},
            {"step": 7, "type": "ROBOT", "duration": 20, "file": "cps_o_pos5.txt"},
            {"step": 8, "type": "CHECK", "duration": 3},
            {"step": 9, "type": "PLAYER", "duration": 10},
            {"step": 10, "type": "CHECK", "duration": 3}
        ]

        seq_idx = 0
        step_start_time = time.time()
        robot_thread_started = False
        skip_requested = False

        board = ["empty"] * 9
        transform = get_perspective_transform(corners)
        
        game_over = False
        game_result_text = ""
        win_indices = None

        while True:
            ok, frame = cap.read()
            if not ok:
                print("Error: Could not grab frame.")
                break

            warped = cv2.warpPerspective(frame, transform, (WARPED_SIZE, WARPED_SIZE))

            current_step = sequence[seq_idx] if seq_idx < len(sequence) else None
            elapsed = time.time() - step_start_time

            if not game_over and current_step:
                time_remaining = max(0, int(current_step["duration"] - elapsed))
                step_type = current_step["type"]
                step_num = current_step["step"]

                # --- STEP-SPECIFIC LOGIC ---
                if step_type == "CHECK":
                    # Actively read/classify board only during CHECK steps
                    board, warped = read_board(frame, corners)
                    status, winner, current_win_indices = check_game_status(board)

                    if status == "WIN":
                        game_over = True
                        win_indices = current_win_indices
                        game_result_text = f"GAME OVER: '{winner}' WINS!"
                    elif status == "DRAW":
                        game_over = True
                        game_result_text = "GAME OVER: IT'S A DRAW!"

                elif step_type == "ROBOT":
                    # Non-blocking threaded trajectory execution to maintain video feed
                    if not robot_thread_started:
                        filepath = current_step["file"]
                        print(f"\n>>> STEP {step_num}: Executing Robot Trajectory '{filepath}' <<<")
                        thread = threading.Thread(target=execute_trajectory, args=(filepath,), daemon=True)
                        thread.start()
                        robot_thread_started = True

                # Step Transitioning (triggers when duration finishes or 's' is pressed)
                if elapsed >= current_step["duration"] or skip_requested:
                    skip_requested = False
                    seq_idx += 1
                    step_start_time = time.time()
                    robot_thread_started = False

                    if seq_idx >= len(sequence):
                        game_over = True
                        if not game_result_text:
                            game_result_text = "SEQUENCE COMPLETED"

            # --- BOARD VISUAL OVERLAY ---
            cell_size = WARPED_SIZE // 3
            for idx, symbol in enumerate(board):
                r, c = idx // 3, idx % 3
                cx = c * cell_size + cell_size // 2
                cy = r * cell_size + cell_size // 2

                if symbol == "X":    # Black Piece
                    cv2.putText(warped, "X", (cx - 15, cy + 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 0), 3)
                elif symbol == "O":  # White Piece
                    cv2.putText(warped, "O", (cx - 15, cy + 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 3)

            # Draw grid lines
            for i in range(1, 3):
                cv2.line(warped, (i * cell_size, 0), (i * cell_size, WARPED_SIZE), (200, 200, 200), 1)
                cv2.line(warped, (0, i * cell_size), (WARPED_SIZE, i * cell_size), (200, 200, 200), 1)

            # Draw winning line
            if game_over and win_indices:
                p1 = (win_indices[0] % 3 * cell_size + cell_size // 2, win_indices[0] // 3 * cell_size + cell_size // 2)
                p2 = (win_indices[2] % 3 * cell_size + cell_size // 2, win_indices[2] // 3 * cell_size + cell_size // 2)
                cv2.line(warped, p1, p2, (0, 255, 0), 4)

            # --- CAMERA UI BANNERS ---
            display = frame.copy()
            pts = np.array(corners, dtype=np.int32)
            cv2.polylines(display, [pts], True, (0, 255, 0), 2)

            if not game_over and current_step:
                if current_step["type"] == "PLAYER":
                    color = (0, 255, 0)
                    msg = f"Step {current_step['step']}: Player Turn | Time Left: {time_remaining}s (Press 'S' to Skip)"
                elif current_step["type"] == "ROBOT":
                    color = (0, 165, 255)
                    msg = f"Step {current_step['step']}: Robot Turn ({current_step['file']}) | Time Left: {time_remaining}s (Press 'S' to Skip)"
                else:  # CHECK
                    color = (255, 255, 0)
                    msg = f"Step {current_step['step']}: Checking Board Status... | Time Left: {time_remaining}s (Press 'S' to Skip)"

                cv2.rectangle(display, (10, 10), (750, 60), (0, 0, 0), cv2.FILLED)
                cv2.putText(display, msg, (20, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
            else:
                cv2.rectangle(display, (10, 10), (display.shape[1] - 10, 110), (0, 0, 0), cv2.FILLED)
                cv2.putText(display, game_result_text, (20, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
                cv2.putText(display, "Press 'r' to REPLAY or 'q' to RETURN TO MENU", (20, 90),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

            cv2.imshow("Main Camera Stream", display)
            cv2.imshow("Warped Board Detector", warped)

            key = cv2.waitKey(30) & 0xFF

            if key in (ord('s'), ord('S')):
                skip_requested = True

            elif key == ord('q'):
                cap.release()
                cv2.destroyAllWindows()
                return

            elif key == ord('c'):
                cap.release()
                cv2.destroyAllWindows()
                calibrate_corners(camera_index)
                return play_game(camera_index)

            elif game_over and key == ord('r'):
                break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    CAMERA_INDEX = 0
    while True:
        action = main_menu(CAMERA_INDEX)
        if action == "START":
            play_game(CAMERA_INDEX)
        elif action == "CALIBRATE":
            calibrate_corners(CAMERA_INDEX)
            play_game(CAMERA_INDEX)
        elif action == "EXIT":
            break