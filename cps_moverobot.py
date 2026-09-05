"""
play_csv_positions.py

Reads CSV motor positions from a user-specified .txt file and replays them
sequentially on the SO-101 follower arm. Press Ctrl+C at any time to halt playback.
"""

import os
import csv
import time
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

# ---------------------------------------------------------------------------
# Configuration Parameters
# ---------------------------------------------------------------------------
FOLLOWER_PORT = "COM5"           # Your follower arm's serial port
FOLLOWER_ID = "my_follower_arm"  # Must match the ID used during calibration
PLAYBACK_HZ = 20                 # Match recording rate (20 Hz = 0.05s per frame)

# Exact joint order matching your saved CSV format
JOINT_ORDER = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper"
]


def select_input_file():
    """Prompts the user to confirm playback and specify the file name to read from."""
    choice = input("Do you want to play recorded positions from a file? (yes/no): ").strip().lower()
    
    if choice not in ("yes", "y"):
        print("Exiting program.")
        return None

    filename = input("Enter the name of the txt file to read from (e.g., recorded_angles.txt): ").strip()
    
    # Auto-append .txt extension if omitted
    if not filename.endswith(".txt"):
        filename += ".txt"

    # Check if file exists before proceeding
    if not os.path.exists(filename):
        print(f"Error: File '{filename}' does not exist in the current folder.")
        print("Exiting program.")
        return None

    return filename


def load_positions_from_file(filepath):
    """Reads CSV joint values from file into a list of dictionaries."""
    positions = []
    with open(filepath, "r") as f:
        # csv.reader handles lines with or without header
        reader = csv.reader(f)
        for row in reader:
            # Skip empty lines
            if not row:
                continue
            
            # Skip header row if present
            if row[0].strip() == "shoulder_pan":
                continue

            # Ensure row has all 6 motor values
            if len(row) >= 6:
                try:
                    pos_dict = {
                        f"{joint}.pos": float(val.strip())
                        for joint, val in zip(JOINT_ORDER, row[:6])
                    }
                    positions.append(pos_dict)
                except ValueError:
                    continue  # Skip corrupted lines

    return positions


def main():
    # Prompt user for confirmation and file name before connecting to robot
    input_file = select_input_file()
    if input_file is None:
        return

    positions = load_positions_from_file(input_file)
    if not positions:
        print(f"No valid position data found to execute inside '{input_file}'. Exiting.")
        return

    print(f"Loaded {len(positions)} waypoint(s) from '{input_file}'.")

    # Initialize and connect the robot arm
    config = SO101FollowerConfig(port=FOLLOWER_PORT, id=FOLLOWER_ID)
    robot = SO101Follower(config)
    
    print("Connecting to robot...")
    robot.connect(calibrate=False)

    try:
        # Re-enable motor torque for active control
        robot.bus.enable_torque()
        print("Torque enabled — starting trajectory execution.")
        print("Press Ctrl+C to abort at any time.\n")

        frame_duration = 1.0 / PLAYBACK_HZ

        for idx, pos_target in enumerate(positions, start=1):
            start_time = time.time()

            # Send position targets to the follower arm
            robot.send_action(pos_target)

            # Terminal progress feedback
            print(f"\rExecuting frame {idx}/{len(positions)}", end="", flush=True)

            # Maintain constant playback frequency (20 Hz)
            elapsed = time.time() - start_time
            sleep_time = max(0.0, frame_duration - elapsed)
            time.sleep(sleep_time)

        print("\n\nTrajectory playback completed successfully.")

    except KeyboardInterrupt:
        print("\n\nPlayback interrupted by user.")
    finally:
        # Disable torque so arm can be moved manually safely after script ends
        print("Disabling motor torque...")
        robot.bus.disable_torque()
        robot.disconnect()
        print("Disconnected.")


if __name__ == "__main__":
    main()