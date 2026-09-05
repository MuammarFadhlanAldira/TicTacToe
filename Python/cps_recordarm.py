"""
Puts the SO-101 arm into free-move mode (torque off), prints joint angles
in real time, and AUTOMATICALLY saves positions in CSV format inside a .txt file
every 0.05 seconds (or specified interval). Press Ctrl+C or 'Q' to exit.
"""

import os
import time
import msvcrt  # Built-in library on Windows for non-blocking key detection
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

FOLLOWER_PORT = "COM5"           # your follower arm's port
FOLLOWER_ID = "my_follower_arm"  # must match the id used during `lerobot-calibrate`
READ_HZ = 20                     # how many times per second to check/print
RECORD_INTERVAL = 0.05           # interval in seconds between automatic saves

# Ordered list of joints to extract and save
JOINT_ORDER = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper"
]


def format_csv_line(obs):
    """Formats the current motor observation into a CSV row matching JOINT_ORDER."""
    # Extract values based on joint names ending with .pos or direct joint names
    values = []
    for joint in JOINT_ORDER:
        # Check both potential key forms in the dictionary ('joint_name.pos' or 'joint_name')
        key = f"{joint}.pos" if f"{joint}.pos" in obs else joint
        angle = obs.get(key, 0.0)
        values.append(f"{angle:.2f}")
    
    return ",".join(values)


def setup_output_file():
    """Prompts the user to create a text file before starting recording."""
    choice = input("Do you want to create a new txt file? (yes/no): ").strip().lower()
    
    if choice not in ("yes", "y"):
        print("Exiting program.")
        return None

    filename = input("Enter the name of the new txt file (e.g., recorded_angles.txt): ").strip()
    
    # Auto-append .txt extension if omitted
    if not filename.endswith(".txt"):
        filename += ".txt"

    # Check if file exists to display appropriate message
    if os.path.exists(filename):
        print(f"File '{filename}' already exists. Overwriting existing file...")
    else:
        print(f"Creating new file '{filename}'...")

    # Clear/create the file using write mode ('w') and insert the CSV header
    with open(filename, "w") as f:
        f.write(",".join(JOINT_ORDER) + "\n")
    
    print(f"Initialized CSV header in '{filename}'. Starting recording...\n")
    return filename


def main():
    # Prompt user for file creation before connecting to robot
    output_file = setup_output_file()
    if output_file is None:
        return

    config = SO101FollowerConfig(port=FOLLOWER_PORT, id=FOLLOWER_ID)
    robot = SO101Follower(config)
    robot.connect(calibrate=False)

    # Free the motors so the arm can be moved by hand
    robot.bus.disable_torque()
    print("Torque disabled — you can move the arm freely by hand.")
    print(f"Recording joint angles automatically every {RECORD_INTERVAL} second(s) to '{output_file}'.")
    print("  -> Press 'Q' or Ctrl+C to exit.\n")

    period = 1.0 / READ_HZ
    saved_count = 1  # Starts count at Position 1
    last_save_time = time.time()  # Track when the last recording happened

    try:
        while True:
            obs = robot.get_observation()
            current_time = time.time()

            # 1. Print live streaming values on terminal line
            line = "  ".join(
                f"{name.replace('.pos', ''):>13}: {angle:7.2f}"
                for name, angle in obs.items()
                if name.endswith(".pos")
            )
            print(f"\r{line}", end="", flush=True)

            # 2. Automatic recording check
            if current_time - last_save_time >= RECORD_INTERVAL:
                csv_line = format_csv_line(obs)

                # Append CSV row to output file
                with open(output_file, "a") as f:
                    f.write(csv_line + "\n")

                print(f"\n[AUTO-SAVED #{saved_count}] -> {csv_line}")

                saved_count += 1
                last_save_time = current_time

            # 3. Non-blocking keypress check for exiting
            if msvcrt.kbhit():
                key = msvcrt.getch().decode("utf-8", errors="ignore").lower()
                if key == "q":
                    print("\n\nExiting program...")
                    break

            time.sleep(period)

    except KeyboardInterrupt:
        print("\n\nStopped by user.")
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()