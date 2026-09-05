# TicTacToe Robot

TicTacToe Robot is a cyber physical system that combines computer vision, game logic, and an **SO-101 robotic arm. A camera observes a physical 3×3 board, OpenCV detects the placed pieces, and the software checks the current game state**. During robot turns, the arm executes recorded joint trajectories to physically place a game piece **(X and O)** on the board.

The project also includes a standalone Pygame version called **Tic Tac Clash**, which allows the game interface and logic to be tested without the physical robot or camera.

The project transforms a simple TicTacToe game to allow software to  interact directly with the physical world. Instead of playing only on a screen, the system combines visual perception, board state recognition, game logic, and robotic movement.

The main goal is to demonstrate how a camera and robotic arm can work together in a closed interaction loop:

**Player move → Camera detection → Game state evaluation → Robot movement → Camera verification**

---

## Table of Contents
* [TicTacToe Robot](#-tictactoe-robot)
* [Features](#-features)
* [Hardware](#-hardware)
* [Software](#-software)
* [Project Files](#-project-files)
* [Dependencies & Setup](#️-dependencies--setup)

---

## Project Structure

The project is organized into Python source files for the game, robot control, and trajectory recording, along with recorded robot trajectories stored as text files.

```text
.
├── Python
│   ├── cps_maingame.py
│   ├── cps_maingameoffline.py
│   ├── cps_moverobot.py
│   └── cps_recordarm.py
├── README.md
└── Txt
    ├── cps_o_pos4.txt
    ├── cps_o_pos5.txt
    ├── cps_x_pos1.txt
    ├── cps_x_pos2.txt
    └── cps_x_pos3.txt
```

| File | Purpose |
| --- | --- |
| `cps_maingame.py` | Main physical TicTacToe application combining camera vision, board detection, game logic, UI, and robot trajectory execution. |
| `cps_maingameoffline.py` | Standalone Pygame implementation for testing the game interface and logic without a camera or robot. |
| `cps_recordarm.py` | Places the SO-101 arm in freemove mode and records all six joint positions every 0.05 seconds. |
| `cps_moverobot.py` | Loads a recorded trajectory from a `.txt` file and replays it on the SO-101 arm at 20 Hz. |
| `cps_x_pos1.txt` | Recorded SO-101 trajectory for an `X` placement sequence. |
| `cps_x_pos2.txt` | Recorded SO-101 trajectory for an `X` placement sequence. |
| `cps_x_pos3.txt` | Recorded SO-101 trajectory for an `X` placement sequence. |
| `cps_o_pos4.txt` | Recorded SO-101 trajectory used by the main game for an `O` placement. |
| `cps_o_pos5.txt` | Recorded SO-101 trajectory used by the main game for an `O` placement. |
| `board_corners.json` | Generated after camera calibration and stores the four selected board corners. |

---

## Features
| Feature | Description |
|---|---|
| **Live board detection** | Using a camera and OpenCV. |
| **Perspective correction** | Transforms the detected board into a normalized 300 × 300 pixel view. |
| **Automatic cell classification** | Classifies cells as white `O`, black `X`, or empty. |
| **Win and draw detection** | Detects wins and draws across all rows, columns, and diagonals. |
| **SO-101 robotic arm control** | Controls the SO-101 robotic arm using the LeRobot framework. |
| **Recorded trajectory playback** | Enables repeatable physical robot movements using recorded trajectories. |
| **Six joint trajectory control** | Controls shoulder, elbow, wrist, and gripper movement across six joints. |
| **20 Hz robot recording and playback** | Provides smooth motion execution through 20 Hz recording and playback. |
| **Interactive board calibration** | Allows selection of the four board corners directly from the camera image. |
| **Live visual feedback** | Displays the camera stream and warped board detector in real time. |
| **Standalone Pygame mode** | Allows testing TicTacToe without the physical robot setup. |
| **Configurable turn timer** | Provides a configurable turn timer in the offline version. |
| **Replay, recalibration, and main menu controls** | Provides controls for replaying the game, recalibrating the board, and returning to the main menu. |

---

## Hardware

- SO-101 follower robotic arm
- Camera connected to the host computer
- Physical 3×3 TicTacToe board
- White `O` game pieces
- Black `X` game pieces
- Computer for vision processing and robot control

The current robot configuration uses the serial port `COM5` and the LeRobot follower ID `my_follower_arm`. These values can be changed in the Python scripts to match the local setup.

---

## Software

- Python 3
- OpenCV (`cv2`) for camera input, calibration, perspective transformation, image processing, and board detection
- NumPy for image and matrix operations
- LeRobot for SO 101 robotic arm communication and control
- Pygame for the standalone TicTacToe interface
- Python `csv` for robot trajectory files
- Python `threading` for non blocking robot execution during the live camera loop
- Python `json` for storing board calibration coordinates
- `msvcrt` for keyboard input in the Windows robot recording utility

---

## Project Files

---

## Dependencies & Setup