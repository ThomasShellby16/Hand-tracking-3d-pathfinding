# Hand-tracking-3d-pathfinding
This repository contains a Python application that utilizes MediaPipe for real-time 3D hand tracking and implements the A* algorithm for pathfinding in a 3D space. The application captures hand movements via webcam and visualizes a path from the camera position to the tracked hand position using Pygame.
# 3D Hand Tracking and A* Pathfinding

This project demonstrates 3D hand tracking using the MediaPipe library and implements the A* algorithm for pathfinding in a 3D grid space.

## Requirements

- Python 3.x
- OpenCV (`pip install opencv-python`)
- NumPy (`pip install numpy`)
- Mediapipe (`pip install mediapipe`)
- Pygame (`pip install pygame`)

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/your-username/3d-hand-tracking-a-star.git
   cd 3d-hand-tracking-a-star
2. Install the required Python packages:
   pip install -r requirements.txt
Usage
Run the main script:
python main.py
Use your webcam to capture hand movements. The program will track the movement of your hand in a 3D space and visualize an A* path from the camera position to the tracked hand position.
Features
Real-time 3D hand tracking using MediaPipe.
A* pathfinding algorithm implemented for 3D grid navigation.
Pygame visualization for displaying the tracked hand and the path.
Contributing
Contributions are welcome! Please fork the repository and create a pull request for any improvements or new features.

License
This project is licensed under the MIT License - see the LICENSE file for details.
