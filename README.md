# Robotic Navigation Simulation with LiDAR

This project implements a robotic simulation of a **mobile robot equipped with a LiDAR** sensor to explore an unknown environment. The system uses **OpenCV** for real-time visualization of both the map and occupancy grid.  

## Project Objectives
The robot explores an unknown environment using LiDAR to build a map. It must avoid obstacles, cover all accessible areas, and navigate efficiently.  


## Documentation
- For the raycasting : https://lodev.org/cgtutor/raycasting.html
- For the ArUco tags : https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
- To create ArUco tags : https://chev.me/arucogen/
- To Create the map we used a pixel art creator : https://www.pixilart.com/
- We used two maps : the first to do some tests and the real map
- We also used AI to help us debugging, write comments or if we got stuck o na problem.

## Key Features
- **Randomly generated environments** with obstacles of varying sizes.
- **Random robot initialization** with no prior knowledge of its position.
- **LiDAR simulation** using a raycasting algorithm 
- **Autonomous exploration** Using right-handed wall-following algorithm
   

***All robot decisions are based exclusively on LiDAR sensor data, with no direct or indirect access to the environment.*** 

## Dependencies
- **[OpenCV](https://opencv.org/)**: for graphical visualization.
- **[CMake](https://cmake.org/)**: for project configuration and build.


## Build
  ```
**Build and run**:
   ```bash
   cd Robot_LIDAR
   mkdir build
   cd build
   cmake ..
   make
   ./main
  ```
  Then add the map you want in the build folder
  Put the name map in the Simulation.cpp file (line 7)

## Usage
1. Run the program to randomly place the robot in a the map
2. Press key "1" or scan an ArUco tag with ID 0 to activate the manual mod
   Press key "2" or scan an ArUco tag with ID 1 to activate the wall-following mod
3. Once exploration is complete, press "escape" to close the simulation.

## Customization
You can chose whatever map you want.
Just put the .png file in the build folder and add the map name in the Simulation.cpp file (line 7)


## Contributors
- Raphaël Maul
- Alexandre Raffin
- Enzo Barro
- Tanguy Villequez
