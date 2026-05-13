# Home Service Robot - Depth-Informed Item Mapping

**A ROS-based reconstruction of my paper's home service robot system**

---

## ⚠️ Important Note: This is a Reconstruction

**I lost the original code from my paper.** This repository contains a **complete reconstruction** that I built based on:

1. The equations and algorithms I wrote in the paper
2. My methodology section detailing steps from object detection to navigation
3. Figures 1-5 showing geometrical relationships and test results
4. Table I with camera specifications

**What this means:**
- This is **not** my original code from when I ran the experiments
- It is a **faithful re-implementation** following my own mathematical framework
- All my equations (1-8) are implemented exactly as I described them
- The activation function testing recreates my Figures 3 and 4 methodology
- The distance calculation follows the 2-meter threshold I found in Figure 5

**Why I rebuilt it:** I lost the original implementation on an old hard drive, but my paper had enough detail that I could rebuild everything from scratch.

---

## 📝 About This Project

This project implements the algorithms from my paper *"Streamlining Navigation: Depth-Informed Item Mapping for Enhanced Home Service Robotics"*. I use an Astra RGB-D camera to detect when someone raises their hand, calculate their exact position on my robot's map (using my coordinate transform equations), and drive to a spot in front of them based on where they're looking.

### What My Robot Does

1. **Hand Gesture Detection** - Watches for raised hands using pose estimation
2. **Depth-Based Mapping** - Calculates where the person is using camera depth data
3. **Coordinate Transformation** - Converts camera coordinates to map coordinates (my Eq. 1-7)
4. **Gaze Estimation** - Figures out which way the person is looking
5. **Smart Navigation** - Drives to approach from the front so they can see my robot coming

## 🔄 How I Rebuilt This

### What I Still Had From My Paper

| My Paper Section | What I Used | Where I Put It |
|------------------|-------------|----------------|
| Methodology C - Object Detection | OpenVINO + EfficientHRNet approach | `find_poses.py` |
| Methodology D - Point Coordinate Calculation | My Equations 1 & 2 with camera parameters | `my_robot_mapping.py` |
| Methodology E - Coordinate Transformation | My Equations 3-7 and Figure 2 geometry | `my_robot_mapping.py` |
| Methodology F - Navigation Adjustment | My Equation 8 (circle generation) + head pose | `drive_to_person_v2.py` |
| Results B - Activation Function Testing | My distance thresholds from Figures 3 & 4 | `activation_function.py` |
| Results C - Calculation Section | My 2-meter accuracy limit from Figure 5 | `my_robot_mapping.py` |
| Table I | My camera FOV, resolution, range | `camera_params.yaml` |

### What I Had to Rebuild From Scratch

Since I lost my original code, I had to rebuild:

1. **The coordinate transformation pipeline** - I re-implemented the geometry from Figure 2 using my equations
2. **The activation function testing** - I built this based on the distance curves I published in Figures 3 and 4
3. **The navigation approach selection** - I recreated the "gaze-based point selection" I described in Section F
4. **The ROS node architecture** - I designed this to match the pipeline I outlined in Methodology sections A-G


## 🏗️ My Package Structure

```
home_service_robot/
├── scripts/                    # All my ROS nodes (reconstructed)
│   ├── my_robot_mapping.py    # My main mapping - Eq. 1-8
│   ├── find_poses.py          # My hand gesture detection
│   ├── head_pose_estimation.py # My gaze estimation (Section F)
│   ├── activation_function.py  # My tests from Figures 3 & 4
│   └── drive_to_person_v2.py   # My navigation + gaze approach
├── launch/
│   └── mapping_pipeline.launch # Launches all my nodes
├── config/
│   └── camera_params.yaml      # My Table I parameters
└── CMakeLists.txt
```

## 🔧 Requirements

### Hardware I Used
- **Astra RGB-D Camera** (any ROS-compatible depth camera should work)
- Robot with ROS navigation stack (move_base)
- Computer running ROS (I tested on Ubuntu 20.04 with ROS Noetic)

### Software Dependencies

```bash
# ROS packages I needed
sudo apt-get install ros-noetic-astra-camera
sudo apt-get install ros-noetic-move-base
sudo apt-get install ros-noetic-tf2-ros

# Python packages I used
pip install opencv-python
pip install numpy
pip install mediapipe  # optional, for better pose detection
```

## 🚀 How to Run My Code

1. **Clone my repository** into your catkin workspace:
```bash
cd ~/catkin_ws/src
git clone https://github.com/yourusername/home_service_robot.git
```

2. **Build the package**:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

3. **Make my scripts executable**:
```bash
chmod +x ~/catkin_ws/src/home_service_robot/scripts/*.py
```

## 🎮 How to Use My Robot

### Quick Start

Launch everything with one command:
```bash
roslaunch home_service_robot mapping_pipeline.launch
```

This starts:
- My Astra camera driver
- All my mapping and navigation nodes
- RViz so I can see what's happening

### What Should Happen

1. **Stand in front of my robot** (I found 1-4 meters works best)
2. **Raise your hand** - my robot should see this
3. My robot calculates where you are using depth data (my Eq. 1-2)
4. It transforms to map coordinates (my Eq. 3-7)
5. It draws a circle of approach points around you (my Eq. 8)
6. It figures out which way you're looking (my Section F head pose method)
7. My robot drives to the point in front of you


## 📊 The Math Behind My Robot (Direct from My Paper)

### My Equation 1 & 2 - Pixel to Real Coordinates:
```
x = (2*x'/w') * d * tan(β/2)
y = (2*y'/h') * d * tan(α/2)
```
This converts pixel coordinates (x', y') to real-world meters relative to my camera.

*This is from my Methodology D*

### My Equations 3-7 - Coordinate Transformation:
```
AC = sqrt(AM² + d²)
x_r = sin(90 - θ - arctan(AM/d)) * AC
y_r = cos(90 - θ - arctan(AM/d)) * AC
x_f = x + x_r
y_f = y + y_r
```
This transforms camera-relative coordinates to my global map coordinates.

*This is from my Methodology E and Figure 2*

### My Equation 8 - Navigable Circle:
```
(x - h)² + (y - k)² = r²
```
This generates points around the person that my robot can drive to.

*This is from my Methodology F*

## 📈 My Test Results (From My Paper)

From my Figures 3, 4, and 5:

| Distance | Detection Accuracy (Unblocked) | Detection Accuracy (Blocked) |
|----------|-------------------------------|------------------------------|
| 1-2m     | ~95%                          | ~90%                         |
| 2-3m     | ~90%                          | ~75%                         |
| 3-4m     | ~75%                          | ~40%                         |
| >4m      | Rapidly decreasing            | Unreliable                   |

**Best operating range:** 2-3 meters for unblocked scenarios (this matches what I found)
**Error threshold:** Minimal error within 2m, increases beyond (see my Figure 5)

## 🐛 Issues I Know About

1. **Skin color detection** isn't perfect in weird lighting (this is a simplified version of what I described)
2. **Hand detection** would work better with the proper OpenVINO model (my paper used EfficientHRNet)
3. **Distance accuracy** drops off after 2 meters (this is in my Figure 5 - it's expected)
4. **Move_base must be running** for navigation to work
5. **Head pose estimation** is simplified - my paper's CNN would work better

## 🔄 Differences Between This Code and My Original Paper

Since I lost my original code, here's what's different:

| My Original Implementation | This Reconstruction | Why |
|---------------------------|---------------------|-----|
| My original EfficientHRNet | MediaPipe fallback + skin detection | I lost my original model files |
| My original ROS architecture | Standard ROS node structure | I rebuilt from my methodology |
| My specific camera calibration | Generic Astra parameters | I used my Table I specs |
| My original head pose CNN | Simplified geometric estimation | I only had my algorithm description left |

**But the core equations and algorithms are exactly how I wrote them in my paper.**

## 📚 My References

My own paper: *"Streamlining Navigation: Depth-Informed Item Mapping for Enhanced Home Service Robotics"* - Pokman Han, Pui Ching Middle School, Macau SAR, China

Paper available here: https://ieeexplore.ieee.org/document/10612955

Citation: P. Han, "Streamlining Navigation: Depth-Informed Item Mapping for Enhanced Home Service Robotics," 2024 4th Asia-Pacific Conference on Communications Technology and Computer Science (ACCTCS), Shenyang, China, 2024, pp. 587-591, doi: 10.1109/ACCTCS61748.2024.00109.

Other papers I cited:
- SLAM fundamentals [11-13]
- EfficientHRNet for pose estimation [14-15]
- OpenVINO inference engine


## 👨‍🔬 Author

Me - Pokman Han - Pui Ching Middle School, Macau SAR, China | University of California, Santa Barbara

## 🙏 Acknowledgments

- OpenVINO Team for the pose estimation models I referenced
- ROS Communnity for the navigation stack I built on
- Astra Camera Developers for the RGB-D driver I used
- Teachers at Pui Ching Middle School's FABLAB for assisting me throughout this project

---


**Questions? Found a bug?** Email me: pokmanhan@gmail.com
