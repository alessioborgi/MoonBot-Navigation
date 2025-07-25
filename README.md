# 🤖 MoonBot Navigation 🌕

**Copyright © 2025 Alessio Borgi, Andre Khoo, Kristjan Tarantelli, Rasmus Börjesson Dahlstedt**

---

**Robot Navigation, Obstacle Avoidance and Interaction on the Moon**

This project aims to design, build, and program an autonomous mobile robot capable of navigating a static sand terrain, simulating lunar conditions, and interacting with objects in its environment. 

> A Space Robotics Lab project under Prof. K. Yoshida, Tohoku University  
---

## 🚀 Project Overview

Our task was to design and build an autonomous mobile robot capable of navigating a simulated lunar environment, avoiding obstacles, and interacting with specific target objects placed on the terrain.

This mission required overcoming several key challenges. First, we had to build the robot entirely from scratch, optimizing its mechanical design for a sandy, uneven surface that caused slippage and instability. This meant carefully selecting components, configuring the drivetrain, and iteratively refining the robot’s physical structure.

The second major challenge was to implement a robust navigation and control system. This involved generating reliable paths through the terrain—despite limited sensor data and unpredictable motion—and finally enabling the robot to detect and interact with objects (turtles) using onboard vision and a custom-built gripper.

<p align="center">
  <img width="972" height="376" alt="architecture" src="https://github.com/user-attachments/assets/4f9f935e-7525-47d4-a69c-73f2f1fa633d" />
</p>

### 👇 Real Task (Storyline) 😄
> The Koopas are stranded on the Moon, in the domain of Dry-Bowser. With Mario on holiday with Peach, it's up to **R.O.B.** to rescue them by navigating lunar terrain and interacting with targets (Turtles).
<p align="center">
  <table>
    <tr>
      <td>
        <img width="616" height="333" alt="Map 1" src="https://github.com/user-attachments/assets/6b826c1c-890a-459f-9874-bbda8fe49c25" />
      </td>
      <td>
        <img width="563" height="302" alt="map_story" src="https://github.com/user-attachments/assets/a700bc44-b0fe-4e28-89cf-d167906bb126" />
      </td>
    </tr>
  </table>
</p>


---

## 🛠️ Hardware Architecture

### 🤖 Robot Evolution
We iteratively prototyped 4 robot models, each improving on mobility, power, and design stability:
- **Tsukikage**: Lightweight, 2 motors, simple
- **Seigetsu**: 4 motors, powerful but heavy
- **Mikazuki**: Compact turning, unstable front
- **Tenshiko (Final)**: Loader-inspired, 1 motor linear gripper

The final robot is the following: 

<p align="center">
  <table>
    <tr>
      <td>
        <img width="616" height="333" alt="Map 1" src="https://github.com/user-attachments/assets/edbd2462-bbca-4bbe-8247-a54c02609f8b" />
      </td>
      <td>
        <img width="616" height="333" alt="Map 1" src="https://github.com/user-attachments/assets/d210ad52-6cd2-45a2-bf1e-96abaa3c81f6" />
      </td>
    </tr>
  </table>
</p>

### 🧠 Electronics Stack
- **Raspberry Pi** — Sensor & ML processing
- **EV3 Brick** — Motor control
- **Camera Module** — Turtle detection
- **Motors** — Controlled via ROS2 or manually

---

## 📡 Software Architecture

### 🧭 Path Planning (ROS2 Simulator)
- **Input**: Binary Map (simulated satellite view)
- **Planner**: Dijkstra Algorithm (suboptimal/fallback pathing)
- **Controller**: PD controller outputs velocity `(v)` and angular velocity `(w)`
- **Visualization**: RViz

To enable autonomous navigation, we developed a computer vision and planning pipeline that converts a raw image of the terrain into a structured navigation map used by our path planner.

##### Capture from Satellite
A top-down image of the terrain is acquired, simulating satellite imagery of a lunar environment with sand and rocks.

##### Binary Map Generation 
The image is converted into a binary map using basic thresholding. Obstacles appear in black, and free space appears in white—simplifying the environment for planning algorithms.
<p align="center">
  <img width="976" height="243" alt="img_to_binary" src="https://github.com/user-attachments/assets/fdd7cf85-a59e-41ad-b5bc-7b429c33cfa4" />
</p>

##### Distance Map Calculation & Retraction Algorithm 
Using the binary map, a distance transform is applied to compute a distance map. This map indicates how far each point is from the nearest obstacle. Brighter regions are safer and more navigable. Finally, we apply a retraction process on the distance map to generate a safe navigation zone, pulling the valid path away from obstacles while preserving reachability. This map serves as the input for our Dijkstra-based path planner.



<p align="center">
  <img width="681" height="353" alt="retraction_map" src="https://github.com/user-attachments/assets/165df5ac-2760-4bbe-9081-b85cfa2ab202" />
</p>

### ⚠️ Issues Encountered
- Communication delay between EV3 and Raspberry Pi
- ROS not viable in real-time due to latency
- Resort to **precomputed paths** and **camera-only close-range navigation**

---

## 🧠 Machine Learning for Object Detection

- **Camera**: GC0308 CMOS, 2MP, 30FPS
- **Dataset**: 240 labeled images (manual masks)
- **Platform**: Roboflow for data augmentation + training
- **Model**: Simple object detection used for turtle identification and aimpoint guidance
- **Issues**: Blurry images → failed detections

---

## 🎮 Navigation & Object Interaction

### Short-Range Tracking
- Use visual servoing to keep turtle centered in view
- Simple heuristic-based steering

### Gripper Mechanism
- Loader-style linear gripper (must stay off ground while navigating)
- Difficulty aligning due to nondeterministic slip/glide on sand

---

## 📊 Results

- Successfully navigated via precomputed path
- Detected and interacted with turtles in ~60–70% of trials
- Some failures due to visual errors or drift during final approach

---

## 🔭 Future Work

### Hardware Improvements
- Eliminate EV3, use RPi-only architecture
- Use high-precision motors
- Add distance sensors (e.g., ultrasonic or LIDAR)

### Software Improvements
- Speed up ML model inference
- Direct integration of ROS2 with motor control

---

## 👥 Team

| Name                         | Affiliation                                | Country |
|------------------------------|---------------------------------------------|---------|
| Andre Khoo                  | Nanyang Technological University            | 🇸🇬 Singapore |
| Alessio Borgi               | Sapienza University of Rome (AI & Robotics) | 🇮🇹 Italy |
| Kristjan Jurij Tarantelli   | Sapienza University of Rome (AI & Robotics) | 🇮🇹 Italy |
| Rasmus Börjesson Dahlstedt | Chalmers University of Technology           | 🇸🇪 Sweden |

---
