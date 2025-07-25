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

### 👇 Real Task (Storyline) 😄
> The Koopas are stranded on the Moon, in the domain of Dry-Bowser. With Mario on holiday with Peach, it's up to **R.O.B.** to rescue them by navigating lunar terrain and interacting with targets (Turtles).
<p align="center">
  <table>
    <tr>
      <td>
        <img width="616" height="333" alt="Map 1" src="https://github.com/user-attachments/assets/6b826c1c-890a-459f-9874-bbda8fe49c25" />
      </td>
      <td>
        <img width="616" height="333" alt="Map 2" src="https://github.com/user-attachments/assets/16d7b243-25c1-47a4-8a33-423a71570960" />
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
        <img width="616" height="333" alt="Map 1" src="https://github.com/user-attachments/assets/4910935e-200a-46f1-b29a-f84ce77ce710" />
        
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
