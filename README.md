# 🤖 MoonBot Navigation — TESP '25 Robotics Project 🌕

**Obstacle Avoidance and Object Interaction on the Moon**

> A Space Robotics Lab project under Prof. K. Yoshida, Tohoku University  
> Developed during TESP ’25 by Andre Khoo, Alessio Borgi, Kristjan Tarantelli, and Rasmus Börjesson Dahlstedt

---

## 🚀 Project Overview

This project aims to design, build, and program an autonomous mobile robot capable of navigating a static sand terrain—simulating lunar conditions—and interacting with objects in its environment.

### 👇 Real Task (Storyline)
> The Koopas are stranded on the Moon, in the domain of Dry-Bowser. With Mario busy, it's up to **R.O.B.** to rescue them by navigating lunar terrain and interacting with targets (Turtles).

---

## 🛠️ Hardware Architecture

### 🤖 Robot Evolution
We iteratively prototyped 4 robot models, each improving on mobility, power, and design stability:
- **Tsukikage**: Lightweight, 2 motors, simple
- **Seigetsu**: 4 motors, powerful but heavy
- **Mikazuki**: Compact turning, unstable front
- **Tenshiko (Final)**: Loader-inspired, 1 motor linear gripper

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

## 📸 Demo & Media

> _Add images or video links here once available (YouTube, GIFs, etc.)_

---

## 📁 Repository Structure (suggested)

