# Unified Stealth BWB UAV System ⚡✈️

An advanced, end-to-end autonomous UAV system featuring a **6-DOF Blended Wing Body (BWB) Physics Engine**, low **Radar Cross Section (RCS)** modeling, real-time **Computer Vision HUD**, **Three.js 3D C2 Dashboard**, and a **ROS 2 Network Bridge**.

![License](https://img.shields.io/badge/License-MIT-blue.svg)
![Python](https://img.shields.io/badge/Python-3.10%2B-brightgreen.svg)
![ROS 2](https://img.shields.io/badge/ROS%202-Humble%2Fsup-orange.svg)
![FastAPI](https://img.shields.io/badge/FastAPI-0.100%2B-009688.svg)

---

## 🌟 Key Features

* **6-DOF Aerodynamic Physics Engine (`stealth_bwb_physics.py`)**:
  * Simulates non-linear flight dynamics, reflexed airfoils, and elevon differential control.
  * Achieves high lift-to-drag efficiency ($L/D \approx 18.0 - 22.0$) compared to traditional multirotors.
  * Models Li-Ion battery power consumption dynamically based on thrust.
  * Calculates real-time **Radar Cross Section (RCS)** values down to $-24\text{ dBsm}$ ($0.004\text{ m}^2$).

* **Real-time Computer Vision System (`camera_vision_system.py`)**:
  * Integrates real video feeds from USB/onboard cameras or generates a synthetic tactical optical HUD with an artificial horizon.
  * Overlays key flight telemetry directly onto the video stream.

* **Command & Control C2 Dashboard (`stealth_c2_server.py`)**:
  * Built with **FastAPI** and high-frequency WebSockets ($30\text{ Hz}$).
  * Interactive 3D visualization using **Three.js** and OrbitControls.
  * Flight mode control (`TAKEOFF`, `CRUISE`, `GLIDE`).
  * Instant high-resolution 3D screenshot exporter (`📸 CAPTURE PNG`).

* **ROS 2 Swarm & System Bridge (`ros2_swarm_bridge.py`)**:
  * Relays real-time pose (`geometry_msgs/PoseStamped`) and trajectory (`nav_msgs/Path`).
  * Features a graceful fallback mode when running outside of Linux/ROS 2 environments.

---

## 📐 Mathematical & Physics Foundations

### 1. Aerodynamic Efficiency
Lift and drag coefficients are modeled dynamically:
$$C_L = C_{L0} + C_{L\alpha}\alpha + C_{L\delta_e}\delta_e$$
$$C_D = C_{D0} + \frac{C_L^2}{\pi \cdot e \cdot AR}$$

Where:
* $AR = \frac{b^2}{S} \approx 7.6$ (Aspect Ratio)
* $e = 0.92$ (Oswald Efficiency Factor)
* $C_{D0} = 0.013$ (Parasitic Drag Factor without vertical stabilizers)

### 2. Low-RCS Radar Model
RCS is calculated based on edge alignment angles ($\pm 30^\circ$) to minimize radar reflection:
$$\sigma(\phi, \theta) = \sigma_{base} + \sigma_{spike} \cdot e^{-\frac{(|\phi| - 30^\circ)^2}{k}} \cdot \cos^2(\theta)$$

---

## 📁 Repository Structure

```text
Unified-Stealth-BWB-System/
├── stealth_bwb_physics.py     # 6-DOF Aerodynamic & Low-RCS Physics Engine
├── camera_vision_system.py   # Computer Vision Engine & Tactical HUD Overlay
├── stealth_c2_server.py      # FastAPI C2 Telemetry Server & Three.js 3D Web UI
├── ros2_swarm_bridge.py      # ROS 2 Publisher & Command Bridge
├── requirements.txt          # Python Dependencies
└── README.md                 # Project Documentation
```

---

## 🚀 Quick Start Guide

### 1. Prerequisites & Installation

Clone the repository and install the dependencies:

```bash
git clone https://github.com/your-username/Unified-Stealth-BWB-System.git
cd Unified-Stealth-BWB-System
pip install -r requirements.txt
```

### 2. Running the Stealth C2 System

Start the C2 Server and web interface:

```bash
python stealth_c2_server.py
```

Open your browser and navigate to:
```text
http://localhost:8000
```

### 3. Running the ROS 2 Bridge (Optional)

In a ROS 2 sourced environment (Humble / Jazzy):

```bash
python ros2_swarm_bridge.py
```

Inspect live ROS 2 telemetry topics:
```bash
ros2 topic echo /uav/stealth/pose
```

---

## 🛠️ Flight Modes

| Mode | Throttle | Elevon Trim | Description |
| :--- | :--- | :--- | :--- |
| **`TAKEOFF`** | 85% | Pitch Up ($-5^\circ$) | Maximum thrust climb phase. |
| **`CRUISE`** | 45% | Level | Balanced economic flight mode ($L/D > 11$). |
| **`GLIDE`** | 0% | Trimmed | Motor off, maximum energy conservation mode. |

---

## 🤝 Contributing

Contributions are welcome! Feel free to open an issue or submit a pull request for new features such as autonomous waypoint navigation, YOLOv8 target detection, or Gazebo SITL integration.

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.