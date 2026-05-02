# 🤖 Robotic Arm VR Digital Twin with Cycloidal Drive

## 📌 Overview

This project presents a **low-cost, high-precision robotic arm** integrated with a **Virtual Reality (VR) Digital Twin** and powered by a **custom-designed cycloidal drive mechanism**. The system combines mechanical design, embedded systems, and robotics software to achieve real-time control and visualization.

The robotic arm is designed and simulated in **Fusion 360**, physically manufactured using **3D printing**, and controlled using **ROS 2** with communication to an **ESP32 microcontroller**.

---

## 🚀 Key Features

* ⚙️ Custom **Cycloidal Drive** (High torque, low backlash)
* 🦾 Multi-DOF Robotic Arm Structure
* 🖥️ **ROS 2 Integration** for real-time control
* 🎮 **VR-based Digital Twin** using Unity (Linux compatible)
* 📡 Wireless Communication (PC ↔ ESP32)
* 💸 ~90% Cost Reduction compared to industrial arms
* 🧠 Scalable for AI/ML & research applications

---

## 🧩 System Architecture

### Hardware

* ESP32 Microcontroller
* Servo Motors (RDS3225)
* Stepper Motors
* Custom Cycloidal Gear Mechanism
* 3D Printed Arm Links
* Power Supply Module

### Software

* ROS 2 (Robot Operating System)
* Python (rclpy nodes)
* Unity (VR Digital Twin)
* Serial Communication Interface

---

## 🛠️ Requirements

### 🔧 Hardware Requirements

* ESP32 Development Board
* Servo Motors (RDS3225 recommended)
* Stepper Motor + Driver
* Power Supply (5V / appropriate current)

### 💻 Software Requirements

* Ubuntu 22.04 (Recommended)
* ROS 2 (Humble/Foxy)
* Python 3.10+
* Unity Hub + Unity Editor (for VR module)
* Git

---

## ⚙️ Installation & Setup

### 1️⃣ Clone the Repository

```bash
git clone https://github.com/rakesh001n/robotic-arm-vr-digital-twin-with-cycloidal-drive.git
cd robotic-arm-vr-digital-twin-with-cycloidal-drive
```

### 2️⃣ Setup ROS 2 Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
cp -r robotic-arm-vr-digital-twin-with-cycloidal-drive .
cd ..
colcon build
source install/setup.bash
```

### 3️⃣ Install Dependencies

```bash
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions
pip install pyserial numpy
```

### 4️⃣ ESP32 Setup

* Flash ESP32 with firmware from the project
* Connect via USB and identify port:

```bash
ls /dev/ttyUSB*
```

---

## ▶️ Running the System

> ⚠️ Open **separate terminals** for each node. This helps in debugging, viewing logs, warnings, and errors clearly.

### Terminal 1 – Kinematics Node

```bash
source ~/ros2_ws/install/setup.bash
ros2 run robotic_arm kinematics_node
```

### Terminal 2 – Robot Controller Node

```bash
source ~/ros2_ws/install/setup.bash
ros2 run robotic_arm robot_controller_node
```

### Terminal 3 – Pendant / Input Node

```bash
source ~/ros2_ws/install/setup.bash
ros2 run robotic_arm pendant_node
```

### Terminal 4 – Serial Communication Node (ESP32)

```bash
source ~/ros2_ws/install/setup.bash
ros2 run robotic_arm serialComms_node
```

---

## 🎮 How It Works

1. User interacts via VR or control interface
2. Pendant node sends desired positions
3. Kinematics node computes joint angles
4. Controller node processes commands
5. Serial node sends data to ESP32
6. ESP32 actuates motors
7. Feedback updates digital twin

---

## 📊 DH Parameters & Kinematics

The robotic arm follows Denavit-Hartenberg (DH) convention for kinematic modeling. Transformation matrices are used to compute forward kinematics for end-effector positioning.

*(Add your DH table here)*

---

## 🎯 Applications

* 🤖 Robotics Research & Education
* 🏭 Industrial Automation (Low-cost prototyping)
* 🎮 VR Teleoperation Systems
* 🏥 Remote Manipulation

---

## ❓ Why This Project?

* Massive cost reduction compared to industrial robots
* Combines mechanical design + embedded + ROS2 + VR
* Useful for research (Digital Twin, Control, AI)
* Fully customizable and open-source

---

## 📸 Demo & Media

*(Add images/videos here)*

---

## 🧑‍💻 Author

**Rakesh S D**
B.Tech Mechatronics Engineering
VIT Chennai

---

## 📜 License

Apache 2.0

---

## ⭐ Contributing

Contributions are welcome! Feel free to fork and submit pull requests.

---

## 📬 Contact

Use GitHub Issues for queries or collaboration.
