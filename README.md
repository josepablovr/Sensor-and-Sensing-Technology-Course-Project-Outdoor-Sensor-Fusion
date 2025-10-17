# Sensor and Sensing Technology Course Project – Outdoor Sensor Fusion

Welcome to this course project!  
This repository serves as a **baseline for implementing sensor fusion tasks** toward achieving **accurate localization** on the real WetExplorer robot.

---

## 🧰 Prerequisites

### 1. Install Docker
Follow the official installation guide:  
👉 [Docker Installation Guide](https://docs.docker.com/engine/install/)

### 2. Add Your User to the Docker Group
This allows you to run Docker commands without `sudo`:  
👉 [Post-Installation Steps for Linux](https://docs.docker.com/engine/install/linux-postinstall/)

After this, log out and back in to apply the changes.

---

## 📦 Setting Up the Project

### 1. Clone the Repository
```bash
git clone https://github.com/josepablovr/Sensor-and-Sensing-Technology-Course-Project-Outdoor-Sensor-Fusion
cd Sensor-and-Sensing-Technology-Course-Project-Outdoor-Sensor-Fusion
```

### 2. Build the Docker Image
This step will take **20–30 minutes** the first time:
```bash
./build.sh
```

Once it finishes, it will open a container automatically.

---

## 🧑‍💻 Working Inside the Container

A **container** is an isolated environment hosting all dependencies for the project.  
To open a terminal connected to the container at any time:
```bash
./run.sh
```

This works even if it’s your first time running the container.

To stop the container, simply press **Ctrl+D** in the first console window you opened.

> 💡 **Tip:** The repository folder on your computer is directly mapped into the container.  
> Any changes made in Visual Studio Code (or your editor) are immediately reflected inside the container.

---

## ⚙️ Building the Workspace

ROS 2 projects are organized as **packages** that need to be compiled whenever changes are made.

### 1. Build the Workspace
```bash
colcon build
```

### 2. Source the Workspace
```bash
source install/setup.bash
```

This step ensures ROS 2 recognizes your freshly built packages.

---

## 🚀 Running the Simulation

To start the robot simulation:
```bash
ros2 launch wetexplorer_gazebo sim_launch.py
```

Once Gazebo opens, click **“Play”** (bottom left corner).

<img src="imgs/gazebo.gif" alt="Gazebo Simulation" width="400"/>

### 🎮 Controls
- **W, A, X, D** — move the robot  
- **S** — stop  
- **Q, E, Z, C** — perform turns and linear movements  

> 📝 Make sure the simulation window is active while using the keyboard controls.

---

## 🧭 Visualization Tools

You’ll need a **second terminal** (also inside the Docker container).  
Open a new terminal window in the same directory and run:
```bash
./run.sh
```

### 1. RViz — Visualization and Debugging

General visualization:
```bash
rviz2
```

Pre-configured sessions:
- **Robot TF structure:**
  ```bash
  ros2 launch wetexplorer_description rviz.py
  ```
- **IMU orientation & gravity vector:**
  ```bash
  ros2 launch wetexplorer_description imu.py
  ```

---

## 📈 Plotting Sensor Data

### Option 1: RQT Plot

Run:
```bash
ros2 run rqt_plot rqt_plot
```

Add topics manually, for example:
```
/imu/data/linear_acceleration/x
/imu/data/angular_velocity/x
```
Click the **“+”** button to add more plots.

> ⚠️ Note: Orientation is published as a quaternion, so it’s not directly interpretable in roll–pitch–yaw form here.

---

### Option 2: PlotJuggler

PlotJuggler provides a more advanced visualization, including **roll, pitch, and yaw**.

Make sure the simulation is running, then:
```bash
ros2 run plotjuggler plotjuggler
```

- Press **Start**
- Select the topic `imu/data/`
- Expand the topic tree on the left and **drag variables** to the white area

#### 💡 Tips
- You can adjust the **buffer size** (the visible time window).
- You can **record data** to a CSV file:
  1. Run your experiment until the desired data is in the buffer.  
  2. Press **Stop**
  3. Choose **CSV Exporter**
  4. Set the time limits and save your file.

---

## ✅ Summary

You are now ready to:
- Build and launch the WetExplorer simulation
- Visualize the robot and sensor data in RViz
- Plot and analyze IMU data in RQT Plot or PlotJuggler

Enjoy exploring sensor fusion and visualization tools in ROS 2! 🚀