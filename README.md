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

## wetexplorer_sensors – IMU Processing Template

The `wetexplorer_sensors` package is a ready-to-use ROS 2 module designed to process IMU data. It already contains all the setup required to receive IMU messages from the robot, process them, and republish the results. Inside this package, the main file is located at:



### Where the script is
```
ros2_ws/src/wetexplorer/wetexplorer_sensors/wetexplorer_sensors/imu_processing_node.py
```
Inside this file, only modify the function:
```python
def imu_callback(self, msg: Imu):
    # -------- algorithm goes here --------
```
This node is a normal Python class. If needed, add your own members, helper functions, or parameters elsewhere in the class. The subscription and publication wiring is already set up.

---

## Build the package

Every time you modify the script, rebuild:

```bash
colcon build
# or, to build only this package:
colcon build --packages-select wetexplorer_sensors
```

Source the workspace if you opened a new shell:

```bash
source install/setup.bash
```

---

## Run the node

Open a new terminal inside the container or your ROS 2 environment:

```bash
ros2 run wetexplorer_sensors imu_processing_node
```

Use the plotting tools described earlier (PlotJuggler) to verify outputs, e.g. angular velocities, linear accelerations, and orientation signals.

---

## Recording and replaying data (rosbag)

Reference: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html

Choose a folder for your data:
```bash
cd /ros2_ws/src/data
```
Recording:
```bash
ros2 bag record -o robot_not_moving /imu/data
# General form:
# ros2 bag record -o {file_name} {topics...}
```
Playback:
```bash
ros2 bag play robot_not_moving
# General form:
# ros2 bag play {file_name}
```

---

## wetexplorer_sensors – Forward Kinematics Template

Inside 'wetexplorer_sensors` package, the main file is located at:



### Where the script is
```
ros2_ws/src/wetexplorer/wetexplorer_sensors/wetexplorer_sensors/odometry_real.
ros2_ws/src/wetexplorer/wetexplorer_sensors/wetexplorer_sensors/odometry_sim.py
```
There are two nodes since the real robot publishes the angular velocities of the sprockets as a string, which is not the standard for the simulation. Since we have different types of messages we need to handle the simulation script differently at the beginning. However, the algorithm block you paste in each script should be the same. Try it in the simualtion first and then we will test it with real data.

---

## Build the package

Every time you modify the script, rebuild:

```bash
colcon build
# or, to build only this package:
colcon build --packages-select wetexplorer_sensors
```

Source the workspace if you opened a new shell:

```bash
source install/setup.bash
```

---

## Run the node

Open a new terminal inside the container or your ROS 2 environment:

```bash
ros2 run wetexplorer_sensors odometry_sim
```
or

```bash
ros2 run wetexplorer_sensors odometry_real
```

You can debug this node by plotting Vx vs time and Omega vs time. Also X pos vs Y pos. For that, use Plotjuggler.


f you want to get more familiar with the kinematic modeling this node is based on, check out the section 3.2.2 Forward kinematic models in the book "Introduction to Autonomous Mobile Robots":
https://www.ucg.ac.me/skladiste/blog_13268/objava_56689/fajlovi/Introduction%20to%20Autonomous%20Mobile%20Robots%20book.pdf

---
