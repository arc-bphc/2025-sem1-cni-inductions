# Running ROS 2 Humble with Turtlesim on macOS (Apple Silicon: M1–M4) #

This guide documents an alternative method to complete Task 1: Getting Started with Turtlesim on macOS with Apple Silicon (M1–M4).
The original setup relies on docker-compose with noVNC (browser-based desktop), but it assumes x86 compatibility and often fails on Apple Silicon.
This workaround uses Docker + XQuartz for GUI rendering and works reliably on M1–M4 Macs.

---

## Steps ##

### 1. Install XQuartz ### 
ROS GUI applications (like turtlesim and rviz2) need an X server to render windows. On macOS, this is provided by XQuartz

- Download and install XQuartz
- Restart or log out/in after installation

### 2. Allow Localhost Connections ###

Run the following in a macOS terminal:

` xhost + 127.0.0.1 `

This grants Docker containers permission to render GUI windows via XQuartz.

### 3. Create a Custom Dockerfile ###

Instead of the repo’s docker-compose.yml, use this Dockerfile to ensure compatibility:

`FROM ros:humble`

`# Install prerequisites`
`RUN apt-get update && apt-get install -y \`
    `wget \`
    `lsb-release \`
    `gnupg2 \`
    `curl \`
    `&& rm -rf /var/lib/apt/lists/*`

`# Add Ignition (Gazebo) Fortress repository`
`RUN wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add - && \
    `sh -c "echo 'deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main' > /etc/apt/sources.list.d/gazebo-stable.list" `

`# Install Ignition Gazebo Fortress and ROS GUI packages`
`RUN apt-get update && apt-get install -y \`
    `ignition-fortress \`
    `ros-humble-turtlesim \`
    `ros-humble-teleop-twist-keyboard \`
    `ros-humble-rviz2 \`
    `&& rm -rf /var/lib/apt/lists/*`

`# Set up ROS environment`
`SHELL ["/bin/bash", "-c"]`
`RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc`

`CMD ["bash"]`

This includes:
- ROS 2 Humble base
- turtlesim & teleop_twist_keyboard
- RViz2 (for visualization tasks later)
- Ignition Gazebo Fortress (for simulation tasks)

### 4. Build the Container ###

From your repo folder:
`docker build -t ros2-humble-gazebo .`

### 5. Run the Container ###

Start the container with GUI support:

`docker run -it \`
    `--net=host \`
    `-e DISPLAY=host.docker.internal:0 \`
    `ros2-humble-gazebo`


`--net=host: lets container apps use host networking`

`-e DISPLAY=host.docker.internal:0: routes GUI rendering via XQuartz`

### 6. Launch Turtlesim ###

Inside the container:

`ros2 run turtlesim turtlesim_node`

A turtle window should appear through XQuartz. 🐢

### 7. Control the Turtle ###

Open another terminal (new shell into the same container):

`docker exec -it <container_id> bash`

Then run:

`ros2 run teleop_twist_keyboard teleop_twist_keyboard`

Now use your keyboard:

* i → forward *
* k → stop *
* j → turn left *
* l → turn right *

---

## Troubleshooting ##

No GUI appears?
1. Make sure XQuartz is running
2. Re-run `xhost + 127.0.0.1`
3. Inside container: check `echo $DISPLAY` → should be `host.docker.internal:0`

---

## Performance issues? ##

Some ROS/Gazebo packages are still optimized for x86
Occasional lag may happen on Apple Silicon

---

## Key Differences from Official Instructions ##

- Repo setup = docker-compose + noVNC (browser desktop) → fails on Apple Silicon
- My setup = Docker + XQuartz → runs GUIs natively on macOS
- Provides a base for future tasks with Gazebo and RViz2

---

## Conclusion ##

Although I couldn’t fully use the repo’s docker-compose setup, this alternative path:
-Shows familiarity with Docker, X server setup, and ROS 2 basics
-Provides a working solution for Apple Silicon users
Sets the stage for extending into Gazebo and RViz2 in later tasks
