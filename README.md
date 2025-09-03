# 2025 Sem 1 CNI Induction Tasks

Welcome to the induction program for our club.
This repository will guide you through multiple tasks that help you get started with **Git, Docker, ROS 2, and basic programming concepts**.

The tasks are progressive and will gradually increase in difficulty. You are not expected to complete all the tasks within the timeframe. However, try to maximize on how much you learn during these tasks since this knowledge will help you out in your round 3!

As always, best of luck and happy coding!

---

## 📑 Contents

1. [How to Submit Your Work](#how-to-submit-your-work)
2. [Task 1: Getting Started with Turtlesim](#task-1-getting-started-with-turtlesim)
3. [Task 2: Writing Your First ROS 2 Node](#task-2-writing-your-first-ros-2-node)
4. [Task 3: Swarm Algorithm with Turtlesim](#task-3-swarm-algorithm-with-turtlesim)

---

## How to Submit Your Work

1. **Fork this repository** into your own GitHub account.

   * [How to fork a repo](https://docs.github.com/en/get-started/quickstart/fork-a-repo)

2. Complete the task(s) as instructed.

3. Create a folder in the src folder of your forked repository:

   ```
   task<task-number>
   ```

   Example:

   ```
   task1
   ```

4. Place your deliverables (screenshots, code, etc.) in this folder.

4.1 Run `git commit -m "message"` to commit your changes.

4.2 Run `git push` to push your changes to GitHub.

5. **Submit a Pull Request (PR) to the main repository**.

   * **PR Title format:**

     ```
     NAME [ID_NUMBER]
     ```

     Example:

     ```
     Archisman Das [2025B3PS0478H]
     ```

   * **PR Description format:**

     ```
     name: Your Full Name
     ID: Your ID Number
     email: Your Institute Email
     ```

6. Wait for review and feedback.

---

## Task 1: Getting Started with Turtlesim

In this exercise, you will:

* Run a ROS 2 container with Docker.
* Access a Linux desktop in your browser using noVNC.
* Launch the turtlesim simulator.
* Control the turtle using your keyboard.

The goal is not only to make the turtle move but also to get familiar with the tools we use: **Git, Docker, Docker Compose, ROS 2, and the Linux terminal**.

---

### Prerequisites

1. **Install Git**

   * [Install Git](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git)

2. **Install Docker**

   * [Install Docker](https://docs.docker.com/get-docker/)

3. **Install Docker Compose**

   * [Install Docker Compose](https://docs.docker.com/compose/install/)

4. **Learn the Basics of the Terminal**

   * [Linux Command Line Basics](https://ubuntu.com/tutorials/command-line-for-beginners#1-overview)

---
### Steps

#### 1.1. Fork this repository. 

**[DO NOT PULL OR MAKE CHANGES ON THIS REPO]**

#### 1.2. Clone the Repository. 

**[Make sure that you are cloning your own fork and not this repo.]**

```bash
git clone <repository-url>
cd <repository-folder>
```

#### 2. Start the Containers

```bash
docker compose up -d --build
```

This will:

* Build the ROS 2 Humble container with turtlesim installed.
* Start the noVNC container (desktop accessible in browser).

#### 3. Open the noVNC Desktop

Go to [http://localhost:8080](http://localhost:8080)

You should see a Linux desktop inside your browser.

#### 4. Open an Interactive Shell in the ROS 2 Container

```bash
docker compose exec ros2 /bin/bash
```

You are now inside the container.

#### 5. Run Turtlesim

```bash
ros2 run turtlesim turtlesim_node
```

The turtle window should appear in your noVNC browser desktop.

#### 6. Control the Turtle

On your **host machine**, open another terminal and again enter the container:

```bash
docker compose exec ros2 /bin/bash
```

Then run:

```bash
ros2 run turtlesim turtle_teleop_key
```

You can now move the turtle with your keyboard.

---

### Deliverable

* Draw a simple shape (circle or square) using your keyboard.
* Take a screenshot of your running turtlesim window in noVNC.
* Place the screenshot in:

  ```
  src/task1
  ```
* Submit via a **Pull Request** as explained in [How to Submit Your Work](#how-to-submit-your-work).

---

### Troubleshooting

1. **No turtle window in noVNC**

   * Ensure you launched turtlesim *inside the container*.
   * Check you are on `http://localhost:8080`.

2. **`exec: "ros2": executable file not found`**

   * Ensure you ran:

     ```bash
     docker compose exec ros2 /bin/bash
     ```

3. **"no such service" error with `docker compose exec`**

   * Check containers with:

     ```bash
     docker compose ps
     ```
   * Restart if needed:

     ```bash
     docker compose up -d
     ```

4. **Teleop not working**

   * Ensure the terminal running `teleop_twist_keyboard` is active.

---

## Task 2: Writing Your First ROS 2 Node

In this task, you will learn how **ROS 2 topics, nodes, and messages** work by writing your own ROS 2 node to control the turtle.
You will move beyond teleoperation and instead write code that makes the turtle **trace out shapes automatically**.

---

### Objectives

1. Understand the basics of **ROS 2 nodes, topics, and messages**.
2. Create a new **ROS 2 workspace** dedicated for this task.
3. Write a ROS 2 Python node to:

   * Generate a list of points on a **circle**.
   * Move the turtle through these points, tracing out the circle.
   * **BONUS:** Modify your code to draw **a different shape** of your choice.

---

### Steps

#### 1. Open a Shell in the ROS 2 Container

```bash
docker compose exec ros2 /bin/bash
```

#### 2. Create a New Workspace

Inside the `/src` folder, create a new workspace for Task 2:

```bash
cd /src
mkdir -p task2
cd task2
colcon build
```
You need to run this again to use your custom pkg
```bash
source install/setup.bash
```

#### 3. Create a New Package

We’ll use Python for simplicity.

```bash
cd /src/task2
ros2 pkg create --build-type ament_python turtle_draw --dependencies rclpy geometry_msgs
```

This will create a new package called `turtle_draw` with Python dependencies.

---

### 4. Helpful Resources

Before you start coding, read these short tutorials:

* [ROS 2 Concepts: Nodes, Topics, Messages](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools.html)
* [Writing a Publisher and Subscriber (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
* [geometry\_msgs/Twist Message](https://docs.ros.org/en/crystal/Tutorials/Topics/Understanding-ROS2-Topics.html#ros2-interface-show)

---

### 5. Your Task

Inside the `turtle_draw` package:

1. Create a Python node file:

   ```bash
   cd /src/task2/turtle_draw/turtle_draw
   touch circle_drawer.py
   chmod +x circle_drawer.py
   ```

2. Write code that does the following:

   * Generate a list of points on a circle using the equations:

     ```
     x = r * cos(theta)
     y = r * sin(theta)
     ```

   * Move the turtle from one point to the next by publishing velocity commands to the `/turtle1/cmd_vel` topic.

3. Add an entry point in `setup.py` so you can run it using:

   ```bash
   ros2 run turtle_draw circle_drawer
   ```

	**NOTE: Make sure to check out /src/example-submission for guidance on how to setup entrypoints and structure your code!**

	 NOTE: You will need to run `colcon build` and `source install/setup.bash` again to make sure you are running the latest version of your node.

	 NOTE: Make sure that turtlesim_node is running before running your node.

---

### 6. BONUS Challenge

Modify your code to draw **any other shape** (square, spiral, star, etc.) by changing your point generation function.

---

### Deliverables

* Push your workspace inside:

  ```
  src/task2
  ```

  (only include your package, not build/install/logs folders).

* Add screenshots of your turtle drawing shapes in noVNC.

* If you want to submit a video you can upload it to YouTube, and add a link to it as a comment on your pull request.

* Submit via a **Pull Request** as described in [How to Submit Your Work](#how-to-submit-your-work).

---

### Troubleshooting

1. **Cannot edit files created by ros2 pkg create**

   * Ensure you are on your host computer.
   * Run `sudo chown -R $USER:$USER src/task2` in the root directory to change the ownership of the created files.

2. **`Package 'turtle_draw' not found`**

   * Ensure you ran:

     ```bash
	 colcon build
	 source install/setup.bash
     ```
---

## Task 3: Swarm Algorithm with Turtlesim

### Objective

In this task, you are required to implement a **swarm behavior** using at least **five turtles** in the `turtlesim` simulator.
This is a slightly open-ended task designed to encourage creativity and experimentation.

One well-known algorithm for simulating swarm behavior is the **Boids algorithm**, which models the flocking behavior of birds. You can read more about it here:
🔗 [Boids — Wikipedia](https://en.wikipedia.org/wiki/Boids)

---

### Requirements

1. **Spawn at least 5 turtles** inside `turtlesim`.

   * Use the `/spawn` service to create multiple turtles.
   * Each turtle should move independently according to your algorithm.

2. **Implement a swarm algorithm** where turtles show some form of coordinated movement.

   * Examples of behaviors: flocking, chasing a leader, forming patterns, or avoiding collisions.
   * Boids is strongly suggested, but other creative approaches are welcome.

3. **Use ROS 2 nodes and topics** to control the turtles.

   * Publish velocity commands to `/turtleX/cmd_vel`.
   * Optionally, subscribe to `/turtleX/pose` for position feedback.

---

### Deliverables

* **Code**: Submit your ROS 2 package containing the implementation.
* **Demonstration Video**: Upload a short demo (1–3 minutes) of your turtles executing the swarm algorithm.

  * Upload as an **unlisted YouTube video** and include the link in your submission.

---

### Suggested Approach

1. Research the **Boids algorithm** and its three rules:

   * **Separation**: avoid crowding neighbors.
   * **Alignment**: move in the same direction as neighbors.
   * **Cohesion**: move toward the average position of neighbors.
2. Start by spawning 5 turtles in random positions.
3. Implement each rule step by step and test.
4. Tune parameters (speed, turning rate, neighbor distance) until the behavior looks realistic.
