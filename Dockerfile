FROM ros:humble

# Install turtlesim, teleop, rviz, gazebo
RUN apt-get update && apt-get install -y \
    ros-humble-turtlesim \
    ros-humble-teleop-twist-keyboard \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Source ROS env automatically
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

CMD ["bash"]