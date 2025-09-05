# Example Dockerfile for ROS 2 Humble
    FROM ros:humble-ros-base

    # Install common ROS 2 development tools
    RUN apt-get update && apt-get install -y \
        build-essential \
        colcon-common-extensions \
        git \
        # Add any other necessary ROS 2 packages or dependencies
        && rm -rf /var/lib/apt/lists/*

    # Set up ROS 2 environment
    RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

    # Set a default command to keep the container running
