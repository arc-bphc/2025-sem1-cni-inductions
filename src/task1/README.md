Instead of the given `ros2 run teleop_twist_keyboard teleop_twist_keyboard` command I used `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/turtle1/cmd_vel` since the first one didn't work for me.

I had to remap cmd_vel so that the teleop_twist_keyboard node would know which topic to send the updates to.

Hope that's fine :)


