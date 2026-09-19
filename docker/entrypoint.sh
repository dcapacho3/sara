#!/bin/bash
# Bind-mounted dirs created by Docker on a fresh host come out root:root.
# Fix ownership so the container user can write ROS logs and rviz configs.
sudo chown -R "$(id -un):$(id -gn)" /home/"$(id -un)"/.ros /home/"$(id -un)"/.rviz2 2>/dev/null || true
mkdir -p /home/"$(id -un)"/.ros/log

exec "$@"
