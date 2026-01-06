FROM ros:foxy-ros-base

SHELL ["/bin/bash", "-c"]

# Workspace
WORKDIR /home/dev_ws

# Create workspace
RUN mkdir -p /home/dev_ws/src

# Copy your ROS2 packages from the mono-repo
COPY ./ros2_ws/src /home/dev_ws/src

# Clone ROS TCP Endpoint (ROS2-compatible version)
RUN git clone -b ROS2v0.7.0 \
    https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git \
    /home/dev_ws/src/ros_tcp_endpoint

# Build workspace
RUN source /opt/ros/foxy/setup.bash && \
    colcon build

# Auto-source on shell start
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc && \
    echo "source /home/dev_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]
