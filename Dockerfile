# import the ros2 humble 
FROM ros:humble-ros-base

RUN echo "hello"
RUN chmod 1777 /tmp
# Install dependencies needed for FaSTTUBe
RUN apt update && apt -y upgrade
RUN apt-get -y install git curl python3 python3-pip python3-colcon-common-extensions
RUN yes | pip install --upgrade pip
    ## install packaged dependencies
RUN yes | pip install setuptools==58.2 transformations empy==3.3.4 catkin-pkg
    ## install git sourced dependencies
RUN yes | pip install "fsd-path-planning[demo] @ git+https://git@github.com/FSLART/ft-fsd-path-planning.git"
RUN yes | pip install numba==0.59.1
# Create and go to ros2 src directory
RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws/src

# Clone needed repositories from host directory or git
RUN git clone https://github.com/FSLART/FaSTTUBe_planner.git
RUN git clone https://github.com/FSLART/lart_msgs.git -b dev

## Source ROS2 & Local build
WORKDIR /root/ros2_ws

    
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --parallel-workers 6 --symlink-install"

CMD /bin/bash -c "source /opt/ros/humble/setup.bash && \
    source install/setup.bash && \
    ros2 run path_planner my_node"
