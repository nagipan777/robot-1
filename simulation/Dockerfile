FROM ros:jazzy-ros-base-noble
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1
COPY create3_sim /opt/create3_ws/src/create3_sim
COPY irobot_create_msgs /opt/create3_ws/src/irobot_create_msgs
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential python3-colcon-common-extensions python3-rosdep \
    ros-jazzy-ros-gz ros-jazzy-teleop-twist-keyboard \
    xvfb x11vnc openbox novnc websockify supervisor x11-utils \
    libgl1-mesa-dri mesa-utils
RUN rosdep update --rosdistro jazzy \
    && rosdep install --from-paths /opt/create3_ws/src --ignore-src -r -y \
       --rosdistro jazzy \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/create3_ws
RUN source /opt/ros/jazzy/setup.bash \
    && MAKEFLAGS=-j2 colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && rm -rf build log
COPY worlds/room.sdf /opt/create3_ws/install/irobot_create_gz_bringup/share/irobot_create_gz_bringup/worlds/room.sdf
COPY supervisord.conf /etc/supervisor/conf.d/create3.conf
COPY start-simulation.sh /usr/local/bin/start-simulation
COPY webcam_bridge.py /usr/local/bin/webcam-bridge
COPY vision_turn_controller.py /usr/local/bin/vision-turn-controller
COPY unstamped_cmd_relay.py /usr/local/bin/unstamped-cmd-relay
COPY vision_controller.yaml /etc/vision-controller.yaml
COPY add_litter_marker_display.py /tmp/add-litter-marker-display.py
RUN chmod +x /usr/local/bin/start-simulation /usr/local/bin/webcam-bridge \
    /usr/local/bin/vision-turn-controller /usr/local/bin/unstamped-cmd-relay \
    && python3 /tmp/add-litter-marker-display.py \
    && rm /tmp/add-litter-marker-display.py \
    && echo 'source /opt/create3_ws/install/setup.bash' >> /root/.bashrc
EXPOSE 6080 8765
CMD ["/usr/bin/supervisord", "-n", "-c", "/etc/supervisor/supervisord.conf"]
