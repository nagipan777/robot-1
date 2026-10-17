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
RUN apt-get update \
    && apt-get install -y --no-install-recommends wmctrl \
    && rm -rf /var/lib/apt/lists/*
COPY worlds/room.sdf /opt/create3_ws/install/irobot_create_gz_bringup/share/irobot_create_gz_bringup/worlds/room.sdf
COPY worlds/room_camera.sdf /opt/create3_ws/install/irobot_create_gz_bringup/share/irobot_create_gz_bringup/worlds/room_camera.sdf
COPY supervisord.conf /etc/supervisor/conf.d/create3.conf
COPY start-simulation.sh /usr/local/bin/start-simulation
COPY start-camera-bridge.sh /usr/local/bin/start-camera-bridge
COPY start-camera-activation.sh /usr/local/bin/start-camera-activation
COPY start-camera-view.sh /usr/local/bin/start-camera-view
COPY start-camera-mjpeg.sh /usr/local/bin/start-camera-mjpeg
COPY sim_camera_mjpeg.py /usr/local/bin/sim-camera-mjpeg
COPY known_target_test.py /usr/local/bin/known-target-navigation-test
COPY known_target_test.py /usr/local/bin/known_target_test.py
COPY multi_target_test.py /usr/local/bin/multi-target-navigation-test
COPY synthetic_detection_test.py /usr/local/bin/synthetic-detection-coordinate-test
COPY yolo_navigation_test.py /usr/local/bin/yolo-navigation-test
COPY oscillation_diagnostics.py /usr/local/bin/oscillation-diagnostics
COPY known_target_test.json /etc/known-target-test.json
COPY multi_target_test.json /etc/multi-target-test.json
COPY sim_camera_correspondences.json /etc/sim-camera-correspondences.json
COPY litter-model.sh /usr/local/bin/litter-model
COPY calibration-marker.sh /usr/local/bin/calibration-marker
COPY start-vision-controller.sh /usr/local/bin/start-vision-controller
COPY models/demo_litter /opt/pai/models/demo_litter
COPY models/demo_litter_baseline /opt/pai/models/demo_litter_baseline
COPY models/demo_litter_paper /opt/pai/models/demo_litter_paper
COPY models/demo_litter_bottle /opt/pai/models/demo_litter_bottle
COPY models/calibration_marker /opt/pai/models/calibration_marker
COPY webcam_bridge.py /usr/local/bin/webcam-bridge
COPY map_frame_adapter.py /usr/local/bin/map-frame-adapter
COPY map_frame_adapter.yaml /etc/map-frame-adapter.yaml
COPY target_manager.py /usr/local/bin/target_manager.py
COPY vision_turn_controller.py /usr/local/bin/vision-turn-controller
COPY unstamped_cmd_relay.py /usr/local/bin/unstamped-cmd-relay
COPY roomba_dry_run_adapter.py /usr/local/bin/roomba-dry-run-adapter
COPY cmd_vel_output_adapter.py /usr/local/bin/cmd-vel-output-adapter
COPY cmd_vel_output_adapter.yaml /etc/cmd-vel-output-adapter.yaml
COPY differential_drive.py /usr/local/bin/differential_drive.py
COPY roomba_tcp_transport.py /usr/local/bin/roomba_tcp_transport.py
COPY roomba_bridge_tcp.py /usr/local/bin/roomba-bridge-tcp
COPY roomba_wheel_lift_test.py /usr/local/bin/roomba-wheel-lift-test
COPY roomba_wheel_lift_test_core.py /usr/local/bin/roomba_wheel_lift_test_core.py
COPY recording_status.py /usr/local/bin/recording-status
COPY presentation_visualizer.py /usr/local/bin/presentation-visualizer
COPY vision_controller.yaml /etc/vision-controller.yaml
COPY vision_controller.sim_camera.yaml /etc/vision-controller.sim-camera.yaml
COPY vision_controller.integration.yaml /etc/vision-controller.integration.yaml
COPY add_litter_marker_display.py /tmp/add-litter-marker-display.py
RUN chmod +x /usr/local/bin/start-simulation /usr/local/bin/webcam-bridge \
    /usr/local/bin/start-camera-bridge /usr/local/bin/start-camera-view \
    /usr/local/bin/start-camera-activation \
    /usr/local/bin/start-camera-mjpeg /usr/local/bin/sim-camera-mjpeg \
    /usr/local/bin/known-target-navigation-test /usr/local/bin/multi-target-navigation-test \
    /usr/local/bin/synthetic-detection-coordinate-test \
    /usr/local/bin/yolo-navigation-test \
    /usr/local/bin/oscillation-diagnostics \
    /usr/local/bin/litter-model /usr/local/bin/start-vision-controller \
    /usr/local/bin/calibration-marker \
    /usr/local/bin/map-frame-adapter /usr/local/bin/vision-turn-controller \
    /usr/local/bin/unstamped-cmd-relay /usr/local/bin/roomba-dry-run-adapter \
    /usr/local/bin/cmd-vel-output-adapter \
    /usr/local/bin/roomba-bridge-tcp /usr/local/bin/roomba-wheel-lift-test \
    /usr/local/bin/recording-status /usr/local/bin/presentation-visualizer \
    && python3 /tmp/add-litter-marker-display.py \
    && rm /tmp/add-litter-marker-display.py \
    && echo 'source /opt/create3_ws/install/setup.bash' >> /root/.bashrc
EXPOSE 6080 8765 8766
CMD ["/usr/bin/supervisord", "-n", "-c", "/etc/supervisor/supervisord.conf"]
