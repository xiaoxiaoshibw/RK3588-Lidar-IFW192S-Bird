# RK3588 human-fall recognition container (isolated from slam-localization driver container)
# Online build:
#   docker build -f deploy/rk3588_containers/human_fall/Dockerfile -t lidar-human-fall:latest deploy/rk3588_containers/human_fall
# Offline RK3588 build reusing existing local slam-localization image (already has ROS Noetic + numpy):
#   docker build --build-arg BASE_IMAGE=<slam-localization-image> -t lidar-human-fall:latest deploy/rk3588_containers/human_fall
ARG BASE_IMAGE=ros:noetic-ros-base
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    ROS_MASTER_URI=http://127.0.0.1:11311 \
    ROS_IP=127.0.0.1 \
    HUMAN_FALL_BUNDLE_DIR=/opt/lidar-human-fall/bundle \
    HUMAN_FALL_MODE=instant_start \
    HUMAN_FALL_MOUNT_PITCH_DEG=26.0 \
    HUMAN_FALL_MOUNT_ROLL_DEG=0.0 \
    HUMAN_FALL_DISABLE_IMU=1 \
    HUMAN_FALL_PHYSICAL_V1=0 \
    HUMAN_FALL_SINGLE_PERSON_LOCK=0 \
    HUMAN_FALL_DIAGNOSTICS=0 \
    ROS_MASTER_WAIT_SEC=60

RUN if ! bash -lc 'source /opt/ros/noetic/setup.bash 2>/dev/null && python3 -c "import numpy, rospy, sensor_msgs.msg, std_msgs.msg"' >/dev/null 2>&1; then \
        apt-get update && \
        apt-get install -y --no-install-recommends python3-numpy ros-noetic-sensor-msgs ros-noetic-std-msgs && \
        rm -rf /var/lib/apt/lists/*; \
    fi

WORKDIR /opt/lidar-human-fall
COPY entrypoint.sh /opt/lidar-human-fall/entrypoint.sh
COPY healthcheck.py /opt/lidar-human-fall/healthcheck.py
RUN sed -i 's/\r$//' /opt/lidar-human-fall/entrypoint.sh && \
    chmod +x /opt/lidar-human-fall/entrypoint.sh

HEALTHCHECK --interval=10s --timeout=4s --start-period=15s --retries=3 \
    CMD bash -lc 'source /opt/ros/noetic/setup.bash && python3 -B /opt/lidar-human-fall/healthcheck.py'

ENTRYPOINT ["/opt/lidar-human-fall/entrypoint.sh"]
CMD []
