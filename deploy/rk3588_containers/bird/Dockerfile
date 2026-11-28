# RK3588 native C++17 bird recognition container (read-only Foxglove subscriber + web UI)
# Online build:  docker build -f deploy/rk3588_containers/bird/Dockerfile -t lidar-bird:latest .
# Offline RK3588 build reusing existing local image (already has g++, python3, numpy):
#   docker build --build-arg BASE_IMAGE=<slam-localization-image> -f deploy/rk3588_containers/bird/Dockerfile -t lidar-bird:latest .
ARG BASE_IMAGE=ubuntu:20.04
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    BIRD_FOXGLOVE_URL=ws://127.0.0.1:8765 \
    BIRD_INPUT_TOPIC=/innolidar_points \
    BIRD_HTTP_HOST=0.0.0.0 \
    BIRD_HTTP_PORT=8895 \
    BIRD_TRACKER_LIB=/app/libbird_tracker.so

# Install toolchain and numpy only when missing in BASE_IMAGE (enables 100% offline builds on RK3588).
RUN if ! command -v g++ >/dev/null 2>&1 || ! python3 -c "import numpy" >/dev/null 2>&1; then \
        apt-get update && \
        apt-get install -y --no-install-recommends g++ python3 python3-numpy ca-certificates && \
        rm -rf /var/lib/apt/lists/*; \
    fi

WORKDIR /app

COPY firmware/esp32s3_bird_core/ /app/core/
COPY deploy/rk3588_containers/bird/bird_core_c_api.cpp /app/bird_core_c_api.cpp
COPY deploy/rk3588_containers/bird/native_tracker.py /app/native_tracker.py
COPY deploy/rk3588_containers/bird/pointcloud_grid.py /app/pointcloud_grid.py
COPY deploy/rk3588_containers/bird/foxglove_client.py /app/foxglove_client.py
COPY deploy/rk3588_containers/bird/bird_service.py /app/bird_service.py
COPY web/s3_live_server.py /app/s3_live_server.py
COPY web/s3_live.html /app/s3_live.html
COPY bird_audit/raw_capture.py /app/bird_audit/raw_capture.py

# Compile unchanged firmware/esp32s3_bird_core into a native ARM64/x86_64 shared library.
RUN touch /app/bird_audit/__init__.py && \
    g++ -O3 -std=c++17 -shared -fPIC -Wall -Wextra -Werror \
        -I/app/core \
        /app/bird_core_c_api.cpp \
        /app/core/grid_algorithm.cpp \
        /app/core/tracker.cpp \
        /app/core/geometry.cpp \
        -o /app/libbird_tracker.so && \
    python3 -B -c "import sys; sys.path.insert(0, '/app'); import native_tracker, pointcloud_grid, foxglove_client, bird_service; print('BIRD_IMAGE_VERIFY_OK')"

EXPOSE 8895

HEALTHCHECK --interval=10s --timeout=3s --start-period=10s --retries=3 \
    CMD python3 -B -c "import os, urllib.request; port=os.environ.get('BIRD_HTTP_PORT','8895'); urllib.request.urlopen(f'http://127.0.0.1:{port}/healthz', timeout=2).read()"

ENTRYPOINT ["python3", "-u", "-B", "/app/bird_service.py"]
CMD []
