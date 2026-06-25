#!/usr/bin/env bash
# Build the Duco TCP-pose reader (tools/read_tcp_pose.cpp).
#
# Links against the precompiled Duco SDK (libDucoCobotAPI.a) that ships with
# the duco_ros2_driver submodule. Thrift is bundled inside the .a, so the only
# extra system deps are pthread + OpenSSL.
#
# Usage:
#   tools/build_read_tcp_pose.sh
#   ./tools/read_tcp_pose [robot_ip]      # default 192.168.1.10
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DRV="$HERE/external/duco_ros2_driver/src/duco_ros_driver"

g++ -std=c++14 -O2 \
    -I "$DRV/include" \
    "$HERE/tools/read_tcp_pose.cpp" \
    "$DRV/lib/libDucoCobotAPI.a" \
    -lpthread -lssl -lcrypto \
    -o "$HERE/tools/read_tcp_pose"

echo "Built: $HERE/tools/read_tcp_pose"
