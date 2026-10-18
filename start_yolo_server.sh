#!/usr/bin/env bash
# Script to launch YOLOv8 Detection Server for Webots UR10e Pick-and-Place
set -e

echo "=========================================================="
echo " Starting YOLOv8 Oriented Bounding Box Detection Server   "
echo " Listening on 127.0.0.1:5050                             "
echo "=========================================================="

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/yolo_server"

if ! command -v python3 &> /dev/null; then
    echo "Error: python3 could not be found. Please install Python 3."
    exit 1
fi

python3 yolo_server.py
