"""
YOLOv8 Oriented Bounding Box (OBB) TCP Detection Server
Universal Robots UR10e Pick-and-Place System

This server receives raw RGB images from the Webots camera controller via TCP,
runs inference using an Oriented Bounding Box (OBB) YOLOv8 model, calculates the
optimal grasp orientation angle, and returns the computed angle in degrees.

Author: Jasser Chaabane
"""

import socket
import struct
import sys
import numpy as np
from ultralytics import YOLO


def start_server(host='127.0.0.1', port=5050, model_path='best.pt'):
    # Load YOLO model
    print(f"Loading YOLOv8 OBB model from '{model_path}'...")
    try:
        model = YOLO(model_path)
    except Exception as e:
        print(f"Error loading YOLO model: {e}")
        sys.exit(1)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        server.bind((host, port))
        server.listen(2)
        print(f"YOLO Server running on {host}:{port}")
        print("Waiting for Webots simulation to connect...\n")
    except Exception as e:
        print(f"Failed to bind server to {host}:{port}: {e}")
        sys.exit(1)

    try:
        while True:
            conn, addr = server.accept()
            print(f"[TCP] Client connected from {addr}")

            try:
                while True:
                    # 1) Receive width, height, and data size (4 bytes each, big-endian)
                    header = conn.recv(12)
                    if not header or len(header) < 12:
                        print("[TCP] Client disconnected or incomplete header received.")
                        break

                    w, h, size = struct.unpack("!III", header)
                    print(f"[Detection] Frame received: {w}x{h} ({size} bytes)")

                    # 2) Receive full raw image payload
                    data = bytearray()
                    while len(data) < size:
                        packet = conn.recv(min(size - len(data), 65536))
                        if not packet:
                            break
                        data.extend(packet)

                    if len(data) != size:
                        print(f"[TCP] Error: received {len(data)} bytes, expected {size}")
                        break

                    # 3) Reconstruct RGB image array
                    img_np = np.frombuffer(data, dtype=np.uint8)
                    try:
                        img_np = img_np.reshape((h, w, 3))
                    except ValueError as e:
                        print(f"[Error] Reshape failed: {e}")
                        break

                    # 4) YOLOv8 OBB Inference
                    results = model.predict(img_np, verbose=False)
                    angle_deg = -999.0

                    # 5) Extract angle from Oriented Bounding Box
                    obb = results[0].obb if hasattr(results[0], 'obb') else None
                    if obb is not None and len(obb.xywhr) > 0:
                        first_obb = obb.xywhr.cpu().numpy()[0]  # [x_center, y_center, w, h, rotation]
                        rotation_rad = first_obb[4]
                        w_box, h_box = first_obb[2], first_obb[3]

                        # Convert rotation to degrees
                        angle_deg = float(np.degrees(rotation_rad))

                        # Normalize orientation for symmetric rectangular grasp
                        if h_box > w_box:
                            angle_deg += 90.0

                        # Map into [0, 180] range
                        if angle_deg > 180.0:
                            angle_deg -= 180.0
                        elif angle_deg < 0.0:
                            angle_deg += 180.0

                        confidence = float(obb.conf.cpu().numpy()[0]) if hasattr(obb, 'conf') and len(obb.conf) > 0 else 1.0
                        print(f"[Inference] Object detected! Angle={angle_deg:.2f}°, Conf={confidence:.2f}")
                    else:
                        print("[Inference] No object detected in frame.")

                    # 6) Send angle back to Webots camera controller as 8-byte double
                    conn.sendall(struct.pack("d", angle_deg))
                    print(f"[TCP] Sent angle {angle_deg:.2f}° to Webots controller\n")

            except (ConnectionResetError, BrokenPipeError):
                print("[TCP] Client connection lost.")
            finally:
                conn.close()
                print("[TCP] Waiting for next Webots client connection...\n")

    except KeyboardInterrupt:
        print("\n[Shutdown] Server stopped by user.")
    finally:
        server.close()
        print("[Shutdown] Server socket closed.")


if __name__ == "__main__":
    start_server()
