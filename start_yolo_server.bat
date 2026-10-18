@echo off
echo ==========================================================
echo  Starting YOLOv8 Oriented Bounding Box Detection Server   
echo  Listening on 127.0.0.1:5050                             
echo ==========================================================
echo.
echo Make sure Python dependencies are installed:
echo   pip install -r requirements.txt
echo.
cd yolo_server
python yolo_server.py
pause
