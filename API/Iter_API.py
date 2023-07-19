#!/usr/bin/env python3

import smbus                    # Library for I2C communication
import struct                   # Library for packing/unpacking binary data
import asyncio                  # Library for asynchronous programming
from fastapi import FastAPI, Response, BackgroundTasks  # FastAPI for web framework
from pydantic import BaseModel  # Pydantic for data validation
import uvicorn                  # Uvicorn for ASGI server
from fastapi.responses import StreamingResponse  # StreamingResponse for video streaming
import pyrealsense2 as rs       # Library for Intel RealSense camera
import numpy as np              # Library for numerical operations
import cv2                      # OpenCV library for image processing
import base64                   # Base64 encoding for image transmission


# Arduino I2C address
arduino_address_base = 0x09             # Base address for Arduino
arduino_address_camera_tilt = 0x08      # Address for camera tilt control

# Open the I2C bus
bus = smbus.SMBus(0)                    # Use SMBus 0 for Raspberry Pi 1 Model B

# Create FastAPI instance
app = FastAPI()

# Initialize Intel RealSense pipeline and configuration
pipe = rs.pipeline()                    # Create pipeline object
cfg = rs.config()                       # Create configuration object
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # Configure color stream
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)   # Configure depth stream
pipe.start(cfg)                         # Start the pipeline

# Define request body model
class DataRequest(BaseModel):
    value1: float
    value2: float
    value3: float


def generate_color_frame(frame):
    # Generate color frame from RealSense frame data

    color_frame = frame.get_color_frame()                  # Get color frame from the input frame

    color_image = np.asanyarray(color_frame.get_data())    # Convert color frame to NumPy array

    color_image = np.flipud(color_image)                   # Flip the image vertically
    color_image = np.fliplr(color_image)                   # Flip the image horizontally

    _, color_frame_encoded = cv2.imencode('.jpg', color_image)   # Encode color frame as JPEG image
    color_frame_base64 = base64.b64encode(color_frame_encoded).decode('utf-8')  # Convert encoded image to base64 string

    return color_frame_base64


def generate_depth_frames():
    # Generate depth frames from RealSense depth stream

    while True:
        frame = pipe.wait_for_frames()                      # Wait for frames from the RealSense pipeline
        depth_frame = frame.get_depth_frame()               # Get depth frame from the input frame

        depth_image = np.asanyarray(depth_frame.get_data())  # Convert depth frame to NumPy array

        depth_image = np.flipud(depth_image)                 # Flip the image vertically
        depth_image = np.fliplr(depth_image)                 # Flip the image horizontally

        depth_cm = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.5), cv2.COLORMAP_JET)  # Apply color map to depth image

        _, depth_frame_encoded = cv2.imencode('.jpg', depth_cm)  # Encode depth frame as JPEG image
        depth_frame_data = depth_frame_encoded.tobytes()          # Convert encoded image to bytes
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + depth_frame_data + b'\r\n\r\n')  # Yield frame data as multipart response


# API endpoint to receive data and send it over I2C
@app.post("/send_velocity/{Vx}/{Vy}/{Wz}")
async def send_velocity(Vx: float, Vy: float, Wz: float):
    # Transform base velocities to wheel velocities
    L = 10.7    # Distance between wheels (cm)
    R = 5.6     # Wheel radius (cm)

    Wa = (-Vx - np.sqrt(3)*Vy + 2*Wz*L) / (2*R)   # Calculate wheel velocity for motor A
    Wb = (-Vx + np.sqrt(3)*Vy + 2*Wz*L) / (2*R)   # Calculate wheel velocity for motor B
    Wc = (Vx + Wz*L) / R                          # Calculate wheel velocity for motor C

    # Limit wheel velocities to the range of -10 to 10
    Wa = max(min(Wa, 10.0), -10.0)
    Wb = max(min(Wb, 10.0), -10.0)
    Wc = max(min(Wc, 10.0), -10.0)

    # Pack float values into byte array
    values = struct.pack('fff', Wa, Wb, Wc)

    # Send data to Arduino
    bus.write_i2c_block_data(arduino_address_base, 0, list(values))

    # Return success message
    return {"message": "Data sent successfully."}


@app.get('/')
def index():
    return "Iter API"


@app.post('/color_frame')
async def color_frame():
    # Capture a color frame from the RealSense camera and return it as a response

    frame = pipe.wait_for_frames()                  # Wait for frames from the RealSense pipeline
    color_frame_data = generate_color_frame(frame)   # Generate color frame data
    return Response(content=color_frame_data, media_type='text/plain')


@app.get('/depth_stream')
async def depth_stream(background_tasks: BackgroundTasks):
    # Stream depth frames from the RealSense camera

    return StreamingResponse(generate_depth_frames(), media_type='multipart/x-mixed-replace; boundary=frame')


# API endpoint to receive data and send it over I2C
@app.post("/send_data_tilt_dualsense")
async def send_data_tilt_dualsense(data: int):
    # Prepare data to send over I2C
    angle = data
    angle_high_byte = (angle >> 8) & 0xFF
    angle_low_byte = angle & 0xFF

    # Send data to Arduino
    bus.write_i2c_block_data(arduino_address_camera_tilt, 1, [angle_high_byte, angle_low_byte])

    # Return success message
    return {"message": "Data sent successfully."}


# API endpoint to check from controller if robot is still connected
@app.post("/check_connection")
async def check_connection():
    # Return success message
    return {"message": "Still connected!"}


# Close the I2C connection when the server shuts down
@app.on_event("shutdown")
def shutdown_event():
    bus.close()


# Run the server
if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(uvicorn.run(app, host="192.168.0.34", port=8000))
    finally:
        loop.close()
