import asyncio
import websockets
from picamera2 import Picamera2
import cv2
import struct

# Initialize camera
picam2 = Picamera2()

config = picam2.create_video_configuration(
    main={"size": (1280, 720), "format": "RGB888"},
    controls={"FrameRate": 30}
)

picam2.configure(config)
picam2.start()


async def stream(websocket):
    print("Client connected")

    try:
        while True:
            # Capture frame
            frame = picam2.capture_array()

            # Convert RGB → BGR (required for OpenCV)
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            # ✅ SINGLE flip (combine operations)
            frame = cv2.flip(frame, -1)   # same as flip 0 + 1

            # ❌ REMOVE heavy processing (optional)
            # If really needed, reduce strength
            # frame = cv2.GaussianBlur(frame, (3, 3), 0)

            # Encode (slightly lower quality for speed)
            ret, buffer = cv2.imencode(
                ".jpg",
                frame,
                [cv2.IMWRITE_JPEG_QUALITY, 60]
            )

            if not ret:
                continue

            data = buffer.tobytes()

            # Send size + frame together (faster)
            await websocket.send(struct.pack(">I", len(data)) + data)

            # Small yield instead of fixed delay
            await asyncio.sleep(0)

    except websockets.exceptions.ConnectionClosed:
        print("Client disconnected")


async def main():
    async with websockets.serve(
        stream,
        "0.0.0.0",
        8765,
        max_size=5_000_000
    ):
        print("Server started")
        await asyncio.Future()


asyncio.run(main())