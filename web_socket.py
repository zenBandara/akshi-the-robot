import asyncio
import websockets
from picamera2 import Picamera2
import cv2
import struct
import firebase_request as fr

# Set IP to Firebase

fr.update_connected_ip()

# Initialize camera
picam2 = Picamera2()

#High resolution - 1280, 720 Low - 640, 480
config = picam2.create_video_configuration(
    main={"size": (1280, 720), "format": "RGB888"}
)

picam2.configure(config)
picam2.start()

async def stream(websocket):
    print("Client connected")

    try:
        while True:
            # Capture frame
            frame = picam2.capture_array()

            # Convert RGB → BGR
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            # 🔴 Apply transformations
            # frame = cv2.flip(frame, 0)   # upside down
            # frame = cv2.flip(frame, 1)   # mirror left-right
            # frame = cv2.flip(frame, 1)
            frame = cv2.flip(frame, -1)
            frame = cv2.flip(frame, 1)

            # Improve clarity
            frame = cv2.GaussianBlur(frame, (0, 0), 1)
            frame = cv2.addWeighted(frame, 1.5, frame, -0.5, 0)

            # Encode
            ret, buffer = cv2.imencode(
                ".jpg",
                frame,
                [cv2.IMWRITE_JPEG_QUALITY, 60] #70
            )

            if not ret:
                continue

            data = buffer.tobytes()

            # Send size
            await websocket.send(struct.pack(">I", len(data)))

            # Send frame
            await websocket.send(data)

            await asyncio.sleep(0.05)

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