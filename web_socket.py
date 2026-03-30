import asyncio
import websockets
import cv2
import struct
import threading
import firebase_request as fr

# Set IP to Firebase
fr.update_connected_ip()


class WebSocketServer:

    def __init__(self):
        self.latest_frame = None

    # 🔥 Receive frame from main.py
    def update_frame(self, frame):
        self.latest_frame = frame

    async def stream(self, websocket):
        print("Client connected")

        try:
            while True:
                if self.latest_frame is None:
                    await asyncio.sleep(0.01)
                    continue

                # Use latest frame (NO camera here)
                frame = self.latest_frame.copy()

                # 🔴 Apply transformations (UNCHANGED)
                frame = cv2.flip(frame, 1)

                # Improve clarity (UNCHANGED)
                frame = cv2.GaussianBlur(frame, (0, 0), 1)
                frame = cv2.addWeighted(frame, 1.5, frame, -0.5, 0)

                # Encode (UNCHANGED)
                ret, buffer = cv2.imencode(
                    ".jpg",
                    frame,
                    [cv2.IMWRITE_JPEG_QUALITY, 60]
                )

                if not ret:
                    continue

                data = buffer.tobytes()

                # Send size (UNCHANGED)
                await websocket.send(struct.pack(">I", len(data)))

                # Send frame (UNCHANGED)
                await websocket.send(data)

                await asyncio.sleep(0.05)

        except websockets.exceptions.ConnectionClosed:
            print("Client disconnected")

    def start(self):
        def run():
            async def main():
                async with websockets.serve(
                    self.stream,
                    "0.0.0.0",
                    8765,
                    max_size=5_000_000
                ):
                    print("Server started")
                    await asyncio.Future()

            asyncio.run(main())

        threading.Thread(target=run, daemon=True).start()