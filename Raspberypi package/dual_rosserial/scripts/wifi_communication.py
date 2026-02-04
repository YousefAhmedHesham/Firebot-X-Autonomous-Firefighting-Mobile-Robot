#!/usr/bin/env python3
import asyncio
import websockets

async def handler(websocket):
    async def send_hello():
        message = "hello from mahmoud\n hello from yousef \nshawaf mo2rf"
        while True:
            await websocket.send(message)
            await asyncio.sleep(1)  # Send every second

    async def receive_messages():
        async for message in websocket:
            print("Received:", message)
            await websocket.send("Pi received: " + message)

    # Run both send and receive concurrently
    await asyncio.gather(send_hello(), receive_messages())

async def main():
    async with websockets.serve(handler, "0.0.0.0", 8765):
        await asyncio.Future()  # Run forever

asyncio.run(main())
