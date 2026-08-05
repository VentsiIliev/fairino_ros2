import asyncio
import websockets

async def main():
  async with websockets.connect("ws://localhost:5001/ws/state") as ws:
      while True:
          print(await ws.recv())

asyncio.run(main())