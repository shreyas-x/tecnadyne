import serial
import websockets
import asyncio

def parser(msg):
    msgArr = msg.split(",")
    reply = ""
    match msgArr[0]:
        case "status":
            reply = "Status OK"
        
        case "start":
            reply = f"Started at {msgArr[1]} and {msgArr[2]}"

    return reply

async def handler(websocket, path):
    async for data in websocket:
        reply = parser(data)
        await websocket.send(reply)

startServer = websockets.serve(handler, "localhost", 8000)

asyncio.get_event_loop().run_until_complete(startServer)
asyncio.get_event_loop().run_forever()