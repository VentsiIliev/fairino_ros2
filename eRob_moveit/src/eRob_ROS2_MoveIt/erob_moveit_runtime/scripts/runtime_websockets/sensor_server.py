from __future__ import annotations

import asyncio
import json
import logging
import threading


logger = logging.getLogger("erob_moveit_rest_server")


def start_sensor_websocket_server(*, supervisor, config, fallback_host: str, fallback_port: int):
    if not bool(getattr(config, "REST_WS_SENSOR_ENABLED", True)):
        logger.info("Sensor WebSocket disabled by REST_WS_SENSOR_ENABLED")
        return
    try:
        import websockets
        from websockets.exceptions import ConnectionClosed
    except Exception as exc:
        logger.error("Sensor WebSocket unavailable: %s", exc)
        return

    ws_host = str(getattr(config, "REST_WS_SENSOR_HOST", fallback_host) or fallback_host)
    ws_port = int(getattr(config, "REST_WS_SENSOR_PORT", int(fallback_port) + 3))

    async def sensor_handler(connection):
        request_obj = getattr(connection, "request", None)
        request_path = str(getattr(request_obj, "path", "") or getattr(connection, "path", "") or "")
        if request_path.split("?", 1)[0] != "/ws/sensors":
            await connection.close(code=1008, reason="unsupported websocket path")
            return
        supervisor.set_sensor_connected(True)
        await connection.send(json.dumps({"type": "hello", "endpoint": "/ws/sensors"}, separators=(",", ":")))
        try:
            async for message in connection:
                try:
                    event = json.loads(message)
                except (TypeError, ValueError):
                    await connection.send('{"type":"sensor_ack","accepted":false,"error":"invalid_json"}')
                    continue
                if not isinstance(event, dict):
                    await connection.send('{"type":"sensor_ack","accepted":false,"error":"invalid_event"}')
                    continue
                accepted = event.get("type") == "sensor_state" and supervisor.accept_sensor_event(event)
                await connection.send(json.dumps({
                    "type": "sensor_ack",
                    "accepted": bool(accepted),
                    "sensor": event.get("sensor"),
                    "sequence": event.get("sequence"),
                    "conditional_servo": supervisor.snapshot(),
                }, separators=(",", ":")))
        except ConnectionClosed:
            pass
        finally:
            supervisor.set_sensor_connected(False)

    async def run_server():
        async with websockets.serve(
            sensor_handler, ws_host, ws_port,
            ping_interval=10, ping_timeout=3, max_size=16 * 1024,
        ):
            logger.info("Sensor WebSocket running on ws://%s:%d/ws/sensors", ws_host, ws_port)
            await asyncio.Future()

    def websocket_thread():
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(run_server())
        except Exception as exc:
            logger.exception("Sensor WebSocket server failed: %s", exc)
        finally:
            loop.close()

    threading.Thread(target=websocket_thread, daemon=True, name="SensorWebSocketServer").start()
