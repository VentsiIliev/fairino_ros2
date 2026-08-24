#!/usr/bin/env python3
"""
OpenAPI/Swagger documentation constants for the REST server.
"""


OPENAPI_SPEC = {
    "openapi": "3.0.3",
    "info": {
        "title": "ZeroErr Robot Runtime API",
        "version": "1.0.0",
        "description": "REST API for startup polling, robot motion, state, tools, safety walls, drives, and interlocks.",
    },
    "servers": [{"url": "/"}],
    "paths": {
        "/health": {"get": {"tags": ["startup"], "summary": "HTTP/runtime health"}},
        "/startup/status": {"get": {"tags": ["startup"], "summary": "Startup progress for frontend polling"}},
        "/status": {"get": {"tags": ["state"], "summary": "Robot execution, queue, runtime, drive, and interlock status"}},
        "/state/snapshot": {"get": {"tags": ["state"], "summary": "Combined UI state snapshot"}},
        "/state/kinematics": {"get": {"tags": ["state"], "summary": "Current TCP position, velocity, and acceleration"}},
        "/position/current": {"get": {"tags": ["state"], "summary": "Current TCP position in active WorkObject"}},
        "/position/base_tcp": {"get": {"tags": ["state"], "summary": "Current TCP position in robot base"}},
        "/position/flange": {"get": {"tags": ["state"], "summary": "Current flange position"}},
        "/velocity/current": {"get": {"tags": ["state"], "summary": "Current TCP velocity"}},
        "/move/linear": {"post": {"tags": ["motion"], "summary": "Queue or execute a linear move"}},
        "/move/ptp": {"post": {"tags": ["motion"], "summary": "Queue or execute a point-to-point move"}},
        "/execute/path": {"post": {"tags": ["motion"], "summary": "Execute a waypoint path"}},
        "/execute/sequence": {"post": {"tags": ["motion"], "summary": "Execute mixed motion segments"}},
        "/execute/ordered_motion_chain": {"post": {"tags": ["motion"], "summary": "Execute ordered motion chain"}},
        "/execute/ordered_motion_chain/prepare": {"post": {"tags": ["motion"], "summary": "Prepare an ordered chain from an explicit future start pose"}},
        "/execute/ordered_motion_chain/prepared/{plan_id}/execute": {"post": {"tags": ["motion"], "summary": "Authorize execution of a prepared chain"}},
        "/execute/ordered_motion_chain/prepared/{plan_id}": {"get": {"tags": ["motion"], "summary": "Prepared chain status"}, "delete": {"tags": ["motion"], "summary": "Discard a prepared chain"}},
        "/execute/ordered_motion_chain/status": {"get": {"tags": ["motion"], "summary": "Ordered motion chain status"}},
        "/unwind/joint6": {"post": {"tags": ["motion"], "summary": "Unwind joint 6"}},
        "/jog/joint": {"post": {"tags": ["motion"], "summary": "Jog one robot joint"}},
        "/servo/cartesian/start": {
            "post": {
                "tags": ["servo"],
                "summary": "Start Cartesian Servo session",
            }
        },

        "/servo/cartesian/update": {
            "post": {
                "tags": ["servo"],
                "summary": "Update Cartesian Servo velocity command",
            }
        },

        "/servo/cartesian/stop": {
            "post": {
                "tags": ["servo"],
                "summary": "Stop Cartesian Servo session",
            }
        },
        "/jog": {"post": {"tags": ["motion"], "summary": "Jog along one robot axis"}},
        "/servojog/start": {"post": {"tags": ["servo"], "summary": "Start continuous ServoJog"}},
        "/servojog/stop": {"post": {"tags": ["servo"], "summary": "Stop continuous ServoJog"}},
        "/stop": {"post": {"tags": ["motion"], "summary": "Stop active motion and clear queued work"}},
        "/reachability/pose": {"post": {"tags": ["planning"], "summary": "Validate pose reachability from a start pose"}},
        "/workobject/set": {"post": {"tags": ["frames"], "summary": "Set active work object origin"}},
        "/workobject/registry": {"get": {"tags": ["frames"], "summary": "Get workobject registry"}},
        "/workobject/registry/{user_id}": {
            "post": {
                "tags": ["frames"],
                "summary": "Update one workobject registry entry",
                "parameters": [{"name": "user_id", "in": "path", "required": True, "schema": {"type": "integer"}}],
            }
        },
        "/workobject/active": {
            "get": {"tags": ["frames"], "summary": "Get active workobject"},
            "post": {"tags": ["frames"], "summary": "Set active workobject"},
        },
        "/tool/registry": {"get": {"tags": ["tools"], "summary": "Get tool registry"}},
        "/tool/registry/{tool_id}": {
            "post": {
                "tags": ["tools"],
                "summary": "Update one tool registry entry",
                "parameters": [{"name": "tool_id", "in": "path", "required": True, "schema": {"type": "integer"}}],
            }
        },
        "/tool/active": {
            "get": {"tags": ["tools"], "summary": "Get active tool"},
            "post": {"tags": ["tools"], "summary": "Set active tool"},
        },
        "/safety/walls/enabled": {"get": {"tags": ["safety"], "summary": "Check whether safety walls are enabled"}},
        "/safety/walls/status": {"get": {"tags": ["safety"], "summary": "Safety wall status"}},
        "/safety/walls/enable": {"post": {"tags": ["safety"], "summary": "Enable safety walls"}},
        "/safety/walls/disable": {"post": {"tags": ["safety"], "summary": "Disable safety walls"}},
        "/io/digital_output": {"post": {"tags": ["io"], "summary": "Set digital output"}},
        "/drive/status": {"get": {"tags": ["drives"], "summary": "Drive operation-enable status"}},
        "/drive/enable": {"post": {"tags": ["drives"], "summary": "Request and verify drive operation enable"}},
        "/drive/disable": {"post": {"tags": ["drives"], "summary": "Request and verify drive operation disable"}},
        "/motion/interlock/status": {"get": {"tags": ["interlock"], "summary": "Motion interlock status"}},
        "/motion/interlock/reset": {"post": {"tags": ["interlock"], "summary": "Reset motion interlock"}},
    },
}


def _json_schema(schema_type="object", **extra):
    return {"type": schema_type, **extra}


def _json_request_body(example: dict | None = None, required: bool = False) -> dict:
    media_type = {"schema": _json_schema("object")}
    if example is not None:
        media_type["example"] = example
    return {
        "required": required,
        "content": {
            "application/json": media_type,
        },
    }


def _apply_openapi_details():
    default_response = {
        "description": "JSON response",
        "content": {
            "application/json": {
                "schema": _json_schema("object"),
            },
        },
    }
    for path_item in OPENAPI_SPEC["paths"].values():
        for operation in path_item.values():
            operation.setdefault("responses", {"200": default_response})

    post_examples = {
        "/move/linear": {
            "position": [300, 0, 300, 180, 0, 0],
            "tool": 0,
            "user": 0,
            "vel": 20,
            "acc": 20,
            "blocking": False,
            "trajectory_optimizer": "RUCKIG",
        },
        "/move/ptp": {
            "position": [300, 0, 300, 180, 0, 0],
            "tool": 0,
            "user": 0,
            "vel": 20,
            "acc": 20,
            "blocking": False,
        },
        "/execute/path": {
            "path": [[300, 0, 300, 180, 0, 0], [320, 0, 300, 180, 0, 0]],
            "vel": 20,
            "acc": 20,
            "blocking": False,
            "orientation_mode": "constant",
        },
        "/execute/sequence": {
            "segments": [
                {"motion_type": "linear", "position": [300, 0, 300, 180, 0, 0], "vel": 20, "acc": 20},
                {"motion_type": "ptp", "position": [320, 0, 300, 180, 0, 0], "vel": 20, "acc": 20},
            ],
            "tool": 0,
            "user": 0,
            "blocking": False,
        },
        "/execute/ordered_motion_chain": {
            "segments": [
                {"type": "linear", "label": "approach", "position": [300, 0, 300, 180, 0, 0], "vel": 20, "acc": 20}
            ],
            "tool": 0,
            "user": 0,
            "blocking": True,
            "trajectory_optimizer": "RUCKIG",
        },
        "/unwind/joint6": {"blocking": True, "queue_if_busy": True, "vel": 20, "acc": 20},
        "/jog": {"axis": "X", "direction": "PLUS", "step": 10, "vel": 10, "acc": 10, "frame": "user", "user": 0, "tool": 0},
        "/servojog/start": {"axis": "X", "direction": "PLUS", "linear_mm_s": 10, "angular_deg_s": 3, "frame": "user", "user": 0, "tool": 0},
        "/servojog/stop": {},
        "/servo/cartesian/start": {
            "frame": "user",
            "user": 0,
            "tool": 1,
        },

        "/servo/cartesian/update": {
            "linear_mm_s": [0.0, 0.0, -10.0],
            "angular_deg_s": [0.0, 0.0, 0.0],
        },

        "/servo/cartesian/stop": {},
        "/reachability/pose": {
            "target_position": [300, 0, 300, 180, 0, 0],
            "start_position": [280, 0, 300, 180, 0, 0],
            "tool": 0,
            "user": 0,
        },
        "/workobject/set": {"origin": [0, 0, 0, 0, 0, 0], "user_id": 0},
        "/workobject/registry/{user_id}": {
            "name": "WOBJ_1",
            "transform": [0, 0, 0, 0, 0, 0],
            "persist": False,
        },
        "/workobject/active": {"user_id": 1},
        "/tool/registry/{tool_id}": {
            "name": "TOOL_1",
            "transform": [0, 0, 170, 0, 0, 0],
            "persist": False,
        },
        "/tool/active": {"tool_id": 1},
        "/io/digital_output": {"port": 0, "value": 1},
        "/stop": {},
        "/safety/walls/enable": {},
        "/safety/walls/disable": {},
        "/drive/enable": {},
        "/drive/disable": {},
        "/motion/interlock/reset": {},
    }
    for path, example in post_examples.items():
        operation = OPENAPI_SPEC["paths"].get(path, {}).get("post")
        if operation is not None:
            operation["requestBody"] = _json_request_body(example, required=bool(example))

    OPENAPI_SPEC["paths"]["/jog"]["post"]["requestBody"] = {
        "required": True,
        "content": {
            "application/json": {
                "schema": {
                    "type": "object",
                    "required": ["axis", "direction", "step", "vel", "acc"],
                    "properties": {
                        "axis": {"type": "string", "enum": ["X", "Y", "Z", "RX", "RY", "RZ"]},
                        "direction": {"type": "string", "enum": ["PLUS", "MINUS"]},
                        "step": {"type": "number"},
                        "vel": {"type": "number"},
                        "acc": {"type": "number"},
                        "frame": {"oneOf": [{"type": "string", "enum": ["base", "user", "tool"]}, {"type": "integer"}]},
                        "user": {"type": "integer"},
                        "tool": {"type": "integer"},
                    },
                },
                "example": {"axis": "X", "direction": "PLUS", "step": 10, "vel": 10, "acc": 10, "frame": "user", "user": 0, "tool": 0},
            },
        },
    }

    OPENAPI_SPEC["paths"]["/jog/joint"]["post"]["requestBody"] = {
        "required": True,
        "content": {
            "application/json": {
                "schema": {
                    "type": "object",
                    "required": ["joint", "direction", "step"],
                    "properties": {
                        "joint": {
                            "type": "string",
                            "enum": ["J1", "J2", "J3", "J4", "J5", "J6", "Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5", "Joint_6"],
                        },
                        "direction": {"type": "string", "enum": ["PLUS", "MINUS", "POSITIVE", "NEGATIVE"]},
                        "step": {"type": "number", "description": "Joint increment in degrees"},
                        "vel": {"type": "number"},
                        "acc": {"type": "number"},
                        "blocking": {"type": "boolean"},
                    },
                },
                "example": {"joint": "J6", "direction": "PLUS", "step": 5, "vel": 10, "acc": 10, "blocking": True},
            },
        },
    }

    OPENAPI_SPEC["paths"]["/servojog/start"]["post"]["requestBody"] = {
        "required": True,
        "content": {
            "application/json": {
                "schema": {
                    "type": "object",
                    "required": ["axis", "direction"],
                    "properties": {
                        "axis": {"type": "string", "enum": ["X", "Y", "Z", "RX", "RY", "RZ"]},
                        "direction": {"type": "string", "enum": ["PLUS", "MINUS", "POSITIVE", "NEGATIVE"]},
                        "linear_mm_s": {"type": "number"},
                        "angular_deg_s": {"type": "number"},
                        "vel": {"type": "number"},
                        "acc": {"type": "number"},
                        "frame": {"oneOf": [{"type": "string", "enum": ["base", "user", "tool"]}, {"type": "integer"}]},
                        "user": {"type": "integer"},
                        "tool": {"type": "integer"},
                    },
                },
                "example": {"axis": "X", "direction": "PLUS", "linear_mm_s": 10, "angular_deg_s": 3, "frame": "user", "user": 0, "tool": 0},
            },
        },
    }

    drive_command_responses = {
        "200": {"description": "Command verified against current drive status"},
        "202": {"description": "Command accepted but drive status has not matched yet"},
        "500": {"description": "Command failed or returned unexpected state"},
        "503": {"description": "Hardware not ready"},
    }
    OPENAPI_SPEC["paths"]["/drive/enable"]["post"]["responses"] = drive_command_responses
    OPENAPI_SPEC["paths"]["/drive/disable"]["post"]["responses"] = drive_command_responses
    OPENAPI_SPEC["paths"]["/execute/sequence"]["post"]["responses"] = {
        "200": {"description": "Blocking sequence completed with final result 0"},
        "202": {"description": "Sequence queued or accepted for asynchronous planning/execution"},
        "400": {"description": "Invalid sequence request"},
        "409": {"description": "Drive not enabled or controller execution failure"},
        "500": {"description": "Internal or planning failure"},
        "503": {"description": "MoveIt service, queue, or hardware not ready"},
    }


_apply_openapi_details()


SWAGGER_HTML = """<!doctype html>
<html>
<head>
  <title>ZeroErr Robot Runtime API</title>
  <link rel="stylesheet" href="https://unpkg.com/swagger-ui-dist@5/swagger-ui.css">
  <style>body { margin: 0; background: #fff; }</style>
</head>
<body>
  <div id="swagger-ui"></div>
  <script src="https://unpkg.com/swagger-ui-dist@5/swagger-ui-bundle.js"></script>
  <script>
    window.onload = function() {
      SwaggerUIBundle({
        url: "/openapi.json",
        dom_id: "#swagger-ui",
        deepLinking: true,
        displayRequestDuration: true,
        tryItOutEnabled: true,
        supportedSubmitMethods: ["get", "post"]
      });
    };
  </script>
</body>
</html>
"""
