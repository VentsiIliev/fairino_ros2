# REST Reorganization Plan

## Goal

Move REST-specific code into one folder without changing endpoint behavior, JSON shapes, launch behavior, or the runtime gateway architecture.

The REST layer should become easier to find and maintain, while `RuntimeApi` and `RuntimeGateway` remain transport-neutral.

## Target Structure

```text
erob_moveit_runtime/
  scripts/
    main.py
    runtime_initializer.py
    robot_controller.py
    config.py
    enums.py

    rest/
      __init__.py
      main.py
      server.py
      api_support.py
      openapi.py

    runtime_api/
      __init__.py
      handlers.py

    runtime_gateway/
      __init__.py
      base.py
      local.py
      ros.py        # future

    runtime_websockets/
      __init__.py
      execution_server.py
      state_server.py

    backend/
    motion/
    safety/
    status/
    utils/
```

## File Mapping

```text
scripts/rest_server.py        -> scripts/rest/server.py
scripts/rest_server_main.py   -> scripts/rest/main.py
scripts/rest_api_support.py   -> scripts/rest/api_support.py
```

Also extract the OpenAPI and Swagger constants:

```text
scripts/rest/openapi.py
  OPENAPI_SPEC
  SWAGGER_HTML
```

Keep compatibility wrappers during the migration:

```text
scripts/rest_server.py
scripts/rest_server_main.py
scripts/rest_api_support.py
```

These wrappers should import and re-export the new modules so existing launchers, scripts, tests, and installed entry points keep working.

## Boundaries

### REST Folder

The `rest/` folder owns HTTP-specific code:

- Flask app construction
- route registration
- request parsing helpers
- REST/OpenAPI documentation
- Swagger HTML
- REST process entry point

REST code may import:

- `runtime_api`
- `runtime_gateway`
- `runtime_websockets`
- `config`
- `robot_controller`
- backend/runtime initialization helpers

### Runtime API

Do not move `runtime_api/` into `rest/`.

`RuntimeApi` is intentionally transport-neutral. It should remain usable by REST, WebSocket commands, tests, and future adapters.

### Runtime Gateway

Do not move `runtime_gateway/` into `rest/`.

The gateway is the stable runtime boundary. REST is only one client of it.

### WebSockets

Keep `runtime_websockets/` separate for now.

Although the current websocket servers are started by REST server startup, they are not pure REST files. They may later talk directly to `RuntimeGateway` or ROS topics.

## Phase 1: Add New REST Package

Create:

```text
scripts/rest/__init__.py
scripts/rest/server.py
scripts/rest/main.py
scripts/rest/api_support.py
scripts/rest/openapi.py
```

Move code mechanically:

- `rest_server.py` contents into `rest/server.py`
- `rest_server_main.py` contents into `rest/main.py`
- `rest_api_support.py` contents into `rest/api_support.py`
- `OPENAPI_SPEC`, `_apply_openapi_details()`, and `SWAGGER_HTML` into `rest/openapi.py`

Update imports inside moved files:

```python
from rest.api_support import ...
from rest.openapi import OPENAPI_SPEC, SWAGGER_HTML
```

Keep non-REST imports unchanged unless they need package qualification.

## Phase 2: Add Compatibility Wrappers

Replace old root-level files with thin wrappers.

### `scripts/rest_server.py`

```python
#!/usr/bin/env python3
from rest.server import start_rest_server

__all__ = ["start_rest_server"]

if __name__ == "__main__":
    start_rest_server()
```

### `scripts/rest_server_main.py`

```python
#!/usr/bin/env python3
from rest.main import main

if __name__ == "__main__":
    main()
```

### `scripts/rest_api_support.py`

```python
#!/usr/bin/env python3
from rest.api_support import *  # compatibility re-export
```

Keep these wrappers until all external references have been migrated.

## Phase 3: Update Internal Imports

Update internal imports to use the new REST package:

```python
from rest.server import start_rest_server
from rest.api_support import parse_move_linear_request
```

Likely files:

```text
scripts/main.py
scripts/runtime_api/handlers.py
scripts/rest/main.py
scripts/rest/server.py
```

Search commands:

```bash
rg "rest_server|rest_server_main|rest_api_support" scripts
```

Expected result after migration:

- wrappers may still mention old names
- internal runtime code should use `rest.*`
- external compatibility remains intact

## Phase 4: Update CMake Install Rules

Add `rest` to the Python module directory install loop:

```cmake
foreach(module_dir api backend motion rest runtime_api runtime_gateway runtime_websockets safety status utils)
```

Keep installing wrapper scripts/files:

```cmake
install(PROGRAMS
  scripts/main.py
  scripts/simple_monitor_gui.py
  scripts/rest_server_main.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/rest_server.py
  scripts/rest_api_support.py
  ...
  DESTINATION lib/${PROJECT_NAME}
)
```

This preserves installed paths while also installing the new package.

## Phase 5: Verification

Run syntax checks:

```bash
python3 -m py_compile \
  scripts/rest/server.py \
  scripts/rest/main.py \
  scripts/rest/api_support.py \
  scripts/rest/openapi.py \
  scripts/rest_server.py \
  scripts/rest_server_main.py \
  scripts/rest_api_support.py \
  scripts/runtime_api/handlers.py \
  scripts/main.py
```

Build:

```bash
source /opt/ros/jazzy/setup.bash
source /home/ilv/ros2_ws/install/local_setup.bash
cd /home/ilv/ros2_ws/eRob_moveit
colcon build --packages-select erob_moveit_runtime --allow-overriding erob_moveit_runtime
```

Check installed files:

```bash
find install/erob_moveit_runtime/lib/erob_moveit_runtime/rest -maxdepth 1 -type f | sort
```

Import smoke checks:

```bash
source /opt/ros/jazzy/setup.bash
source /home/ilv/ros2_ws/eRob_moveit/install/local_setup.bash
EROB_CONFIG_PACKAGE=zeroerr python3 -c "from rest.server import start_rest_server; print(start_rest_server)"
EROB_CONFIG_PACKAGE=zeroerr python3 -c "from rest_api_support import parse_move_linear_request; print(parse_move_linear_request)"
```

Search for stale imports:

```bash
rg "from rest_server|import rest_server|from rest_api_support|import rest_api_support" scripts
```

Only compatibility wrappers should remain, unless there is a deliberate external-compatibility reason.

## Acceptance Criteria

- REST-specific implementation files live under `scripts/rest/`.
- `runtime_api/` and `runtime_gateway/` remain outside `rest/`.
- Existing entry point `rest_server_main.py` still works.
- Existing import `from rest_server import start_rest_server` still works.
- Existing import `from rest_api_support import ...` still works.
- Endpoint URLs are unchanged.
- JSON request/response shapes are unchanged.
- WebSocket startup behavior is unchanged.
- Package builds with `colcon`.
- New `rest/` package is installed into the overlay.

## Later Cleanup

After all robot-specific launchers and scripts have migrated to `rest.*` imports, consider removing compatibility wrappers in a separate cleanup.

Do not remove wrappers in the same PR/change as the move unless every external reference has been audited.
