# Copilot instructions for uwb_positioning

This file captures concise, actionable guidance for AI coding agents working on the `uwb_positioning` ROS 2 package.

Key concepts
- **Purpose:** a small ROS 2 ament_python package that reads DWM1001-style CSV/serial POS packets and publishes ROS topics; also includes simple IMU I2C support and a TF broadcaster.
- **Build system:** ament_python (see `package.xml` and `setup.py`). Package exposes console entry points in `setup.py` (e.g. `uwb_read_only_node`, `uwb_publisher`, `imu_publish`).

Where to look (examples)
- Node that reads serial/UWB input: `uwb_positioning/reader.py` (regex POS parser, publishers: `uwb/raw`, `uwb/pose`, `uwb/qf`).
- Launch templates: `launch/uwb_bridge.launch.py` and `launch/mapping.launch.py` (shows parameter names and intended run pattern).
- TF broadcaster: `uwb_positioning/uwb_publisher.py` (subscribes `/uwb/pose` and broadcasts TF from `odom` -> `base_link`).
- IMU sensor code and ROS publisher: `uwb_positioning/imu_publish_data.py` and helper `imu_gather_data.py` (I2C via `smbus`, publishes `/imu/data`).
- Entry point wrappers: `uwb_positioning/runner.py` (strips `--ros-args` and forwards to `reader.main`).

Developer workflows & commands (verified from repository files)
- Build (workspace root):
  - `colcon build --packages-select uwb_positioning`
  - Then source: `source install/setup.bash` (or `install/setup.zsh` as appropriate).
- Run via launch (examples using launch args in repo):
  - UWB bridge: `ros2 launch uwb_positioning uwb_bridge.launch.py port:=/dev/ttyACM0 baud:=115200 frame_id:=odom`
  - Full mapping example: `ros2 launch uwb_positioning mapping.launch.py` (this includes UWB and an external lidar launch).
- Run individual nodes:
  - `ros2 run uwb_positioning uwb_publisher`
  - `ros2 run uwb_positioning uwb_read_only_node`
  - IMU node: `ros2 run uwb_positioning imu_publish`

Project-specific conventions & important details
- Parameters used by `reader.py`: `port`, `baud`, `scale_m`, `frame_id`, `read_crlf` — launches set these via `LaunchConfiguration`.
- `reader.py` publishes:
  - `uwb/raw` (std_msgs/String) — raw incoming lines for debugging
  - `uwb/pose` (geometry_msgs/PoseStamped) — position (x,y), z set to 0.0, orientation.w=1.0
  - `uwb/qf` (std_msgs/Int32) — quality factor parsed from POS packets
- POS packet format: `POS ...` regex in `reader.py` — use that file as canonical parser when changing parsing behavior.
- TF frames: `uwb_publisher.py` currently hardcodes `header.frame_id = "odom"` and `child_frame_id = "base_link"`. Mapping.launch sets UWB node `frame_id` to `odom` — be conservative when renaming frames.
- IMU code uses `smbus.SMBus(1)` (I2C bus 1) and assumes MPU-9150 at I2C address `0x68`. Hardware-only code; mock or guard when running on non-embedded systems.

Testing & debugging tips
- No automated tests are provided. Use runtime checks:
  - Subscribe to topics: `ros2 topic echo /uwb/pose` and `ros2 topic echo /uwb/raw` to validate parsing.
  - Use `ros2 node info /uwb_publisher` and `ros2 topic hz /uwb/pose` to inspect rates.
  - Watch logs: run `ros2 run uwb_positioning uwb_read_only_node` and inspect `INFO`/`WARN` output.
- Serial hardware caveats: default ports in launches point to `/dev/serial/by-id/...` or `/dev/ttyACM0`. Confirm correct device on the host.

Safe edit patterns for AI agents
- When editing parser logic, update `uwb_positioning/reader.py` only and add unit-like examples as comments or small parsing helper functions. Keep the published topic schema unchanged unless coordinating with `uwb_publisher.py`.
- Avoid changing hardcoded TF frame names without updating `launch/*` and documenting the change in this file.
- For IMU changes: separate hardware access (`smbus`) into a small adapter class and detect environment (raise or stub) to keep the package runnable on CI/desktop.

If something is missing
- Ask the maintainer which runtime environment to prefer (Raspberry Pi vs development laptop), desired frames, and whether new topics or message types are allowed.

Please review this guidance and tell me if you want different emphasis (more testing steps, CI instructions, or sample expected messages).
