# abb_pkg

ROS 2 package for controlling ABB robots via **EGM (Externally Guided Motion)**. It runs an EGM server on the PC that receives joint feedback from the robot controller and sends target joint positions in real time over UDP.

## Overview

- **egm_node**: A ROS 2 node that acts as an EGM server. It listens on a configurable UDP port (default 6510), waits for EGM messages from the robot controller, and replies with target joint positions for a built-in "one joint move" trajectory (joint 5 moves from 90° to ~0°; joints 1–4 and 6 are fixed).

## Dependencies

- **ROS 2** (tested with Humble)
- **rclcpp**
- **abb_libegm**: ABB EGM library (UDP protocol, protobuf messages)
- **Boost** (thread, asio; typically pulled in by abb_libegm)

## Build

From your ROS 2 workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select abb_pkg
source install/setup.bash
```

## Run

Start the EGM server (robot controller must be in EGM mode and sending to this PC):

```bash
ros2 run abb_pkg egm_node
```

Override the UDP port (default 6510):

```bash
ros2 run abb_pkg egm_node --ros-args -p port:=6511
```

## EGM Interface (non-obvious details)

1. **Roles**: The **robot controller** is the UDP *client*; it sends EGM packets first. The **PC (this node)** is the UDP *server*; it binds to the port and only sends replies after receiving a message. If you start the node before the robot sends, the node will block in `waitForMessage` (up to the timeout, default 1000 ms) until the first packet arrives.

2. **Port**: Default is **6510**. It must match the port configured in the robot EGM RAPID/setup. Firewall must allow UDP on this port.

3. **Message flow**: Each cycle: (1) node waits for one EGM message from the robot (`waitForMessage`), (2) reads it into `Input`, (3) fills `Output` with target joint positions, (4) sends `Output` back. The node does not send until it receives; timing is driven by the robot’s send rate (e.g. 4 ms or 8 ms).

4. **Threading**: `abb_libegm::EGMControllerInterface` uses Boost.Asio. `io_service::run()` is started in a **separate thread** so that the main thread can call `waitForMessage`, `read`, and `write` without blocking the UDP I/O. Do not call these from multiple threads without checking the library’s thread-safety.

5. **Trajectory**: The built-in trajectory advances joint 5 by **0.1°** per received EGM message. When the accumulated angle reaches 90°, the node logs "EGM task finished!" and exits. No ROS topics or services are used; the only ROS dependency is the node lifecycle and the `port` parameter.

6. **Output content**: Only **position** references are sent (joint space). `use_demo_outputs` and `use_velocity_outputs` are false; the configuration is position-only, six axes.

## Prerequisites

- Robot controller with EGM option and a program that starts EGM and sends to the PC’s IP and port.
- Network connectivity between PC and controller (correct IP, subnet, and firewall rules).
- Build and run on the same machine that the robot is configured to use as the EGM peer.

## License

Apache-2.0
