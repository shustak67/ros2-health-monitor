# ROS 2 Health Monitor

This repository contains the submission for the **ROS 2 Junior Developer Home Assignment**.  
It implements a simple **robot health monitoring system** using ROS 2 (Python).

---

## Packages

### 1. health_monitor
- Contains the three core nodes:
  - **`health_analyzer_node`** → subscribes to `/robot_status` and classifies health as `HEALTHY`, `WARNING`, or `CRITICAL`.
  - **`alert_manager_node`** → subscribes to `/health_status` and logs messages with correct ROS 2 logging levels, with frequency limiting.
  - **`can_ros2_status_node`** → simulator that publishes random `RobotStatus` values.

### 2. my_interfaces
- Custom message and service definitions:
  - `msg/RobotStatus.msg` → contains `left_rpm`, `right_rpm`, `battery_percentage`, `bit_errors`.
  - `srv/GetStatusHistory.srv` → returns the last N status readings.

### 3. health_monitor_bringup
- Contains the launch file:
  - `launch/health_monitor.launch.py` → starts the full system.

---

## Build and Run

Clone this repository into the `src` folder of your ROS 2 workspace and build:

```bash
cd ~/your_ros2_workspace   # replace with your actual workspace path
colcon build
source install/setup.bash
```

Launch the full system:

```bash
ros2 launch health_monitor_bringup health_monitor.launch.py
```

---

## Nodes

### health_analyzer_node

* **Subscribes**: `/robot_status` (`my_interfaces/msg/RobotStatus`)
* **Publishes**: `/health_status` (`std_msgs/String`)
* **Logic**:

  * `HEALTHY`: battery ≥ 25%, RPM below threshold, no BIT error
  * `WARNING`: battery 15–25%, RPM > threshold, or BIT error
  * `CRITICAL`: battery < 15% or both motors stopped > 5s
* **Features**:

  * Parameters for thresholds (`rpm_threshold`, `battery_warning_threshold`, `battery_critical_threshold`)
  * Rolling window of last 10 statuses
  * Service `/get_status_history` returns last N status messages

---

### alert_manager_node

* **Subscribes**: `/health_status` (`std_msgs/String`)
* **Logs**:

  * INFO → HEALTHY
  * WARN → WARNING / UNKNOWN
  * ERROR → CRITICAL (with timestamp)
* **Features**:

  * Logs only on transitions
  * Limits logs to once every 10s per status type

---

### can_ros2_status_node

* **Publishes**: `/robot_status` (`my_interfaces/msg/RobotStatus`)
* **Simulates**: random RPM, battery, and BIT error values

---

## Launch File

**`health_monitor.launch.py`**

* Launches all three nodes (`health_analyzer`, `alert_manager`, `can_ros2_status`)
* Uses descriptive names
* Output set to screen for debugging

---

## Bonus Challenges

* ✅ Configurable thresholds via ROS 2 parameters
* ✅ Health history service
* ❌ Emergency stop integration (not implemented; would require raw CAN frame input)

---

## Design Summary

The system is split into three modular nodes:

* One node handles **health logic**
* One node handles **alert logging**
* One node provides **simulated status input**

This follows ROS 2 best practices and makes the system extensible (e.g., replacing the simulator with a real CAN interface).

---

## Example Output

When running the launch file, the system produces logs such as:

```
[INFO] [health_analyzer]: HEALTHY: Battery: 61.5%, Left RPM: 1873, Right RPM: 1798, BIT: False
[WARN] [alert_manager]: WARNING: Low battery: 23.9%, Right RPM too high: 2834, BIT error present
[ERROR] [alert_manager]: CRITICAL: Battery critical: 7.7%
```

---

## Author

**Avishai Shustak**
B.Sc. Electrical & Electronics Engineering, Tel Aviv University

```
