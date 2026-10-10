# Decision Process

## 1\. ROS2 / Stack Setup

* **Distro:** ROS2 Jazzy on Ubuntu 24.04 via WSL2. Only distro with stable Gazebo Harmonic integration.
* **Simulator:** Gazebo Harmonic (gz-sim8). Default for Jazzy; `gz\_ros2\_control` is the sim hardware interface.
* **Workspace:** `\~/ros2\_ws/src`, four packages consolidated into one git repo (`oisin-hott/robot\_arm`). Avoids cross-repo drift when packages change together.
* **Clock:** `use\_sim\_time: False` in hardware launch. Real-time clock, no Gazebo clock dependency.
* **Git auth:** SSH-based, since GitHub dropped HTTPS password auth.

### Dev environment

* **Linux laptop for the physical robot:** A dedicated Linux laptop runs ROS2 and talks to the Teensy. Chosen to work around the issues with WSL-to-Teensy (USB) connectivity.
* **Sync:** Code is synced between machines via GitHub.

## 2\. Motion Planning

* **Planner:** STOMP (not OMPL). Handles constrained, narrow-passage trajectories better for a real arm. OMPL stayed in `motionplan.launch.py` as a fallback. OMPL resulted in long unwanted trajectories
* **IK solver:** TRAC-IK (replaced KDL).

  * KDL gave erratic solutions and had base-rotation ambiguity.
  * TRAC-IK set to Distance solve type for the most stable solutions.
* **Planning groups:** `arm` (joints 1-6) and a separate gripper group (`left\_finger\_joint` only).
* **Start state:** `setStartStateToCurrentState()` called before every move. Prevents start-state deviation errors.
* **Pose targets:** Expressed in the `base\_link\_mesh` frame. Fixed, stationary reference frame, stable across all configurations.

## 3\. Controllers

* **Three controllers:** `joint\_state\_broadcaster`, `joint\_trajectory\_controller` (JTC), `gripper\_controller`.
* **Command type:** All position-command, no velocity commands. The arm is position-controlled; velocity is encoded in trajectory waypoint timing, not sent as explicit commands.
* **Spawner chain:**

  * `joint\_state\_broadcaster\_spawner` is added directly to `LaunchDescription` (self-retries until the controller manager is available).
  * Then JTC, then gripper, then `arm\_motion\_node`, chained via `OnProcessExit`.
  * Controllers chained on `ros2\_control\_node`'s exit never fire, because it never exits.
* **Startup ordering:** The `gripper\_controller` spawner must complete before `arm\_motion\_node` starts. Intermittent `left\_finger\_joint` out-of-bounds errors were caused by the arm executing before the gripper controller was active.

## 4\. URDF / Joint Configuration

* **Joints:** 7 total. `joint\_1`-`joint\_6` (arm) plus `left\_finger\_joint` (gripper). `right\_finger\_joint` is a mimic with no command interface as operating it as a real joint resulted in errors in the physics simulation.
* **Gripper is a placeholder:** The gripper in the model (finger joints, gripper controller, finger gear ratio entry) is a placeholder. The real gripper uses a servo.
* **Mimic joint:**

  * `right\_finger\_joint` mimics `left\_finger\_joint` with multiplier -1 (rack-and-pinion parallel gripper).
  * Declared in the URDF `<mimic>` tag and emulated by `gz\_ros2\_control` at the control layer, since the Gazebo physics engine drops mimic constraints.
  * `right\_finger\_joint` has no command interfaces in the `<ros2\_control>` block. Adding one causes an "Activated mimic joints cannot have command interfaces" crash.
* **Gazebo materials:** Go in `<gazebo reference="${name}">` blocks inside the `mesh\_link` xacro macro. URDF `<material><color>` tags are read only by RViz and ignored by Gazebo.
* **Removed items:**

  * `tcp\_fts\_sensor` block removed from `ros2\_controls\_xacro.xacro`. It was a MoveIt-generated template artefact, not exported by the hardware interface.
  * Velocity command interfaces removed from all joints. The arm is position-controlled, and declaring velocity commands the hardware doesn't export causes a "missing command interfaces" error.

## 5\. Simulation / Gazebo

### Box and grasp plugin

* **Box model:** Native SDF (not URDF). Simpler to add Gazebo system plugins, with no URDF-to-SDF conversion surprises.
* **Grasp plugin:** `DetachableJoint` (built into Gazebo, no extra install).

  * Topic-driven fake grasp: creates a fixed joint on attach, destroys it on detach.
  * Triggered by `arm\_motion\_node` publishing `std\_msgs/msg/Empty` to `/box/attach` and `/box/detach`.
  * Likely Will need to update this in ML Stage
* **Plugin declaration:** In the box's SDF (parent = box, child = robot gripper link). A robot spawned via URDF-to-SDF conversion makes injecting system plugins fiddly; native SDF is cleaner.
* **Attachment point:** Wrist link, not `left\_finger\_joint`. The finger is prismatic and physically misbehaves on contact; the wrist is the stable anchor.

### ROS-Gazebo bridge

* Required for attach/detach, because `DetachableJoint` listens on Gazebo Transport, not ROS2.
* `ros\_gz\_bridge` bridges with the `]` (ROS to GZ) direction: `/box/attach@std\_msgs/msg/Empty]gz.msgs.Empty`

### Known limitations and workarounds

* **`<initial\_attach>false</initial\_attach>`:** Not available on Harmonic (gz-sim8). The feature was added in gz-sim10.

  * Workaround: a one-shot `create\_wall\_timer(2s)` publishes a startup detach, then cancels itself.
* **Attach/detach timing:** Published after `gripper\_group\_->move()` returns. `move()` is blocking, so attaching before it returns would make the box snap to the open gripper prematurely.
* **`robot\_description` publishing:** `publish\_robot\_description=False` on `MoveItConfigsBuilder`.

  * Prevents double-publishing to `/robot\_description`; `robot\_state\_publisher` is the sole publisher.
  * A duplicate causes a "hardware name duplicated" crash and `std::bad\_variant\_access` in `pal\_statistics`.

## 6\. Hardware Interface Plugin (`six\_dof\_arm\_hardware`)

### Package and class structure

* **Package:** Separate package `six\_dof\_arm\_hardware` (ament\_cmake, depends on `hardware\_interface`, `pluginlib`, `rclcpp`). Keeps hardware-specific code isolated.
* **Class:** `SixDofArmHardware : public hardware\_interface::SystemInterface`, in namespace `six\_dof\_arm\_hardware`.
* **Jazzy API:** `on\_init` signature is `const hardware\_interface::HardwareComponentInterfaceParams\&` (not the deprecated `HardwareInfo\&`).
* **Registration:** Plugin registered via `six\_dof\_arm\_hardware.xml` and exported in `package.xml`. Uses the pluginlib mechanism, not a standalone node.

### Data members and interfaces

* **Member vectors:** `hw\_commands\_positions\_`, `hw\_states\_positions\_`, `hw\_states\_velocities\_`, `hw\_steps\_`, `steps\_per\_rad\_`.

  * All resized to `info\_.joints.size()` in `on\_init` before any loop indexing (a missing resize causes a segfault).
* **Exported interfaces:**

  * Position command interface only, no velocity command interfaces.
  * `export\_state\_interfaces` exports position + velocity state per joint.

### Unit conversion

* **`steps\_per\_rad\_`:** Computed once in `on\_init` from fixed physical constants (not per cycle).

  * Formula: `(steps\_per\_rev × microstepping × gear\_ratio) / (2π)`
* **Per-joint gear ratios:** `{1.0, 2.0, 3.0, 2.0, 5.0, 8.0, 2.0}` (7 elements including the finger, which is a placeholder). Hardcoded for now, URDF params deferred.
* **`write()`:** Converts rad to steps into `hw\_steps\_` only. Does not touch the state vectors.
* **`read()`:** Converts `hw\_steps\_` to rad into state.

  * The mock loops `write()`'s output back as `read()`'s input (simulates a perfect motor).
  * This proves the conversion is a correct inverse pair.

### Segfault root causes

1. `steps\_per\_rad\_` not resized before the loop.
2. `joint\_multiplier` had 6 elements but the loop ran over 7 joints.

## 7\. Packet Protocol

|Field|Detail|
|-|-|
|Size|30 bytes, fixed|
|Layout|Start byte (1 B) + 7 × float joint angles (28 B) + checksum (1 B)|
|Start byte|`0xAA`. Alternating bit pattern, easy to spot in a debugger or logic analyser|
|Payload type|`float`, not `int32\_t`. Radians are sent rather than steps; unit conversion moved to the Teensy|
|Endianness|Little endian raw copy is valid. Both the x86 PC and the Teensy (ARM Cortex-M7) are little endian, so no byte swapping is needed|
|Checksum|Sum of bytes 0-28 stored as `uint8\_t`; overflow wraps naturally|
|Cadence|One atomic packet per control cycle in each direction|

* **Checksum note:** The checksum includes the start byte (summing from `i=0`) on both the PC and Teensy sides.

## 8\. Control Architecture

* **PID location:** Runs on the Teensy, not the PC. ROS2 is not real time and a kHz control loop is needed.
* **Division of work:** The PC sends radian targets; the Teensy handles radian-to-steps conversion and PID.

  * `steps\_per\_rad` conversion removed from the PC side, lives entirely on the Teensy.
* **Loop structure:** Kept simple: a single line setting the speed for fine motor control.
* **Closed loop:** On the MT6701 encoder. Missed steps are irrelevant, giving servo-like performance at stepper cost.
* **Microstepping:** TMC2209 microstepping affects smoothness and resolution, and is invisible to the PID loop.
* **Cost:** Stepper + encoder is roughly 3-5x cheaper per joint than an equivalent servo system.
* **Topology:** Centralised control chosen for v1. Distributed per-joint CAN nodes considered for v2 to cut harness complexity and improve noise immunity.
* **PID theory:** Comes from Newton-Leibniz (simplified calculus to discrete steps).

## 9\. Hardware Interface: Serial Communication

* **`teensy\_connected` flag:** Boolean that allows software stack testing without physical hardware.
* **Read behaviour:**

  * `VMIN=30` in steady state: blocks until a full packet is available.
  * `VMIN=1` for resync: slides one byte at a time looking for the start byte.
* **Sync state:** `synced\_` boolean tracks whether packet alignment is established.
* **Code style:** Early-return guard clause pattern for packet validation checks.

## 10\. Homing

* **Encoder limitation:** The MT6701 is single-turn absolute, so gearing introduces multi-turn ambiguity.
* **Limit switches:** Needed per joint. They trigger before the hard stop and give a known physical reference.
* **Homing offset:** Maps the limit switch position to the URDF zero position.
* **Sequence:** Runs from base to end effector to avoid collisions.
* **Motion gating:** ROS2 rejects motion commands until all joints are homed.

## 11\. ROS2 Stack on Real Hardware

* **Trajectory interpolation:** JTC interpolates between MoveIt2 waypoints each control cycle.
* **State feedback:** Encoder positions are sent to ROS2 for state feedback and trajectory completion detection.
* **Goal tolerance:** Will be larger on real hardware than in simulation. Tune empirically.

## 12\. Mechanical Design

### Shoulder joint: cycloidal drive

* **Placement:** At the shoulder joint, where the most torque is required (see calcs).
* **Low backlash.**
* **Two-layer design:** Chosen for constant contact at two points.
* **Printed:** Stronger, and easier to get dimensionally right, than a harmonic drive.

### Bearings

* **Bought, not printed:** For longevity.
* **Fit:** Press-fit with a 0.1 mm tolerance.

### Wrist

* **Differential wrist:** Gives a compact 2-DOF wrist.
* **Why not independent drives:** Concentric shafts etc. to drive these joints separately were too expensive and complicated for MK1.

## 13\. Electronics

### Power

* **Layout:** The board has a 24 V pour and a ground pour.
* **24 V:** Supplied by an external power supply.
* **3.3 V:** Any 3.3 V needed comes from the Teensy's USB.
* **Reverse polarity protection:** PFET used for reverse voltage polarity protection.
* **Power Surge**: 10A fuse used to protect the circuit

### Encoders

* **Interface choice (SSI vs I2C):** SSI chosen.

  * Works well with few pins.
  * Gives data checks, which are needed for safety: a large error in the encoder value results in large motion jumps.
* **Mode change:** MT6701 modified to go from I2C mode to SSI mode (removal of resistors etc.).
* **Noise reduction:** Decoupling capacitors are used on the encoders at their connection to the board.

### Other

* **Limit switch pin:** Switch to pin 37 for limit switches due to faulty trace from original, 38 pin..



## 14\. References

* Encoder programming example (MT6701 SPI read): https://github.com/I-AM-ENGINEER/MT6701-arduino/blob/main/examples/02\_spi\_read/02\_spi\_read.ino

