# SJ-Future-Engineers

Welcome to the GitHub Repository of Team Robosenian, competing for the Philippine Robotics Olympiad Future Engineers 2026. The robot is an autonomous, scaled-down vehicle designed to navigate a structured track, avoid obstacles (red and green pillars), and park or change lanes based on visual feedback. The architecture decouples high-level computer vision and path planning from low-level motor actuation and sensor data aggregation.

The autonomous vehicle uses a single SPIKE Prime Large Hub as both the high-level logic controller and low-level actuator driver. The robot navigates an obstacle course using color detection and ultrasonic distance mapping, processing all feedback locally on the hub using the Pybricks.

Steering Mechanism (Ackermann Geometry)
Instead of differential drive (like a tank), the vehicle utilizes Ackermann steering geometry. This ensures that when the robot turns, all wheels trace concentric circles around a single common center point, preventing the tires from slipping sideways.
Actuation: A single spike prime motor is linked to the front steering knuckles via adjustable turnbuckles.
Mechanical Advantage: The steering linkages are configured so that the inner wheel turns at a sharper angle than the outbound wheel during a turn.
Propulsion: A single SPIKE Large Motor drives the rear axle.
Color Sensors: Mounted low to the ground, pointing down at the front to recognize boundary lines (blue/orange), or angled forward to evaluate the color signatures of obstacle pillars (red/green). 
Ultrasonic Sensor: Mounted to the side and front to identify track outer walls or confirm distance to a pillar.

# Members 
-**<p>Ethan Blaire S. De la Piña</p>** 
*<p>Background: Student of San Jose National High School</p>*
<p>Role: Coder, Strategist</p>
Year of Birth: 2009

-**<p>Jay Arnel A. Garcia</p>**
*<p>Background: Student of San Jose National High School</p>*
Role: 3D Printer, Documentator
<p>Year of Birth: 2010</p>

-**<p>Mel Iven M. Pangandoyon</p>**
*<p>Background: Student of San Jose National High School</p>*
Role: Build Designer, Coder
<p>Year of Birth: 2009</p>

-**<p>Nelson T. Ingking</p>**
*<p>Background: Teacher in San Jose National High School</p>*
Role: Team Coach, Guide
<p>Year of Birth: 1985</p>



# Strategies

The strategy we implemented for Open Challenge is that, after the color sensor first detects whether it is orange or blue, it accelerates based on the direction it is moving. For the Obstacle Challenge, our strategy is that after the color sensor detects a line, whether orange or blue, the robot maneuvers itself into the direction it will run, parallel to the wall.

### Mobility Management 

Mobility management governs how our vehicle physically moves, translates software decisions into mechanical force, and maintains structural stability on the track. Our architecture decouples traction control (propulsion) from directional control (steering) to optimize efficiency, torque, and tracking accuracy.
<p>A. Propulsion System (The Drive Motor)Component Selected: SPIKE Large Motor (drive = Motor(Port.C))</p>
<p>Implementation: Mounted longitudinally at the rear of the vehicle, driving the rear axle directly.</p>
<p>Engineering Principles (Torque vs. Speed): The propulsion system requires high starting torque to overcome static friction and rapidly accelerate out of corner turns. The Large Motor provides a superior stall torque (continuous, peaking higher under duty cycle) compared to smaller variants. Because our script commands the motor using raw duty cycle percentages (drive.dc(DCSPEED)), the motor naturally draws more current to provide the necessary torque when encountering resistance, maintaining steady momentum without stalling.</p>
<p>B. Directional Control (The Steering Actuator)Component Selected: SPIKE Medium Servo Motor (steer = Motor(Port.A))</p>
Implementation: Positioned at the front of the vehicle, directly over the front axle assembly.</p>
<p>Engineering Principles (Precision & Speed): Steering requires high rotational speed and precise angular positioning rather than high continuous torque. The Medium Servo features a lower internal gear ratio, allowing it to snap to exact targeted angles (steer.run_target(1000, angle)) at speeds up to 1000 deg/s.</p>
<p>Internal PID Feedback: Both motors utilize embedded optical encoders that track positions with 1 degree resolution. This allows the Pybricks firmware to run internal Proportional-Integral-Derivative (PID) loops, ensuring the steering angles match our software's structural variables precisely, even under the lateral stress of a turn.</p>
<p></p>1.2 Vehicle Chassis Design and Geometry. The foundation of our mobility management relies on Ackermann Steering Geometry rather than a differential (tank-style) drive.</p>

**<p>3D PRINTED FILES</p>**
<a href="FE_NEW_BASE.stl">BASE</a>
<a href="NEW_FE_LHOLDERR.stl">L HOLDER</a>
<a href="cover">cover</a>

### Power and Sense Management

<p>Power and sense management dictate how our vehicle distributes electrical energy to sustain high-performance operations and how it gathers environmental telemetry to safely solve the track challenges. This subsystem ensures that our computational nodes, sensors, and actuators are adequately powered without experiencing voltage drops or signal degradation.</p>

**<p>A. Primary Power: LEGO SPIKE Prime Battery</p>**
<p><strong>Specifications: 7.3V, 21000 mAh</strong>Lithium-Ion rechargeable battery pack housed inside the SPIKE Prime Hub.</p>
<p><strong>Allocation:</strong>Powers the central ARM Cortex-M7 processor, the internal 6-axis IMU, the front steering servo motor, the rear drive motor, the downward color sensor, and the ultrasonic sensor.</p>
<p><strong>Engineering Justification:</strong>The drive motor experiences heavy spikes in inductive current draw during acceleration and rapid reversals. By keeping this high-current draw on its own isolated chemical cell, we eliminate the risk of voltage sags (brownouts) that would reset our microcontrollers.</p>

**<p>B. Secondary Power: Vinko Mini Fast-Charging Power Bank</p>**
<p><strong>Specifications: 10000mAh (or 20000mAh)</strong>capacity featuring built-in smart LED capacity tracking and an integrated Type-C braided cable delivering up to <strong>22.5W max</strong> output power.</p>
<p><strong>Allocation:</strong> Dedicated solely to powering our co-processing vision subsystem: the ESP32 microcontroller and the DFRobot HuskyLens AI camera.</p>

### Open Challenge
Objective: Complete three autonomous laps.

The first problem we experienced was the random internal wall placements. The solution we implemented is that we put ultrasonics on both sides(left and right) of the robot. Another problem was the inconsistent reading of the color sensor. The solution we made was to make sure the sensor's physical environment is stable and maintain a consistent height. The last problem we encountered was that sometimes the build would fall off on its own. The solution for that was to balance the weight distribution of the robot.

### Obstacle Challenge 
Objective: Complete three laps with a traffic sign aligned with its direction.

The first problem we experienced was fixing the traffic sign compliance. The solution for that was to put a Husky Lens Camera that can be connected to an LMS. The second was the inconsistent detection of traffic lights from the camera. The solution was to put the camera at the back for a wider field of view. We also implemented a gyro for precise straight movement, maintaining angular velocity, and allowing a robot to track its rotation.

# Modules

pybricks.hubs (PrimeHub): The core library for the main vehicle controller. It manages internal electronics like the 6-axis Inertial Measurement Unit (IMU/Gyro), built-in speaker, status LED matrix, and battery management.

pybricks.pupdevices (Motor, ColorSensor, UltrasonicSensor): The hardware abstraction layer for LEGO Powered Up (PUP) peripherals. It contains built-in feedback control systems (PID loops), allowing the code to command precise angles, tracking speeds, and distances.

pybricks.parameters (Color, Port): A configuration module containing constants that define the physical layout of the hub (e.g., ports A through E) and primitive colors for the LED light.

pybricks.tools (wait, StopWatch): The time-management module. wait() yields CPU control for safe scheduling, while StopWatch() implements software timers used heavily in your code for cooldowns and emergency reverse durations.

pupremote_hub (PUPRemoteHub): A communication protocol module. It enables the LEGO Hub to communicate via UART serial communication over a standard sensor port to an external microcontroller (in your case, an ESP32 running a HuskyLens AI camera).
### Electromechanical Hardware Mapping

| Component Instance | Physical Hardware Component | Vehicle Subsystem | Software Interaction & Functionality |
| :--- | :--- | :--- | :--- |
| **`hub = PrimeHub()`** | LEGO SPIKE Prime Brick / Hub | Central Controller & IMU | Processing unit. Uses internal **6-axis Gyro** via `hub.imu.heading()` to keep the vehicle straight. Controls built-in status lights and speaker beeps. |
| **`steer = Motor(Port.A)`** | LEGO Powered Up Servo Motor | Steering Actuator | Connected to front wheels. Uses internal optical encoders to turn wheels to exact structural angles (`STEER_CENTER`, `SIDE_WALL_NUDGE_STEER`). |
| **`drive = Motor(Port.C)`** | LEGO Powered Up Large Motor | Propulsion System | Connected to drive axles. Commanded via direct motor power duty-cycles (`drive.dc(DCSPEED)`) or regulated velocity track speeds (`drive.run()`). |
| **`color = ColorSensor(Port.E)`** | LEGO Color Sensor | Downward Lap Sensor | Pointed at track floor. Measures Hue, Saturation, Value, and raw reflection variables to flag down-track Blue/Orange boundary lap lines. |
| **`ultra = UltrasonicSensor(Port.D)`** | LEGO Ultrasonic Distance Sensor | Front Collision Buffer | Emits acoustic sound waves straight ahead. Evaluates buffer zone distance (`front_dist()`). Triggers emergency braking routines if wall distance drops below $50\text{ mm}$. |
| **`lms = PUPRemoteHub(Port.B)`** | ESP32 MCU + HuskyLens AI Camera | Vision & Side-Telemetry | Communicates via a serial UART bridge. Translates object bounding boxes (`cx`, `cy`, `cw`) and side-wall ToF sensor distances into raw system data arrays. |

### The Process to Build, Compile, and Upload the Code

We ensure that our LEGO Prime Hub has the  Pybricks Firmware installed. We have a Pybricks Code installed on our laptops. Through this, we can now upload our code to the SPIKE Prime Hub and make the robot do the open and obstacle challenge. When we click the green Play icon inside the Pybricks IDE, the deployment pipeline executes an automated process instantly. First, the browser IDE handles syntactic verification by parsing our code to catch structural Python syntax errors or unmapped variables before deployment. Next, bytecode compilation takes over to instantly compress the raw MicroPython script into an optimized, lightweight binary representation called a .mpy file. Finally, a stream upload sends this compiled bytecode directly over our active Bluetooth Low Energy or USB-C connection into the volatile RAM of the LEGO Prime Hub controller, initializing the vehicle for immediate execution.
