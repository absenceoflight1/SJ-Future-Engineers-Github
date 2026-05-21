# SJ-Future-Engineers

The robot is an autonomous, scaled-down vehicle designed to navigate a structured track, avoid obstacles (red and green pillars), and park or change lanes based on visual feedback. The architecture decouples high-level computer vision and path planning from low-level motor actuation and sensor data aggregation.

The autonomous vehicle uses a single SPIKE Prime Large Hub as both the high-level logic controller and low-level actuator driver. The robot navigates an obstacle course using color detection and ultrasonic distance mapping, processing all feedback locally on the hub using the Pybricks.

Steering Mechanism (Ackermann Geometry)
Instead of differential drive (like a tank), the vehicle utilizes Ackermann steering geometry. This ensures that when the robot turns, all wheels trace concentric circles around a single common center point, preventing the tires from slipping sideways.
Actuation: A single spike prime motor is linked to the front steering knuckles via adjustable turnbuckles.
Mechanical Advantage: The steering linkages are configured so that the inner wheel turns at a sharper angle than the outbound wheel during a turn.
Propulsion: A single SPIKE Large Motor drives the rear axle.
Color Sensors: Mounted low to the ground, pointing down at the front to recognize boundary lines (blue/orange), or angled forward to evaluate the color signatures of obstacle pillars (red/green). 
Ultrasonic Sensor: Mounted to the side and front to identify track outer walls or confirm distance to a pillar.
