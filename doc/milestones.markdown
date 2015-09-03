# Software/Autopilot Mission Planning

---

## 1. System is integrated and demonstrated
In this milestone, the ground-control-pixhawk-raspberry pi-tether system is integrated and demonstrated on a real system. 

### Tasks
#### 1. Select the system configuration

~~In this task the initial hardware and interfaces are selected~~

#### 2. Integrate systems

In this task (Due 2015/9/17)
* the raspberry pi can send/receive commands to/from the pixhawk at the maximum rate (10 Hz)
* the raspberry pi can send/receive commands to/from the ground station
* the ground station can receive telemetry/GPS information from the pixhawk
* the ground station can send GPS information and mission commands to the raspberry pi, and receive data packages from from the raspberry pi
* the pixhawk can receive commands from the raspberry pi and send gps/telemetry information to the raspberry pi
* the pixhawk can send telemetry/gps information to the ground station

#### 3. Develop autonomous mission control

In this task (Due 2015/9/24)
* A user can specify a sampling grid with various parameters
   * r, $\theta$, $\phi$, maxes and mins and number of points between max and min
   * loiter time at points
   * travel speed between points
* The sampling grid waypoints will be generated
* The raspberry pi can take this grid and control the pixhawk to cover the gridded area flying relative to the ground-station gps coordinates
* If communication with the ground station is lost, the raspberry pi can interrupt the mission and control the pixhawk to land
   * Same thing with other critical problems (Such as low battery) -- need to define failure criteria

#### 4. Manual System Demonstration

In this task (Due 2015/09/24), it will be demonstrated that the copter can fly when manually controlled when
* the copter is untethered and the ground station is stationary
* the copter is tethered and the ground station is stationary
* the copter is tethered and the ground station is moving

#### 5. Autonomous System Demonstration

In this task (Due 2015/10/08), it will be demonstrated that the copter can fly a simple grid pattern relative to the ground station when
* the copter is untethered and the ground station is stationary
* the copter is untethered and the ground station is moving
* the copter is tethered and the ground station is stationary
* the copter is tethered and the ground station is moving

#### 6. Tether reel integration
In this task (Due 2015/10/15)
* The reel control can be controlled through the ground station
* The reel status can be sent to the raspberry pi, and the raspberry pi can ask the ground control station to reel the copter in or out
* In case of catastrophe, the reel will automatically retract to pull the copter back to landing

-----
## 2. System is optimized and stable
In this milestone, the copter-tether dynamical system is understood from energy-optimal and stability perspectives, and can be robustly controlled. 

### Tasks

#### 1. Analytically analyze the tether-copter system to determine limits of the performance envelope
In this task (Due 2015/11/10)
* Equation for the tether-copter system are developed
* The maximum tether-copter angles (both $\theta$ and $\phi$ are calculated to show where the tether reduces energy requirements

#### 2. Optimize energy performance
In this task (Due 2015/11/10)
* A function or lookup table for the optimal tether tension and copter pitch is calculated (inputs being tether length, $\theta$, and $\phi$

#### 3. Optimize stability
In this task (Due 2015/11/10)
* The best tether attachment point is determined

---
## 3. Ground Control Software is user-friendly
In this milestone, then Ground control software has an easy-to-use and robust GUI

### Tasks
#### 1. Integrate outputs received from quadcopter with GUI elements in GCS
In this task (Due 2015/09/17) there are new dials to show the outputs from measurements in real time

#### 2. Add custom mission control buttons
In this task (Due 2015/09/17), new GUI elements are added to control the mission on a high level. This includes specifying the grid, controlling the tether, starting and stopping the mission, emergency landing, and emergency stop

#### 3. Validate and test GUI components with Hardware for normal usage
In this task (Due 2015/10/01), it is verified that the GUI buttons and outputs work as expected

#### 4. Test GUI elements

In this task (Due 2015/10/15), it is verified that the GUI buttons and outputs work as expected for anticipated emergency cases


----
# Other Milestones