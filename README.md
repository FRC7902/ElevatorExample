# Elevator Subsystem Example
Use this as a reference when programming an elevator in the future!

## What it does:
This is just a full, can be simulated, elevator subsystem in WPILib. It contains all the information you should know
and what you should do to program an elevator. It uses 2 Kraken x60s and CTRE MotionMagic©

## High Level Overview:
In ElevatorSubsystem.java

### Initialize all your objects:
- Elevator leader motor
- Elevator follower motor(will do exactly what the leader motor does in reverse)
- Motor Control
- Mechanism2d
- Mechanism root
- Mechanism Ligament(represents whole elevator)
- ElevatorSim physics simulation
- Elevator setpoint in meters

### Constructor:
Configure motor:
- state the brake mode
- state the inversion mode
- configure PID and gravity type
- configure motion magic control parameters
- set current limits (supply and stator)
- make the follower motor follow the leader motor
- apply same config to both motors

### Periodic (Happens once every 20msec):
- Put data on SmartDashBoard, especially the elevator's Mechanism2d

### SimulationPeriodic(Same as periodic but only happens in simulation mode)
- set supply voltage to 12
- set the input to the elevator physics simulation
- update it with a delay of 0.2 secs (20msec)
- set the rotor position and velocity for simulated motors using data from the physics simulation
- set the ligament length to the physics simulation length

### Some helper methods
- getElevatorPositionMeters()
- hasReachedAngle()

### Setting elevator position
- clamp the input to the min/max range
- convert meters given to rotations
- set the control parameters, such as the position to go to and the slot to use
- set the motor's control to the control object

## Challenge
go eat a candy bar