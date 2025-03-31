
# ROS Practice Lab: Turtlesim Control & Publisher-Subscriber Demo
This repository contains practical ROS (Robot Operating System) exercises divided into three levels: Basic (Publisher-Subscriber), Medium (Turtlesim manual control and drawing), and Advanced (P, PI, PID controllers for Turtlesim). Each section includes implementation details, conclusions, and performance analysis.

### ROS Practice Lab: Turtlesim Control & Publisher-Subscriber Demo



## 📌 Package Structure
```bash
Practicas_lab/
├── Lab02/
│   ├── Advanced/
│   │   ├── turtle_pc.py
│   │   ├── turtle_pdc.py
│   │   └── turtle_pidc.py
│   ├── Medium/
│   │   ├── ST_turtle.py
│   │   ├── ST_turtle.launch
│   │   ├── teleop.py
│   │   └── tortuga_teleop.launch
└── README.md
````
# 🗣️ Publisher-Subscriber Communication Model
## 📢 Message Publisher (talker.py)
### Core Functionality:

- Establishes a ROS node dedicated to broadcasting data

- Continuously transmits messages on a designated channel (/chatter)

- Uses standardized message format (std_msgs/String)

- Operates at configurable transmission frequency (default: 10Hz) 

## 👂 Message Receiver (listener.py)
### Core Functionality:

- Creates a complementary ROS node for data reception

- Subscribes to the publisher's channel (/chatter)

- Implements automatic message handling through callbacks

- Processes incoming data packets asynchronously

## 🔄 Communication Dynamics
### Real-Time Interaction
### Connection Protocol:

- Nodes automatically discover each other through ROS Master

- Direct peer-to-peer link established after discovery

- Connection persists until node termination

### Message Handling:

- Publisher maintains transmission regardless of subscriber status

- Subscriber queues incoming messages (configurable depth)

- Zero-copy transport used when possible for efficiency
# ⚠️ Controller Performance Analysis
## 1. Proportional Control (P)

- Exhibits consistent following error proportional to the target velocity

- System response shows gradual convergence without overshoot

- Permanent offset remains between desired and actual position

- Ideal for systems where precise positioning isn't critical

## 2. Proportional-Derivative Control (PD)

- Noticeably quicker response than pure P control

- Reduced oscillation tendencies during transients

- Maintains better tracking during velocity changes

- Still shows minor positional drift over time

## 3. Complete PID Implementation

- Achieves zero steady-state positioning error

- Demonstrates rapid convergence to target

- Maintains stability across various speed ranges

- Provides optimal disturbance rejection

## 💡Conclusions and Recommendations
### P Controller:

- Best for slow, stable systems

- Minimal configuration required

- Energy efficient but imprecise

### PD Controller:

- Ideal for moderate-speed applications

- Good balance between complexity and performance

- Recommended when small steady-state error is acceptable

### PID Controller:

- Essential for high-precision applications

- Requires careful tuning

- Delivers best overall performance

- Preferred for critical positioning tasks
