
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
