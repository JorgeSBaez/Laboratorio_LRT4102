
# 🤖 Guide Robot

A smart navigation robot for classroom simulation using **ROS** and **TurtleSim**.  
It listens to voice commands to navigate to chairs, avoids obstacles (other chairs), and provides feedback on navigation performance.

---

## 📦 Features

- 🎯 Navigate to any of 9 chairs with voice commands like `"silla 3"`
- 🚧 Avoid obstacles using repulsion logic based on distance to chairs
- 📢 Publishes feedback via a custom topic (`/feedback_robot`)
- 🧠 Uses ROS publishers/subscribers and a control loop
- 📊 Tracks time, distance, and obstacle avoidance attempts

---

## 🗺️ Environment Setup

The script:
- Removes the default turtle (`/kill turtle1`)
- Spawns the main robot at (1.0, 1.0)
- Spawns 9 turtle "chairs" at fixed positions

These are placed as follows:

```
(2.0, 2.0)   (5.0, 2.0)   (8.0, 2.0)
(2.0, 5.0)   (5.0, 5.0)   (8.0, 5.0)
(2.0, 8.0)   (5.0, 8.0)   (8.0, 8.0)
```

---

## 🧠 How It Works

### Command Listener
Listens to `/comando_voz` topic:
- `"silla <n>"`: starts navigating to chair `n`
- `"detener"`: stops the robot

### Pose Updates
Subscribes to `/turtle1/pose` and:
- Updates internal state
- Tracks distance traveled

### Navigation
- Computes angle and distance to target
- If an obstacle is near, it computes a **repulsion force**
- Adjusts velocity and rotation accordingly

### Feedback
Publishes real-time updates like:
- `"A 0.7m del destino"`
- `"Evitando silla 5"`
- `"Destino alcanzado. Tiempo: 25.0s, Distancia: 10.3m, Obstáculos evitados: 14"`

---

## 📊 Metrics Tracked

| Metric               | Description                                  |
|----------------------|----------------------------------------------|
| `tiempo_inicio`      | Time when navigation begins                  |
| `distancia_recorrida`| Cumulative Euclidean distance                |
| `obstaculos_evitados`| Incremented when repulsion is active         |

---

## 🧪 How to Test

1. Launch the simulator:
   ```bash
   rosrun turtlesim turtlesim_node
   ```

2. Run the robot code:
   ```bash
   rosrun your_package robot_guiado_silla.py
   ```

3. Publish voice commands:
   ```bash
   rostopic pub /comando_voz std_msgs/String "data: 'silla 6'"
   ```

---

## ✅ Dependencies

- `rospy`
- `turtlesim`
- `geometry_msgs`
- `std_msgs`

---

## 📁 File Structure

```bash
robot_guiado_silla.py   # Main control script
README.md               # Project documentation
```

---

## ⚠️ Limitations

- Obstacle avoidance is basic and may fail in tight spaces
- No global map or path planning
- Feedback assumes ideal conditions in TurtleSim

---

## 📌 Notes

This robot simulates how a visually impaired student could be guided safely to a seat. It can be extended with real sensors (e.g., ultrasonic or lidar) and used in real robots to assist navigation in classrooms, hospitals, or other indoor environments.

---
