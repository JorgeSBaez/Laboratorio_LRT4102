# OS Practice Lab: TurtleSim Navigation System 

## 🐢 How the system works?

This project controls the turtle in ROS for :
- It calculates automatically the path to the specify target.
- It moves in a straight line.
-  Also displays the DTG and the ATG parameters. 

## 📊 Key Concepts

### DTG (Distance To Goal)
- Is a Straight-line distance between the turtle and target
- **Calculation**:  
  `DTG = √[(target_x - current_x)² + (target_y - current_y)²]` 
- Uses meters (in the turtlesim world)
- It helps to determine when the turtle should slow down when is getting closer to the target.  

### ATG (Angle To Goal)
- Is the optimal angle that the turtle takes to face in direction to the target. 
- **Calculation**:  
  `ATG = atan2(target_y - current_y, target_x - current_x)`  
- Uses radians but is displayed in degrees in logs. 
- It helps to provide the correct angle in all the quadrants.
  
### Proportional Controller 
The turtle receives precise velocity directives from the navigation system for execution purposes.

The Twist.linear.x velocity setting allows to control forward and backward speed from 0 to 1.5 meters per second. The speed has a limit to ensure precise control.

The control system for angular velocity (rotation) functions by altering values of Twist.angular.z between -3.0 and 3.0 radians per second. Positive values produce counter-clockwise rotation.

The control strategy proceeds step-by-step according to this logical sequence.
The transform system first turns the turtle toward its intended orientation up to a margin of 5.7 degrees before proceeding forward. Proper alignment becomes necessary before the device can start its forward progress and performs rotational corrections during its motion.
- **Basic Formula**:  
  `Velocity = Kp × Error`  
  Where Kp is a proportional constant

## 🎮 Expected Behavior 
Operation mode reveals continuously updated navigation metrics which present the current values for DTG and ATG. The destination values will decrease while the turtle moves toward its intended target.

The turtle will trace a direct path toward its endpoint which demonstrates excellent precision of its navigation methodology. The turtle will stop moving upon reaching 5cm proximity to the target since it fulfilled its navigation assignment.


## 📸 Visual Demonstration
![Start position](https://github.com/user-attachments/assets/8bdfe9af-ff22-4c02-92cb-dbe87bf40df5)
- Turtle goes to the starting position assigned in the code `2.0, 2.0, 0.0`
![ATG_DTG](https://github.com/user-attachments/assets/d1e296f6-9d13-4996-81e5-4bc69d731e0f)
- ATG and DTG parameters of the turtle movement. 
![Goal position](https://github.com/user-attachments/assets/02986eda-0487-483a-9c4e-f8483bbfe228)
- Turtle in the goal position `self.goal_x = 8.0
        self.goal_y = 8.0`

## 🚀 Execution
First you have to run `roscore & rosrun turtlesim turtlesim_node`, once you can see the screen, you have to run the spawn `rosrun Practicas_lab turtle_spawn.py` and now that we can see the turtle in the set position we have to proceed to run the controller `rosrun Practicas_lab turtle_angle.py`


