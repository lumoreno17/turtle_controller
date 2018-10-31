# turtle_controller
Position controller for the ROS turtlesim turtle

The goal of this project is create a position controller using a PID for the turtle from ROS tutorials. 

How to Use
------------------------------------------------


First, open the terminal and on catkin_ws/src type:

```
git clone https://github.com/lumoreno17/turtle_controller.git

cd ..

catkin_make

```

To run the node you need to run the turtlesim_node first:

```
rosrun turtlesim turtlesim_node
```

than you open other terminal window and type

```
rosrun turtle_controller PID_control.py
```

The code ask you to write the desired point of destination, just type and see your turtle go :)


