#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from PID import PID
import math

#Define turtle controller class 
class turtle_PID():


    #Initialize ROS parameters, the PID controller and the constants Kp, Ki and Kd
    def __init__(self):

        self.pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("turtle1/pose", Pose, self.pose_callback)

        rospy.init_node('turtle_controller', anonymous=True)
        self.rate = rospy.Rate(100) # 10hz

        self.angle_controller = PID()
        self.distance_controller = PID()

        self.angle_controller.setKp(1.2)
        self.angle_controller.setKi(0)
        self.angle_controller.setKd(0)

        self.distance_controller.setKp(1.2)
        self.distance_controller.setKi(0)
        self.distance_controller.setKd(0)

        self.msg = Twist()

        self.move2goal()

        
    #Get the goal point from user
    def get_goal(self):

        self.goal_x = input("Insert your x goal:")
        self.goal_y = input("Insert your y goal:")


    #Calculate the angle rotation necessary to point to the goal
    def angle_calculation(self):

        self.R = math.sqrt(math.pow(self.current_pose_x - self.goal_x , 2) + math.pow(self.current_pose_y - self.goal_y , 2))

        self.xr = self.R*math.cos(self.current_angle)
        self.yr = self.R*math.sin(self.current_angle)

        self.xim = self.current_pose_x + self.xr
        self.yim = self.current_pose_y + self.yr

        self.C = math.sqrt(math.pow(self.xim - self.goal_x , 2) + math.pow(self.yim - self.goal_y , 2))

        if self.xim > self.goal_x:

            self.alpha = math.acos((2*math.pow(self.R,2) - math.pow(self.C,2))/(2*math.pow(self.R,2)))
        else:
            self.alpha = 2*3.14*math.acos((2*math.pow(self.R,2) - math.pow(self.C,2))/(2*math.pow(self.R,2)))
        
        return self.alpha

    #Calculate the distance between the start and the goal point
    def distance_calculation(self):

        self.distance = math.sqrt(math.pow(self.goal_x - self.current_pose_x , 2) + math.pow(self.goal_y - self.current_pose_y, 2 ))
        
        #print "distance: " + str(self.distance)
        return self.distance
    
    #Do the angle correction using a PID controller
    def angle_correction(self):

        self.angle_diff = self.angle_calculation()  

        while self.angle_diff>0.005: 
            
            self.angle_diff = self.angle_calculation() 

            self.PID_angle = self.angle_controller.update(self.angle_diff)

            self.msg.angular.z = self.PID_angle

            self.pub.publish(self.msg)    

    #Do the distance correction using a PID controller
    def distance_correction(self):

        self.distance_diff = self.distance_calculation()
        
        while self.distance_diff > 0.05:

            self.distance_diff = self.distance_calculation()

            self.PID_distance = self.distance_controller.update(self.distance_diff)

            self.msg.linear.x = self.PID_distance

            self.pub.publish(self.msg)

    
    #Call the PID controllers
    def move2goal(self):

        self.get_goal()

        self.angle_correction()

        self.distance_correction()

     
    #Get the pose in x, y and the tetha angle
    def pose_callback(self,data):

        self.current_pose_x = data.x
        self.current_pose_y = data.y
        self.current_angle = data.theta

if __name__ == '__main__':

    try:
        #Call turtle position controller
        turtle_PID()

    except rospy.ROSInterruptException:

        pass