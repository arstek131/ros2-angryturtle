import rclpy
from rclpy.node import Node
from rclpy.task import Future

import sys
from math import *
import random

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import *
from turtlesim.srv import *

import time

class AngryTurtle(Node):
    def __init__(self, goal_pose, tolerance):
        super().__init__('angry_turtle')

        self.point_list = goal_pose

        next_point = self.point_list[0]
        self.point_list.append(next_point)

        self.goal_pose = Pose()
        self.goal_pose.x = float(next_point[0])
        self.goal_pose.y = float(next_point[1])
        self.goal_pose.theta = float(next_point[2])

        self.tolerance = tolerance
        self.current_pose = None

        self.counter = 0

        self.vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.turtle_pose_callback, 10)

        self.srv_setpen = self.create_client(SetPen, '/turtle1/set_pen')

        self.srv_spawn = self.create_client(Spawn, '/spawn')
        self.spawn_enemy_turtle()
                                                                 
        self.set_pen(1,1)
        
        
    def spawn_enemy_turtle(self):
        req2 = Spawn.Request()
        req2.x = round(random.uniform(1,10),1)
        req2.y = round(random.uniform(1,10),1)
        req2.theta = 0.2
        req2.name = 'offender' #Also name could be randomized
        self.target_name = req2.name

        self.srv_spawn.call_async(req2)
        
        self.target = Pose() 
        self.pose_subscriber = self.create_subscription(Pose, f'/{self.target_name}/pose', self.controlled_turtle_pose_callback, 10)
        self.srv_clear = self.create_client(Empty, '/clear')
        self.srv_setpen_s = self.create_client(SetPen, f'/{self.target_name}/set_pen')

        self.target_vel_publisher = self.create_publisher(Twist, f'/{self.target_name}/cmd_vel', 10)
        self.target_vel_msg = Twist()
       

    def random_walking(self):
        self.target_vel_msg.linear.x = round(random.uniform(1,1.5),1)
        self.target_vel_msg.angular.z = round(random.uniform(-3,5),1)
        self.target_vel_publisher.publish(self.target_vel_msg)

    def set_pen(self, off, flag, r=200, g=200, b=200, width = 2):
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = off
        if flag:
            self.srv_setpen.call_async(req)
        else:
            self.srv_setpen_s.call_async(req)

    
    def move(self):
        self.timer = self.create_timer(0.1, self.draw_callback)
        self.done_future = Future()
        
        return self.done_future     

       
    def turtle_pose_callback(self, msg):
        self.current_pose = msg
        self.current_pose.x = round(self.current_pose.x, 4)
        self.current_pose.y = round(self.current_pose.y, 4)
        self.current_pose.theta = round(self.current_pose.theta, 4)
        
    def controlled_turtle_pose_callback(self, msg):                                                         
        self.target = msg
        self.target.x = round(self.target.x, 4)
        self.target.y = round(self.target.y, 4)
        self.target.theta = round(self.target.theta, 4)
        self.set_pen(0, 0, 165,42,42)
        
    def draw_callback(self):
        self.random_walking()
        if self.current_pose is None:
            return
            
        if self.target != None and self.euclidean_distance(self.target, self.current_pose) <= 2: 
            if self.euclidean_distance(self.target, self.current_pose) <= self.tolerance:
                srv_kill = self.create_client(Kill, '/kill')
                req = Kill.Request()
                req.name = self.target_name
                srv_kill.call_async(req)
                self.target = None
                self.srv_clear.call_async(Empty.Request())

                self.counter = 0
                self.set_pen(1,1)
                self.goal_pose = Pose()
                self.goal_pose.x = float(self.point_list[self.counter][0])
                self.goal_pose.y = float(self.point_list[self.counter][1])
                self.goal_pose.theta = float(self.point_list[self.counter][2])
            else:
                self.set_pen(1,1)
                self.move_2_goal(self.target, constant1=4, constant2=6)
        
        elif self.euclidean_distance(self.goal_pose, self.current_pose) >= self.tolerance:
            self.move_2_goal(self.goal_pose)
            
        else:
            if(abs(self.angle_difference(self.goal_pose, self.current_pose)) < 0.017):
                next_point = self.point_list[self.counter]
                self.point_list.append(next_point)

                self.goal_pose.x = float(next_point[0])
                self.goal_pose.y = float(next_point[1])
                self.goal_pose.theta = float(next_point[2])

                #print(self.counter) Just for debug purposes

                if self.counter in [6,12]:
                    self.set_pen(1,1)
                else:
                    self.set_pen(0,1, width=4)

                self.counter += 1

                if self.counter == 15:
                    self.set_pen(1,1)
            else:                                                  
                self.rotate_2_goal(self.goal_pose)
                
            if self.counter == 16:
                self.stop_walking()
    
    def stop_walking(self):
        self.get_logger().info("Goal reached, shutting down...")
        cmd_vel = Twist() 
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.vel_publisher.publish(cmd_vel)
        self.done_future.set_result(True)
    

    def rotate_2_goal(self, rotation_pose):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = self.angular_vel_rot(rotation_pose, self.current_pose)

        self.vel_publisher.publish(cmd_vel)

    def move_2_goal(self, goal_pose, constant1=1.5, constant2=6):
        cmd_vel = Twist() 
        cmd_vel.linear.x = self.linear_vel(goal_pose, self.current_pose, constant1)
        cmd_vel.angular.z = self.angular_vel(goal_pose, self.current_pose, constant2)

        self.vel_publisher.publish(cmd_vel)    


    def euclidean_distance(self, goal_pose, current_pose):
        return sqrt(pow((goal_pose.x - current_pose.x), 2) +
                    pow((goal_pose.y - current_pose.y), 2))

    def linear_vel(self, goal_pose, current_pose, constant=1.5):
        return constant * self.euclidean_distance(goal_pose, current_pose)

    def steering_angle(self, goal_pose, current_pose):
        return atan2(goal_pose.y - current_pose.y, goal_pose.x - current_pose.x)

    def angular_vel(self, goal_pose, current_pose, constant=8):
        return constant * atan2(sin(self.steering_angle(goal_pose, current_pose) - self.current_pose.theta),
                                 cos(self.steering_angle(goal_pose, current_pose) - self.current_pose.theta))

    def angle_difference(self, goal_pose, current_pose):
        return atan2(sin(goal_pose.theta - current_pose.theta), cos(goal_pose.theta - current_pose.theta))
    def angular_vel_rot(self, goal_pose, current_pose, constant=4):             
        return constant * self.angle_difference(goal_pose, current_pose)


def main():
    time.sleep(1)
    
    tolerance = 0.3
    USI = [[1.0, 8.0, 3*pi/2], [1.0, 5.0, 5 * pi / 3], [2.0, 4.0, 0.0], [3.0, 4.0, pi / 6],
                 [4.0, 5.0, pi / 2], [4.0, 8.0, 0.0],
                 [8.0, 8.0, 5 * pi / 6], [5.5, 7.0, 5 * pi / 4],
                 [5.5, 6.3, 11 * pi / 6], [8.0, 5.7, 7 * pi / 4],
                 [8.0, 5.0, 4 * pi / 3], [5.5, 4.0, 0.0],
                 [9.5, 4.0, pi / 2], [9.5, 8.0,pi / 2]]

    rclpy.init()
    
    
    node = AngryTurtle(USI, tolerance)
    done = node.move()
    

    rclpy.spin_until_future_complete(node, done)
    

if __name__ == '__main__':
    main()
