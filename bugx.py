import math
import numpy as np 
from time import sleep 
import rclpy
from rclpy.node import Node
import tf_transformations as transform
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

class Bug2(Node):
    #Initialization
    def __init__(self):
        super().__init__('bug2')

        # subsribers
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.location_callback, 10)
        self.fl_sensor_sub = self.create_subscription(Range, '/fl_range_sensor', self.fl_sensor_callback, 10)
        self.fr_sensor_sub = self.create_subscription(Range, '/fr_range_sensor', self.fr_sensor_callback, 10)
        self.target_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        # publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bug_algorithm_timer = self.create_timer(0.1, self.bug_algorithm_callback)

        # speed
        self.forward_speed = 0.1
        self.turning_speed = 0.6
        self.turning_speed_yaw_adjustment = 0.08
        self.turning_speed_wf_fast = 0.8
        self.turning_speed_wf_slow = 0.35

        # tolerances
        self.yaw_precision = 2 *(math.pi / 180)
        self.dist_precision = 0.1
        self.distance_to_start_goal_line_precision = 0.1
        self.leave_point_to_hit_point_diff = 0.2

        # limits
        self.dist_thresh_obs = 0.5 
        self.dist_thresh_wf = 0.45 
        self.dist_too_close_to_wall = 0.4 
        self.dist_thresh_bug2 = 0.5 

        # sensors
        self.leftfront_dist = 0.0
        self.rightfront_dist = 0.0

        # current position
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # goal coordinates
        self.goal_x_coordinates = False
        self.goal_y_coordinates = False
        self.goal_idx = 0
        self.goal_max_idx =  None

        # robot mode
        self.robot_mode = "go to goal mode"
        self.go_to_goal_state = "adjust heading"
        self.wall_following_state = "turn left"
        
        # state
        self.bug2_switch = "ON"
        self.start_goal_line_calculated = False

        # Start-Goal Line Parameters
        self.start_goal_line_slope_m = 0
        self.start_goal_line_y_intercept = 0
        self.start_goal_line_xstart = 0
        self.start_goal_line_xgoal = 0
        self.start_goal_line_ystart = 0
        self.start_goal_line_ygoal = 0

        # Wall variables
        self.hit_point_x = 0
        self.hit_point_y = 0
        self.leave_point_x = 0
        self.leave_point_y = 0
        self.distance_to_goal_from_hit_point = 0.0
        self.distance_to_goal_from_leave_point = 0.0
        
        # debug variables
        self.brojac = 0
        self.dodris_linija = 0

        # messages
        self.message1 = ""
        self.message2 = ""
        self.message3 = ""

    def goal_callback(self, msg):
        self.goal_x_coordinates = msg.pose.position.x
        self.goal_y_coordinates = msg.pose.position.y

    def location_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = (
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w)
        self.current_yaw = transform.euler_from_quaternion(q)[2] #[-pi, pi]

    def fl_sensor_callback(self, msg):
        self.leftfront_dist = msg.range

    def fr_sensor_callback(self, msg):
        self.rightfront_dist = msg.range

    def bug_algorithm_callback(self):
        print("")
        print(self.message1)
        print(self.message2)
        print(self.message3)
        print("")

        if self.robot_mode == "obstacle avoidance mode":
            self.avoid_obstacles()
        if self.goal_x_coordinates == False and self.goal_y_coordinates == False:
            return
        self.bug2()

    def avoid_obstacles(self):
        self.message1 = "METHOD: avoid_obstacles"
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
         
        d = self.dist_thresh_obs

        if   self.leftfront_dist > d and self.rightfront_dist > d:
            msg.linear.x = self.forward_speed   

        elif self.leftfront_dist > d and self.rightfront_dist < d:
            msg.angular.z = self.turning_speed  

        elif self.leftfront_dist < d and self.rightfront_dist > d:
            msg.angular.z = -self.turning_speed 

        elif self.leftfront_dist < d and self.rightfront_dist < d:
            msg.angular.z = self.turning_speed  

        else:
            pass
              
        self.cmd_pub.publish(msg)

    def go_to_goal(self):

        self.message1 = "METHOD: go to goal"
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        d = self.dist_thresh_bug2
        d_2 = self.dist_too_close_to_wall

        if (self.leftfront_dist < d or self.rightfront_dist < d_2):
            self.message3 = "MESSAGE: ROBOT JE DOŠAO DO ZIDA"
            self.robot_mode = "wall following mode"
            self.message2 = "ROBOT_MODE = wall following mode"
              
            self.hit_point_x = self.current_x
            self.hit_point_y = self.current_y
                  
            self.distance_to_goal_from_hit_point = math.sqrt(
                (pow(self.goal_x_coordinates - self.hit_point_x, 2)) +
                (pow(self.goal_y_coordinates - self.hit_point_y, 2)))  

            msg.linear.x = 0.0   
            msg.angular.z = self.turning_speed_wf_fast +1
                    
            self.cmd_pub.publish(msg)
                   
            return
                   
        if (self.go_to_goal_state == "adjust heading"):

            self.message3 = "MESSAGE: ROBOT SE PREUSMJERAVA"
             
            desired_yaw = math.atan2(
                    self.goal_y_coordinates - self.current_y,
                    self.goal_x_coordinates - self.current_x)
                    
            yaw_error = desired_yaw - self.current_yaw
             
            if math.fabs(yaw_error) > self.yaw_precision:
             
                if yaw_error > 0:          
                    msg.angular.z = self.turning_speed_yaw_adjustment
                else:
                    msg.angular.z = -self.turning_speed_yaw_adjustment
                 
                self.cmd_pub.publish(msg)
                 
            else:               
                self.go_to_goal_state = "go straight"
                self.message2 = "ROBOT_MODE = go to goal // go straight"
                 
                self.cmd_pub.publish(msg)        
                                        
        elif (self.go_to_goal_state == "go straight"):
             
            position_error = math.sqrt(
                pow(self.goal_x_coordinates - self.current_x, 2) + 
                pow(self.goal_y_coordinates - self.current_y, 2))
                         
                                     
            if position_error > self.dist_precision:
 
                msg.linear.x = self.forward_speed
                     
                self.cmd_pub.publish(msg)
                      
                desired_yaw = math.atan2(
                    self.goal_y_coordinates - self.current_y,
                    self.goal_x_coordinates - self.current_x)
                    
                yaw_error = desired_yaw - self.current_yaw      
         
                if math.fabs(yaw_error) > self.yaw_precision:
                     
                    self.go_to_goal_state = "adjust heading"
                    self.message2 = "ROBOT_MODE = go to goal // adjust heading"

            else:           
                self.go_to_goal_state = "goal achieved"
                self.message2 = "ROBOT_MODE = go to goal // goal achieved"

                self.robot_mode = "done"
                
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.cmd_pub.publish(msg)
        
        elif (self.go_to_goal_state == "goal achieved"):
         
            self.start_goal_line_calculated = False            
         
        else:
            pass

    def follow_wall(self):

        self.message1 = "METHOD: follow wall"
        self.message3 = "MESSAGE: "
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0   
        
        x_start_goal_line = self.current_x
        y_start_goal_line =(self.start_goal_line_slope_m *(x_start_goal_line)) +(self.start_goal_line_y_intercept)
                        
        distance_to_start_goal_line = math.sqrt(
            pow(x_start_goal_line - self.current_x, 2) + 
            pow(y_start_goal_line - self.current_y, 2)) 
                                           
        if distance_to_start_goal_line < self.distance_to_start_goal_line_precision:

            self.message3 = "MESSAGE: DOSLI SMO DO M-LINIJE"
            self.leave_point_x = self.current_x
            self.leave_point_y = self.current_y
            self.distance_to_goal_from_leave_point = math.sqrt(
                pow(self.goal_x_coordinates - self.leave_point_x, 2) + 
                pow(self.goal_y_coordinates - self.leave_point_y, 2)) 
            
            diff = self.distance_to_goal_from_hit_point - self.distance_to_goal_from_leave_point
            self.message3 = "MESSAGE: DOSLI SMO DO M-LINIJE    diff = " +str(diff)

            if diff > self.leave_point_to_hit_point_diff:

                self.message3 = "MESSAGE: ZELIMO ICI DO CILJA diff = " +str(diff)
                self.robot_mode = "go to goal mode"
                self.message2 = "ROBOT_MODE = go to goal"
                self.go_to_goal_state = "adjust heading"
                self.message2 = "ROBOT_MODE = go to goal // adjust heading"
                msg.linear.x = 0.0
                msg.angular.z = self.turning_speed_wf_fast +2 #rotacija lijevo 
                self.cmd_pub.publish(msg)
                return             
         
        d = self.dist_thresh_wf
        d_2 = self.dist_too_close_to_wall
         
        if self.leftfront_dist > d and self.rightfront_dist > d:
            msg.linear.x = self.forward_speed
            msg.angular.z = -self.turning_speed_wf_slow +0.05
             
        elif (self.leftfront_dist > d and self.rightfront_dist < d):
            if (self.rightfront_dist < self.dist_too_close_to_wall):
                msg.linear.x = self.forward_speed
                msg.angular.z = self.turning_speed_wf_fast
            else:           
                msg.linear.x = self.forward_speed
                                     
        elif self.leftfront_dist < d_2 and self.rightfront_dist > d:

            msg.angular.z = self.turning_speed_wf_slow
        
        elif self.leftfront_dist < d and self.rightfront_dist > d:

            msg.linear.x = self.forward_speed
            msg.angular.z = self.turning_speed_wf_slow +0.15
              
        elif self.leftfront_dist < d and self.rightfront_dist < d:
            msg.angular.z = self.turning_speed_wf_slow

        else:
            pass

        self.cmd_pub.publish(msg)

    def bug2(self):
     
        if self.start_goal_line_calculated == False:
         
            self.robot_mode = "go to goal mode"            
            self.start_goal_line_xstart = self.current_x
            self.start_goal_line_xgoal = self.goal_x_coordinates
            self.start_goal_line_ystart = self.current_y
            self.start_goal_line_ygoal = self.goal_y_coordinates
            self.start_goal_line_slope_m = (
                (self.start_goal_line_ygoal - self.start_goal_line_ystart) / 
                (self.start_goal_line_xgoal - self.start_goal_line_xstart))
             
            self.start_goal_line_y_intercept = (
                self.start_goal_line_ygoal - (self.start_goal_line_slope_m * self.start_goal_line_xgoal))
                 
            self.start_goal_line_calculated = True
             
        if self.robot_mode == "go to goal mode":
            self.go_to_goal()     

        elif self.robot_mode == "wall following mode":
            self.follow_wall()

        elif self.robot_mode == "done":
            pass


def main(args=None):

    rclpy.init(args=args)
    bug_node = Bug2()
    rclpy.spin(bug_node)
    bug_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

