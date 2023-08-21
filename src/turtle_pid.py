#!/usr/bin/env python
# license removed for brevity

#turtle-PID controller
#1) get updated time and position of turtlebot
#2) update the initial position as the base point of the reference position
#3) update the initial time as the base time for calculating reference time
#4) calculate the next control command and publish it
#5) repeat the same process

import rospy
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from math import sin,atan2,sqrt,radians,degrees,pi

from tf.transformations import euler_from_quaternion
    
class Position:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0

    def update_position(self,x_new,y_new):
        self.x = x_new
        self.y = y_new

class Quaternion: 
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 0.0
    
    def update(self,x_new,y_new,z_new,w_new):
        self.x = x_new
        self.y = y_new
        self.z = z_new
        self.w = w_new

    def get_zangle(self):
        (roll, pitch, yaw) = euler_from_quaternion([self.x, self.y, self.z, self.w])
        return yaw
    
class Reference:
    def __init__(self):
        self.xi = 0.0
        self.yi = 0.0
        self.x = 0.0
        self.y = 0.0

    def update_init(self,x_new,y_new):
        self.xi = x_new
        self.yi = y_new

    def update_reference(self,dt,size): 
        #dt is the difference of time that has passed from the point when the program started
        dt = dt*3  #speed up the process
        self.x = self.xi + dt/18*5/10*size
        self.y = self.yi + sin(dt/180*pi*5)*size

    def update_rectangle(self,dt,size):
        #this method updates the reference in a rectangular format
        dt = dt/5
        while dt > 4: 
            dt = dt- 4
        
        if dt < 1:
            self.x = self.xi + dt*size
            self.y = self.yi
        elif dt < 2:
            dt = dt - 1
            self.x = self.xi + 1*size
            self.y = self.yi + dt*size
        elif dt < 3:
            dt = dt - 2
            self.x = self.xi + (1-dt)*size
            self.y = self.yi + 1*size
        else:
            dt = dt - 3
            self.x = self.xi
            self.y = self.yi + (1-dt)*size


class Time:
    def __init__(self):
        self.t_ini = 0.0
        self.t_current1 = 0.0
        self.t_current2 = 0.0
    
    def update_init(self,t_new):
        self.t_ini = t_new
        self.t_current1 = t_new
        self.t_current2 = t_new
    
    def update_time(self,t_new):
        self.t_current1 = self.t_current2
        self.t_current2 = t_new

    def Dt(self):       #gives the total time passed so far, for calculating reference position
        return self.t_current2-self.t_ini
    
    def dt(self):       #gives the change of time from previous iteration, used for numerical integration and differentiation
        return self.t_current2-self.t_current1

class Error:
    def __init__(self):
        self.e_v1 = 0.0
        self.e_theta1 = 0.0
        self.e_v2 = 0.0
        self.e_theta2 = 0.0
    
    def update(self,e_v,e_theta):
        self.e_v1 = self.e_v2
        self.e_v2 = e_v
        self.e_theta1 = self.e_theta2
        self.e_theta2 = e_theta

    def get_dv_dt(self,dt): #get differece of error(velocity), used for numerical differentiation
        return (self.e_v2 - self.e_v1)/dt
    
    def get_dtheta_dt(self,dt):    #get difference of error(angular position), used for numerical differentiation
        return (self.e_theta2 - self.e_theta1)/dt
    
class IntegralSum:  
    def __init__(self):
        self.sumV = 0.0
        self.sumTheta = 0.0
    
    def add_error(self,dv,dtheta,dt):
        self.sumV = self.sumV + dv*dt
        self.sumTheta = self.sumTheta + dtheta*dt

    def get_integral_v(self):
        return self.sumV
    
    def get_integral_theta(self):
        return self.sumTheta

def time_callback(data):
    #print(rospy.get_caller_id() + f"Angular Velocity {message.x}")
    #rospy.loginfo(f'The current time is {data.clock.to_sec()}')
    
    #update initial time if the object is new
    #global time1
    if time1.t_ini == 0: 
        time1.update_init(data.clock.to_sec())
    else: 
        time1.update_time(data.clock.to_sec())

    return time1

def pos_update(data):   #update the current position(x,y,z) and quaternion
    #rospy.loginfo(f'The x position is {data.pose.pose.position.x}')
    #global position1
    position1.update_position(data.pose.pose.position.x,data.pose.pose.position.y)
    
    quaternion1.update(data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w)

def get_angleV_V(reference,position,quaternion,kp_v,ki_v,kd_v,kp_w,ki_w,kd_w):  #take reference object, current positio object, and proportional gain as input
    dx = reference.x - position.x
    dy = reference.y - position.y
    dtheta = atan2(dy,dx) - quaternion.get_zangle()  #use atan2 to get reference angle, and subtract current angle from reference to get error
    dv = sqrt(dx**2+dy**2)                              #use distance from current position to reference as error

    #This is where it all went wrong!!! The angle error becomes negative once the angle from atan() function becomes negative, and this caused the robot to diverge
    #This code fix the problem when the angle is shifted, dtheta should always be within from -pi to pi
    if dtheta < -3.14: 
        dtheta = dtheta + 6.28
    elif dtheta > 3.14:
        dtheta = dtheta -6.28
    
    #get the differential change of time
    dt = time1.dt() or 0.1  #can't divide by 0

    error1.update(dv,dtheta)    #update present error into the present sum
    integral1.add_error(dv,dtheta,dt)   #update integral values 

    print(f'Error for angle is {dtheta}, derivative is {error1.get_dtheta_dt(dt)}, integral is {integral1.get_integral_theta()}.')
    print(f'Error for position is {dv}, derivative is {error1.get_dv_dt(dt)}, integral is {integral1.get_integral_v()}')
    print(f'Reference Angle is {atan2(dy,dx)}, current angle is {quaternion.get_zangle()}')

    angular_velocity = dtheta*kp_w + error1.get_dtheta_dt(dt)*kd_w + integral1.get_integral_theta()*ki_w
    
    #gain for angular velocity is changed to a different value (smaller value)
    #if angular_velocity > 1: angular_velocity = 1
    #if angular_velocity < -1: angular_velocity = -1
    #if dv > 0.2: dv = 0.2   #linear velocity shouldn't be too big

    #if dv<0.1: angular_velocity = 0     #make angular velocity 0 if the error is smaller than a threshold

    return angular_velocity ,dv*kp_v + error1.get_dv_dt(dt)*kd_v + integral1.get_integral_v()*ki_v     #multiply error by proportional gain to get final output

def pid_turtle():
    global time1
    time1 = Time()
    global position1
    position1 = Position()
    global reference1
    reference1 = Reference()
    global quaternion1
    quaternion1 = Quaternion()
    global error1
    error1 = Error()
    global integral1
    integral1 = IntegralSum()

    kp_v = float(input('Enter kp for linear velocity: '))      #define proportional gain for linear velocity(kp) 
    ki_v = 0    #integral gain for linear velocity
    kd_v = 0    #derivative gain for linear velocity

    kp_w = float(input('Enter kp for angular velocity: '))      #define proportional gain for angular velocity(kp)
    ki_w = 0    #integral gain for angular velocity
    kd_w = 0    #derivative gain for angular velocity

    #kp_v = 1 and kp_w = 2 will make the turtlebot track reference almost perfectly when reference_size = 1
    #larger reference size will require kp_v to be smaller for turtlebot to not diverge

    reference_size = 1
    #reference size = 4 will cause the turtlebot to diverge in square pattern, because reference is updating at a very fast rate
    #reference_size = 1 is good 


    while not rospy.is_shutdown():
        rospy.init_node('pid_turtle', anonymous=True)

        rospy.Subscriber('/odom', Odometry, pos_update)
        #read odometry data to get current position
        #have to pass the object in main function to the callback function
        

        rospy.Subscriber('/clock', Clock, time_callback) 
        #uses the optional argument parameter to pass pos as one of the arguements when calling the pose_callback function
        #Pose message is going to be the first argument when calling back the pose_callback function
        

        #update reference value based on new time and initial condition
        if reference1.xi == 0 and reference1.yi == 0:
            reference1.update_init(position1.x,position1.y)
        else:
            reference1.update_rectangle(time1.Dt(),reference_size)

        print(f'The reference position is x = {reference1.x}, y = {reference1.y}')
        print(f'The current position is x = {position1.x}, y = {position1.y}')
        print()

        #create publisher object for velocity
        pub = rospy.Publisher('/cmd_vel',Twist, queue_size=100)
        rate = rospy.Rate(10)
        #create twist message to send
        twist_msg = Twist()

        #use function to get right value needed to operate the robot
        twist_msg.angular.z,twist_msg.linear.x = get_angleV_V(reference1,position1,quaternion1,kp_v,ki_v,kd_v,kp_w,ki_w,kd_w)
        
        pub.publish(twist_msg)
        rate.sleep()

if __name__ == '__main__':
    pid_turtle()


