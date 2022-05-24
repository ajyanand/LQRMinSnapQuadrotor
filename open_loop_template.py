#!/usr/bin/env python
# license removed for brevity


import rospy
from std_msgs.msg import String
import math
import geometry_msgs.msg
import tf
import numpy as np


def cmd_command(x = 0,y = 0,z = 0.5):
    
    cmd = geometry_msgs.msg.Twist()
    ##
    ## INSERT YOUR CODE HERE
    ##
    cmd.linear.x = x
    cmd.linear.y = y
    cmd.linear.z = z
    ##
    ##
    ##
    return cmd
class calc_vel_PID:
    """
    	Discrete PID control
    	"""

    def __init__(self, P=1, I=0.25, D=0.5, Derivator=np.array([0,0,0]), Integrator=np.array([0,0,0]), Integrator_max=500,Integrator_min=-500):

        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.Derivator = Derivator
        self.Integrator = Integrator
        self.Integrator_max = Integrator_max
        self.Integrator_min = Integrator_min

        self.set_point = np.array([0,0,0])
        self.error = np.array([0,0,0])

    def update(self, current_value):
        """
        Calculate PID output value for given reference input and feedback
        """

        self.error = self.set_point - np.array(current_value)

        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * (self.error - self.Derivator)
        self.Derivator = self.error

        self.Integrator = self.Integrator + self.error

        for i in range(3):
            if self.Integrator[i] > self.Integrator_max:
                self.Integrator[i] = self.Integrator_max
            elif self.Integrator[i] < self.Integrator_min:
                self.Integrator[i] = self.Integrator_min

        self.I_value = self.Integrator * self.Ki

        PID = self.P_value + self.I_value + self.D_value

        return PID[0],PID[1],PID[2]
    def setPoint(self, set_point):
        """
        Initilize the setpoint of PID
        """
        self.set_point = np.array(set_point)
        self.Integrator = 0
        self.Derivator = 0


def calc_vel(trans,des):
    #rospy.logdebug('printing trans and des, %s,%s',trans,des)
    Kp = 0.1
    x = Kp*(des[0] - trans[0])
    y = Kp*(des[1] - trans[1]) 
    z = Kp*(des[2] - trans[2])
    #rospy.loginfo('inside calc_vel,%s,%s,%s',x,y,z)
    return x,y,z

def closed_loop():
    #sets up a node to read and publish data
    rospy.init_node('closed_loop_control', anonymous=True)
    
    #create a tf listener
    listener = tf.TransformListener()

    #sets up the ros publisher for cmd_command to later publish to.
    quad_vel = rospy.Publisher('cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
    rate = rospy.Rate(10) # 10hz
    vel = calc_vel_PID()
    while not rospy.is_shutdown():
        #reads the transformation data and prints it to console
        try:
            (trans,rot) = listener.lookupTransform('world','base_stabilized', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        #rospy.loginfo('translation: %s ,rotation: %s',trans,rot)
        des = [3,3,10]
        vel.setPoint(des)
        rospy.loginfo('current position:%s',trans)
        rospy.loginfo('desired position:%s',des)
        x,y,z = vel.update(trans)
        rospy.loginfo('calculated x,y and z velocities,%s,%s,%s',x,y,z)
        quad_vel.publish(cmd_command(x,y,z))
        rate.sleep()

if __name__ == '__main__':
	try:
		closed_loop()
	except rospy.ROSInterruptException:
		pass
