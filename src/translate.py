#!/usr/bin/env python
import rospy
from m2_ps4.msg import Ps4Data
from std_msgs.msg import UInt16 
from std_msgs.msg import Bool
# 
old_data = Ps4Data()
servo_const = 90


def callback(data):
    global old_data, servo_const
    servo_vel = 90
    servo_vel_wheel = 90
    hat_dead_zone = 0.1

    # for servo pitch
    if data.hat_ry > hat_dead_zone or data.hat_ry < -hat_dead_zone: 
        servo_vel += servo_const*data.hat_ry
    else:
        servo_vel = servo_const
    pub_servo.publish(int(servo_vel))


    # for servo wheel
    if data.hat_lx > hat_dead_zone or data.hat_lx < -hat_dead_zone: 
        servo_vel_wheel += servo_const*data.hat_lx
    else:
        servo_vel_wheel = servo_const
    pub_servo_wheel.publish(int(servo_vel_wheel))
    

    # for valve
    if (data.r2 == True) and (old_data.r2 == False):
        valve = True
    else: valve = False
    pub_valve.publish(valve)
    old_data = data



if __name__ == '__main__':
    rospy.init_node('ps4_controller')
    
    pub_servo = rospy.Publisher("servo", UInt16, queue_size = 1) # publisher object goes here... 
    pub_servo_wheel = rospy.Publisher("servo_wheel", UInt16, queue_size = 1)
    pub_valve = rospy.Publisher("valve", Bool, queue_size = 1)
    sub = rospy.Subscriber("input/ps4_data", Ps4Data, callback) # subscriber object goes here
    
    rospy.spin()
