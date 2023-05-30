#! /usr/bin/env python

import serial,time
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist

from gpiozero import Servo, AngularServo
from time import sleep

from gpiozero.pins.pigpio import PiGPIOFactory

import math

class Position(Node):

    def __init__(self):

        super().__init__('robot_maker')
        self.subscription_keys = self.create_subscription(String, '/manipulator_key', self.listener_callback, 10)  
        
        factory = PiGPIOFactory()

        self.servo_1 = Servo(12, initial_value =0,min_pulse_width=0.4/1000,max_pulse_width=2.4/1000, pin_factory=factory)
        self.servo_2 = Servo(21, initial_value =0.4, min_pulse_width=0.4/1000,max_pulse_width=2.4/1000, pin_factory=factory)
        self.servo_3 = Servo(16, initial_value =-0.4, min_pulse_width=0.4/1000,max_pulse_width=2.4/1000, pin_factory=factory)
        self.servo_4 = Servo(20, initial_value =0, min_pulse_width=0.4/1000,max_pulse_width=2.4/1000, pin_factory=factory)

    def listener_callback(self, msg):

        info_key = msg.data.split(sep=";")
        self.angulo = float(info_key[0])
        self.num = float(info_key[1])

        if self.num == 1:
            temp = self.servo_1.value+self.angulo
            self.servo_1.value = self.servo_1.value+ (self.angulo if (temp<=1 and temp>=-1) else 0)
            print(self.servo_1.value)
        elif self.num == 2:
            temp = self.servo_2.value+self.angulo
            self.servo_2.value = self.servo_2.value+ (self.angulo if (temp<=1 and temp>=-1) else 0)
        elif self.num == 3:
            temp = self.servo_3.value+self.angulo
            self.servo_3.value = self.servo_3.value+ (self.angulo if (temp<=1 and temp>=-1) else 0)
        elif self.num == 4:
            temp = self.servo_4.value+self.angulo
            self.servo_4.value = self.servo_4.value+ (self.angulo if (temp<=1 and temp>=-1) else 0)

def main(args=None):
        rclpy.init(args=args)
        position_node = Position()

        rclpy.spin(position_node)
    
        Position.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':

        main()

