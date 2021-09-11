#!/usr/bin/env python

from gpiozero import AngularServo as ZeroServo
from gpiozero.pins.pigpio import PiGPIOFactory

import rospy
from rpi_gpio.msg import ServoAngle
from rpi_gpio.srv import ServoCommand

factory = PiGPIOFactory()

class Servo:
    def __init__(self):
        self.node = rospy.init_node('servo')
        self.pub = rospy.Publisher(rospy.get_name() + '/angle', ServoAngle, queue_size=1)
        self.srv = rospy.Service('servo/command', ServoCommand, self.handle_servo_command)
        self.hw = ZeroServo(24, pin_factory=factory, initial_angle=None)
        self.angle = 0.0

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.pub.publish(90.0)
            rate.sleep()

    def handle_servo_command(self, req: ServoCommand):
        if req.enable:
            print("Move to {}".format(req.angle))
            self.hw.angle = req.angle
            self.angle = req.angle
        else:
            print("Disabled")
            self.hw.angle = None

        return self.angle


if __name__ == "__main__":
    try:
        servo = Servo()
        servo.run()
    except rospy.ROSInterruptException:
        pass
