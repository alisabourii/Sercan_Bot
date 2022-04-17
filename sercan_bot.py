#!/usr/bin/env python
from __future__ import division

import rospy
import math
import time

from rosrider_lib.rosrider import ROSRider

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

from tf.transformations import quaternion_from_euler, euler_from_quaternion


class LineFollower:

    def __init__(self):

        # initialize robot, giving name, update rate, and process_image  callback handler
        self.robot = ROSRider('robot1')
        # this array will hod the brigtness data from the sensor
        self.sensor_row = []

        # this flag is true, only when robot is cleared to start
        self.started = False

        # NOTICE: odometry data can be accessed by
        # self.robot.x, self.robot.y, self.robot.yaw, self.robot.vel_x, and self.robot.vel_z

        # ENTER YOUR INITIALIZATION HERE

        ##      ROBOT METRIKLERI
        self.sensor_distance = 0.38
        self.sensor_lenght = 0.36
        self.sensor_number = 32
        self.angle_radian = math.atan2((self.sensor_lenght/self.sensor_number),self.sensor_distance)
        ##

        self.timeperframe = 1/20
        self.onetime_process = 1

        ##      PID
        self.Kp = 3 #3.15
        self.Kd = 2 #0.0001
        self.Ki = -0.1 #0
        self.pid_divider = 1
        self.prew_sum = []
        self.prew_sum.append(0)
        self.prew_err = 0
        self.sum_integral = 10
        ##

        ##      Speed
        self.maksimum_speed =0.96 #1.05
        self.rounded_speed = 0.81 #0.85
        self.straight_speed =0.93 #0.95

        self.speed_cor = 1.012
        ##


        ##      Line Detection
        self.line_rounded_max = 16
        self.line_type = "line"
        self.line_direction = "left"
        self.line_width = 6 #line thickness
        ##


        ##      Sensor Read
        self.last_error_func = 0
        self.sensor_max_value = 255
        ##


        # wait for evaluator, do not remove
        rospy.wait_for_message('/simulation_metrics', String)

        # grace period, do not remove
        time.sleep(0.5)
        
        self.started = True

        self.a = 0

    def main(self):
        # program will process incoming callbacks, namely process_image, and process_odom until shutdown.
        while self.robot.is_ok():
            # if robot is shutting down, or not yet started, do not process image
            if self.robot.is_shutdown or not self.started:
                return
            # reinitialize the array each time, new image arrives
            """self.sensor_row = []

            image = self.robot.image_data()

            # NOTICE: The below code is left as an example. It is the competitors duty, to make it function right.
            # Or you may develop a better method that locks to yellow
            # The code below calculates brightness per pixel, and append it in array sensor_row
            for i in range(image.width):
                brightness = (0.2126 * ord(image.data[i * 3])) + (0.7152 * ord(image.data[i * 3 + 1])) + (0.0722 * ord(image.data[i * 3 + 2]))
                self.sensor_row.append(brightness)"""
            self.sensor_row = self.robot.get_sensor_data()

            # one time process for calibration
            if self.onetime_process == 1:
                self.sensor_max_value = self.sensor_row[16]
                self.middle_point = self.read_sensor()
                self.onetime_process = 0

            # every loop main process
            sensor_degeri = self.read_sensor()
            error = self.error_calculation(sensor_degeri)
            self.line_detect(sensor_degeri)
            out_pid = self.pid_process(error) * (-1)
            spd = self.speed_calculate()
            lef = spd - out_pid
            rig = spd + out_pid
            self.motor_run(lef,rig)

        # rospy.spin has finished waiting, program is shutdown, so send stop to robot.
        self.robot.stop()


    # sensor read
    def read_sensor(self):
        self.sensor_on_line = 0
        size = len(self.sensor_row)
        onLine = False
        avg = 0
        sum = 0
        for i in range(size):
            value = self.sensor_row[i]
            if value > self.sensor_max_value / 4 :
                onLine = True
            if value > self.sensor_max_value / 20:
                avg += value * (i*self.sensor_max_value)
                sum += value
                self.sensor_on_line +=1
        if not onLine:
            if(self.last_error_func < (size-1)*self.sensor_max_value/2):
                return 0
            else:
                return (size-1)*self.sensor_max_value
        self.last_error_func = avg/sum
        return self.last_error_func
        
    # line detection
    def line_detect(self,position):
        if self.sensor_on_line ==0:
            self.line_type = "blank"
            self.line_dir = ""

        elif self.sensor_on_line > self.line_rounded_max:
            self.line_type = "sharp"
            if position == -1:
                self.line_dir = "left"
            elif position == -2:
                self.line_dir = "right"

        elif self.sensor_on_line <= self.line_rounded_max and self.sensor_on_line > self.line_width:
            self.line_type = "rounded"
            if position < self.middle_point:
                self.line_dir = "left"
            elif position > self.middle_point:
                self.line_dir = "right"
            else:
                self.line_dir = "middle"

        elif self.sensor_on_line <= self.line_width:
            self.line_type = "straight"
            if position < self.middle_point - self.sensor_max_value*1.1:
                self.line_dir = "left"
            elif position > self.middle_point + self.sensor_max_value*1.1:
                self.line_dir = "right"
            else:
                self.line_dir = "middle"

    # error calculate
    def error_calculation(self,position):
        error_value = position - self.middle_point
        return (error_value / self.sensor_max_value) * self.angle_radian

    # PID process
    def pid_process(self,error):
        output = error * self.Kp + (error - self.prew_err) * self.Kd + self.Ki * sum(self.prew_sum)
        self.prew_err = error
        self.prew_sum.append(error)
        if len(self.prew_sum) == self.sum_integral:
            self.prew_sum.pop(0)
        return output / self.pid_divider

    # speed calculation based on line type
    def speed_calculate(self):
        if "straight" in self.line_type and "middle" in self.line_dir:
            speed = self.maksimum_speed
        elif "straight" in self.line_type:
            speed = self.straight_speed
        elif "rounded" in self.line_type:
            speed = self.rounded_speed
        else:
            speed = self.rounded_speed
        return speed

    # motion block
    def motor_run(self,in1,in2):
        self.robot.move(((in1+in2)/2)*self.speed_cor)
        self.robot.rotate((in2-in1)*self.speed_cor)

if __name__ == '__main__':
    try:
        node = LineFollower()
        node.main()
    except rospy.ROSInterruptException:
        pass
