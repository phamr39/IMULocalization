#!/usr/bin/env python

import RPi.GPIO as gpio
import time
import sys
import signal
import rospy
from std_msgs.msg import Float32

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
# ROS communication
class sonar():
    def __init__(self):
        rospy.init_node('sonar', anonymous=True)
        self.distance_publisher = rospy.Publisher('/sonar_dist',Float32, queue_size=1)
        self.r = rospy.Rate(15)
    def dist_sendor(self,dist):
        data = Float32()
        data.data=dist
        self.distance_publisher.publish(data)
class UltraSonic():
    def __init__(self,trig_pin,echo_pin,pos=''):
        self.trig = trig_pin
        self.echo = echo_pin
    def GetDistance(self):
        gpio.setmode(gpio.BCM)
        trig = self.trig
        echo = self.echo

        gpio.setup(trig, gpio.OUT)
        gpio.setup(echo, gpio.IN)

        sensor=sonar()
        time.sleep(0.5)
        print (str(pos+'Sonar start'))
        try :
            while True :
                gpio.output(trig, False)
                time.sleep(0.1)
                gpio.output(trig, True)
                time.sleep(0.00001)
                gpio.output(trig, False)
                while(gpio.input(echo) == 0):
                    pulse_start = time.time()
                while(gpio.input(echo) == 1):
                    pulse_end = time.time()
                pulse_duration = pulse_end - pulse_start
                distance = pulse_duration * 17000
                if(pulse_duration >=0.01746):
                    #print('time out')
                    continue
                elif(distance > 300 or distance==0):
                    #print('out of range')
                    continue
                distance = round(distance, 3)
                #print ('Distance : %f cm'%distance)
                # sensor.dist_sendor(distance)
                sensor.r.sleep()      
        except (KeyboardInterrupt, SystemExit):
            gpio.cleanup()
            sys.exit(0)
        except:
            gpio.cleanup()
        return distance
class GetObs():
    def setup():
        front_sensor = UltraSonic(27,17,'front')
        left_sensor = UltraSonic(27,17,'left')
        right_sensor = UltraSonic(27,17,'right')
        pub_signal = sonar()
    def run():
        GetObs.setup()
        Message = 0
        # Get sensor data
        dis_front = front_sensor.GetDistance()
        dis_left = left_sensor.GetDistance()
        dis_right = right_sensor.GetDistance()
        # Create and publish the signal to change the formation
        if(dis_front < 30 or dis_left < 30 or dis_right <30):
            Message = 1
        else:
            Message = 0
        pub_signal.dist_sendor(Message)
if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    GetObs.run()