#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32

import time
import board
#import adafruit_tlv493d
import RPi.GPIO as GPIO

class brake_node:
    def __init__(self):
        self.brake_sub = rospy.Subscriber("cmd/brake", Int32, self.callback)
        self.brake_enc_sub = rospy.Subscriber("brake_enc", Int32, self.callback_enc)
        # self.brake_magY_sub = rospy.Subscriber("brake_magY", Int32, self.callback_magY)
        # self.brake_magZ_sub = rospy.Subscriber("brake_magZ", Int32, self.callback_magZ)
        self.loop_rate = rospy.Rate(10)
        # self.i2c = board.I2C()   # Uses board.SLC and board.SDA
        # self.tlv = adafruit_tlv493d.TLV493D(self.i2c)

        time.sleep(2)
        # i2c = board.I2C()   # Uses board.SLC and board.SDA
        # tlv = adafruit_tlv493d.TLV493D(i2c)

        # Control gain
        self.kp = 5
        self.ki = 0.01
        self.err_i = 0

        # Pin and direction definiton
        #ENA = 17 # Enable pin directly connected to GND
        self.DIR = 4
        self.PUL = 17
        self.CW = 0
        self.CCW = 1
        self.spd = 10000 # steps per second
        self.motor_value = 0
        self.brake_enc = 0

        self.brake_magY = 0
        self.brake_magZ = 0

        self.magValue = 4500
        self.homeFlag = False

        self.current_step = 0

        GPIO.setup(self.DIR, GPIO.OUT)
        GPIO.setup(self.PUL, GPIO.OUT)
        self.Movebackward(200)
        self.findhome()
        time.sleep(1)

    # Functions
    def moveSteps(self, steps):
        for k in range(1,steps):
            GPIO.output(self.PUL,1)
            time.sleep(1.0/self.spd/2)
            GPIO.output(self.PUL,0)
            time.sleep(1.0/self.spd/2)

    def Moveforward(self, steps): # Going to home position
        GPIO.output(self.DIR,self.CW) # Let CW forward
        self.moveSteps(steps)

    def Movebackward(self, steps): # Pressing Brake
        GPIO.output(self.DIR,self.CCW)
        self.moveSteps(steps)

    def findhome(self):
        # if self.brake_magY <0:
        #     while abs(self.brake_magZ) < self.magValue:
        #         print(abs(self.brake_magZ))
        #         self.Movebackward(2)
        #         time.sleep(1/self.spd)

        # else:
        #     while abs(self.brake_magZ) < self.magValue:
        #         print(abs(self.brake_magZ))
        #         self.Moveforward(2)
        #         time.sleep(1/self.spd)

        if self.brake_enc < 10:
            while self.brake_enc <10:
                self.Movebackward(3)

        while self.brake_enc <0:
                self.Moveforward(3)    

        # self.brake_enc

        #self.Moveforward(800)

    def callback(self, data):
        self.brake_percent = data.data
        # print("Data: %d" %data.data)
        #self.motor_value = 20*int(self.brake_percent)
        self.motor_value = int(self.brake_percent)

    def callback_enc(self, data):
        self.brake_enc = data.data
        # print("Data: %d" %data.data)

    def callback_magY(self, data):
        self.brake_magY = data.data
        # print("Data: %d" %data.data)

    def callback_magZ(self, data):
        self.brake_magZ = data.data
        # print("Data: %d" %data.data)

    def start(self):
        while not rospy.is_shutdown():
            """
            if self.motor_value-self.current_step < 0:
                self.Moveforward(abs(self.motor_value-self.current_step))
            elif self.motor_value-self.current_step > 0:
                self.Movebackward(abs(self.motor_value-self.current_step))
            else:
                self.Moveforward(0)
            """

            err = self.motor_value-self.brake_enc
            brake_input = self.kp * err + self.ki * self.err_i

            if brake_input > 40:
                brake_input = 40
            if brake_input <-40:
                brake_input = -40

            if brake_input < 0 and self.homeFlag == False:
                self.Moveforward(int(abs(brake_input)))
            elif brake_input > 0: # Pushing brake
                self.Movebackward(int(abs(brake_input)))
            else:
                self.Moveforward(0)            

            if self.brake_enc <0:
                self.Moveforward(0)
                self.homeFlag = True
                print("Home")
            else:
                self.homeFlag = False

            # if abs(self.brake_magZ) > self.magValue:
            #     self.Moveforward(0)
            #     self.homeFlag = True
            #     print("Home")
            # else:
            #     self.homeFlag = False

            # print("Motor_value: %d" %self.motor_value)
            # print("Current_value: %d" %self.current_step)

            self.err_i = self.err_i + err

            print("Motor_value: %d" %self.motor_value)
            print("Encoder_value: %d" %self.brake_enc)
            print("brake_input: %d" %brake_input)

            # self.current_step=self.motor_value
            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node('brake_driver_node', anonymous=True)
    bnode = brake_node()
    bnode.start()
