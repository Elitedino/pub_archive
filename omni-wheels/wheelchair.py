from inputs import get_gamepad
import wiringpi
import RPi.GPIO as GPIO
import time
import os
import sys
from tabulate import tabulate
from adafruit_servokit import ServoKit
import board
import busio
import adafruit_pca9685
i2c = busio.I2C(board.SCL, board.SDA)
hat = adafruit_pca9685.PCA9685(i2c)
try:
    def _map(x, in_min, in_max, out_min, out_max):
        return (x - in_max) * (out_max - out_min) / (in_max - in_min) + out_min
    drive = 0
    strafe = 0
    rotate = 0
    throttle = 0
    yaxis = 0
    RRPin = 33
    FLPin = 32
    OUTPUT = 3
    finalRL = int
    finalRR = int
    finalFL = int
    finalFR = int
    RRpwm = 0
    FLpwm = 0
    RL_channel = hat.channels[0]
    RR_channel = hat.channels[1]
    FL_channel = hat.channels[2]
    FR_channel = hat.channels[3]
    #GPIO.setmode(GPIO.BOARD)
    #GPIO.setup(RRPin, GPIO.OUT)
    #GPIO.setup(FLPin, GPIO.OUT)
    #RRpwm = GPIO.PWM(RRPin, 350)
    #FLpwm = GPIO.PWM(FLPin, 350)
    #RRpwm.start(0)
    #FLpwm.start(0)
    #print(hat)
    hat.frequency = 350
    while 1:
       events = get_gamepad()
       for event in events:
            #print(event.ev_type, event.code, event.state)
            if event.ev_type == 'Absolute':
                if event.code == 'ABS_X':
                    strafe = _map(event.state, -510, 510, -1, 1)+1
                    #print("x-axis:" + str(_map(xaxis, -510, 510, 1, -1)-1))
                elif event.code == 'ABS_Y':
                    drive = _map(event.state, -510, 510, 1, -1)-1
                    #print("y-axis:" + str(_map(yaxis, -510, 510, 1, -1)-1))
                elif event.code == 'ABS_RZ':
                    rotate = _map(event.state, 0, 255, -1, 1)+2
                    #print("z-axis:" + str(_map(zaxis, 0, 255, 1, -1)-2))
                elif event.code == 'ABS_THROTTLE':
                    throttle = _map(event.state, 0, 255, 3, 0)-3
                    #print("throttle:" + str(_map(throttle, 0, 255, 1, -1)-2))
                frontleft = drive + strafe + rotate
                rearleft = drive - strafe + rotate
                frontright = drive - strafe - rotate
                rearright = drive + strafe - rotate

                finalThrottlemax = _map(throttle, 0, 3, 0, 65534) + 65534
                finalThrottlemin = _map(throttle, 0, 3, 65534,0 ) - 65534
                                
                finalFL = _map(frontleft, 3, -3, 49150.5, 16383.5) - 32812
                #while finalFL > finalThrottlemax:
                 #   finalFL = finalThrottlemax
             #   while finalFL < finalThrottlemin:
             #       finalFL = finalThrottlemin
                finalRL = _map(rearleft, 3, -3, 49150.5, 16383.5) - 32812
               # while finalRL > finalThrottlemax:
              #      finalRL = finalThrottlemax
      #          while finalRL < finalThrottlemin:
       #             finalRL = finalThrottlemin
                finalFR = _map(frontright, 3, -3, 49150.5, 16383.5) - 32812
         #       while finalFR > finalThrottlemax:
          #          finalFR = finalThrottlemax
           #     while finalFR < finalThrottlemin:
            #        finalFR = finalThrottlemin
                finalRR = _map(rearright, 3, -3, 49150.5, 16383.5) - 32812
              #  while finalRR > finalThrottlemax:
              #      finalRR = finalThrottlemax
              #  while finalRR < finalThrottlemin:
              #      finalRR = finalThrottlemin
                #RRpwm.ChangeDutyCycle(finalRR)
                #FLpwm.ChangeDutyCycle(finalFL)
                RR_channel.duty_cycle = int(finalRR)
                FL_channel.duty_cycle = int(finalFL)
                RL_channel.duty_cycle = int(finalRL)
                FR_channel.duty_cycle = int(finalFR)
                d = [
                    ["frontleft",round(finalFL,3)],
                    ["rearleft",round(finalRL,3)],
                    ["frontright",round(finalFR,3)],
                    ["rearright",round(finalRR,3)],
                    ["throttleMax",round(finalThrottlemax,3)],
                    ["throttleMin",round(finalThrottlemin,3)],
		    ]
                print("==="*10 + "\n" + "\n\r".join('{}: {}'.format(*k) for k in enumerate(d)) + "\n" + "==="*10, end='\r')
                #print(finalRR)
except Exception as Error:
    print("Error detected, printing")
    print("====="*7)
    print(Error)
    print("====="*7)
    pass
except KeyboardInterrupt:
    print("\r\n")
    RR_channel.duty_cycle = 0
    FL_channel.duty_cycle = 0
    RL_channel.duty_cycle = 0
    FR_channel.duty_cycle = 0
    time.sleep(0.3)
    os.system("clear")
    time.sleep(0.2)
    print("exitting program")
    exit()
