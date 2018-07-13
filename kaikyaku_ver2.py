from __future__ import division
import time

import RPi.GPIO as GPIO

# Import the PCA9685 module.
import Adafruit_PCA9685

# Initialise the PCA9685 using the default address (0x40).wm = Adafruit_PCA9685.PCA9685()
pwm = Adafruit_PCA9685.PCA9685()
# Helper function to make setting a servo pulse width simpler.
def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 50       # 50 Hz
    #print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
    #print('{0}us per bit'.format(pulse_length))
    #pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)
# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(50)

print('chose the moter and angle')
#x=input()
#angle=input()

def angle_pulse(x):
   x = 170 * x / 18 + 500.0
   x=int(x)
   return x

global servo_angle,pulse_servo 
servo_angle=[10,10,10,0,80,90,80,95,120,120,140,140] 
pulse_servo=[0,0,0,0,0,0,0,0,0,0,0,0]

def angle_conv():
        for k in range(12):
                pulse_servo[k]=angle_pulse(servo_angle[k])
        return pulse_servo

servo_angle[0] = 90
servo_angle[1] = 90
angle_conv()
for i in range(12):
	set_servo_pulse(i,pulse_servo[i])
	time.sleep(0.1)

servo_angle=[10,10,10,0,80,90,80,95,120,120,140,140]
angle_conv()
for i in range(12):
        set_servo_pulse(i,pulse_servo[i])
        time.sleep(0.1)

for i in range(16):
  pwm.set_pwm(i, 0, 0)


