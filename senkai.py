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

"""ang1=  180
ang2 = 10
ang3 = 120
angle3_pulse = angle_pulse(ang3)
time.sleep(0.1)

for l in range(4):
	set_servo_pulse(l+8,angle3_pulse)
for i in range(10):
	ang1_pulse = angle_pulse(ang1)
	ang2_pulse = angle_pulse(ang2)
	for k in range(4):
		set_servo_pulse(k,ang1_pulse)
		time.sleep(0.1)
	for j in range(4):
		print(ang2)
		set_servo_pulse(j+4,ang2_pulse)
		time.sleep(0.1)
	ang1-=18
	ang2+=9
"""

global servo_angle,pulse_servo
servo_angle=[5,10,10,10,110,100,100,100,90,90,90,90]
pulse_servo=[0,0,0,0,0,0,0,0,0,0,0,0]

def angle_conv():
        for k in range(12):
                pulse_servo[k]=angle_pulse(servo_angle[k])
        return pulse_servo

def leg_move(servo_num,ang_ser):
	servo_angle[servo_num-4] -= 20
	angle_conv() 
	print(servo_angle[servo_num-4])
	set_servo_pulse(servo_num-4,pulse_servo[servo_num-4])
	time.sleep(0.1)
	servo_angle[servo_num] += ang_ser
        angle_conv()
        print(servo_angle[servo_num])
        set_servo_pulse(servo_num,pulse_servo[servo_num])
	time.sleep(0.1)
	servo_angle[servo_num-4] += 20
        angle_conv()
        print(servo_angle[servo_num-4])
        set_servo_pulse(servo_num-4,pulse_servo[servo_num-4])
        time.sleep(0.1)

def body_move(servo_num,ang_ser):
	servo_angle[servo_num] += ang_ser
        angle_conv()
        print(servo_angle[servo_num])
        set_servo_pulse(servo_num,pulse_servo[servo_num])
        #time.sleep(0.05)

angle_conv()
for i in range(12):
	set_servo_pulse(i,pulse_servo[i])
        time.sleep(0.1)

def turn(ang_turn):
	ang_turn = ang_turn / 4.0
	for k in range(4):
		leg_move(9,ang_turn)
		leg_move(10,ang_turn)
		leg_move(11,ang_turn)
		leg_move(8,ang_turn)
		body_move(8,-ang_turn)
		body_move(9,-ang_turn)
		body_move(10,-ang_turn)
		body_move(11.-ang_turn)

ang_turn = input()
turn(ang_turn)
for i in range(16):
  pwm.set_pwm(i, 0, 0)


