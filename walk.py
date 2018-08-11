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
servo_angle=[0,90,120,0,90,120,0,90,120,0,90,120] 
pulse_servo=[0,0,0,0,0,0,0,0,0,0,0,0]

def angle_conv():
        for k in range(12):
                pulse_servo[k]=angle_pulse(servo_angle[k])
        return pulse_servo

def leg_move(servo_num,ang_ser):
	servo_angle[servo_num-1] -= 45
	angle_conv() 
	#print(servo_angle[servo_num-4])
	set_servo_pulse(servo_num-1,pulse_servo[servo_num-1])
	time.sleep(0.1)
	servo_angle[servo_num] += ang_ser
        angle_conv()
        #print(servo_angle[servo_num])
        set_servo_pulse(servo_num,pulse_servo[servo_num])
	time.sleep(0.1)
	servo_angle[servo_num-1] += 45
        angle_conv()
        #print(servo_angle[servo_num-4])
        set_servo_pulse(servo_num-1,pulse_servo[servo_num-1])
        time.sleep(2)

def body_move_1(num_1,num_2,num_3,num_4,ang_ser):
	ang_ser = ang_ser / 20.0
	for i in range(20):
		servo_angle[num_1] -= ang_ser
		servo_angle[num_2] += ang_ser
		servo_angle[num_3] -= ang_ser
		servo_angle[num_4] += ang_ser
       		angle_conv()
        	#print(servo_angle[servo_num])
        	set_servo_pulse(num_1,pulse_servo[num_1])
        	#time.sleep(0.01)
		set_servo_pulse(num_2,pulse_servo[num_2])
                #time.sleep(0.01)
		set_servo_pulse(num_3,pulse_servo[num_3])
                #time.sleep(0.01)
		set_servo_pulse(num_4,pulse_servo[num_4])
                #time.sleep(0.1)

angle_conv()
for i in range(12):
	set_servo_pulse(i,pulse_servo[i])
	time.sleep(0.1)
leg_move(2,-30)
leg_move(8,30)
for k in range(5):
	leg_move(2,60)
	#print(servo_angle)
	body_move_1(2,5,8,11,30)
	#body_move(2,-30)
	#body_move(5,30)
	#body_move(8,-30)
	#body_move(11,30)
	time.sleep(0.2)
	print(servo_angle)
	leg_move(11,-60)
	leg_move(5,-60)
	body_move_1(2,5,8,11,30)
	#body_move(2,-30)
	#body_move(5,30)
	#body_move(8,-30)
	#body_move(11,30)
	time.sleep(0.2)
	leg_move(8,60)
	print(servo_angle)
	
for i in range(16):
  pwm.set_pwm(i, 0, 0)


