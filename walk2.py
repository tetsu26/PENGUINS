from __future__ import division
import time
import math
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

def angle_pulse(x):
   x = 170 * x / 18 + 500.0
   x=int(x)
   return x

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
	set_servo_pulse(servo_num-4,pulse_servo[servo_num-1])
	time.sleep(0.1)
	servo_angle[servo_num] += ang_ser
        angle_conv()
        #print(servo_angle[servo_num])
        set_servo_pulse(servo_num,pulse_servo[servo_num])
	time.sleep(0.1)
	servo_angle[servo_num-1] += 45
        angle_conv()
        #print(servo_angle[servo_num-4])
        set_servo_pulse(servo_num-4,pulse_servo[servo_num-1])
        time.sleep(0.1)

def body_move(servo_num,ang_ser):
	servo_angle[servo_num] += ang_ser
        angle_conv()
        #print(servo_angle[servo_num])
        set_servo_pulse(servo_num,pulse_servo[servo_num])
        #time.sleep(0.1)

angle_conv()
for i in range(12):
	set_servo_pulse(i,pulse_servo[i])
	time.sleep(0.1)
move_dist = input()
move_angle = math.asin((move_dist/63)+(1/math.sqrt(2.0)))/math.pi*180-45	
print(move_angle)
leg_move(2,-move_angle)
leg_move(5,move_angle)
for k in range(5):
	leg_move(8,2*move_angle)
	#print(servo_angle)
	body_move(2,-move_angle)
	body_move(11,move_angle)
	body_move(5,-move_angle)
	body_move(8,move_angle)
	time.sleep(0.1)
	print(servo_angle)
	leg_move(8,-2*move_angle)
	leg_move(11,-2*move_angle)
	body_move(2,-move_angle)
	body_move(11,move_angle)
	body_move(5,-move_angle)
	body_move(8,move_angle)
	time.sleep(0.1)
	leg_move(5,2*move_angle)
	print(servo_angle)
	
for i in range(16):
  pwm.set_pwm(i, 0, 0)


