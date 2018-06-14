
from __future__ import division
import time

# Import the PCA9685 module.
import Adafruit_PCA9685

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()

# Helper function to make setting a servo pulse width simpler.
def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 50       # 60 Hz
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
    print('{0}us per bit'.format(pulse_length))
    #pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)
# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(50)

print('please choose modes: 1:Kakudo 2:EXAMINATION')
x=input()
if x==1:
    while True:
        y=input("pulse width(700-2300):")
        y=int(y)
        set_servo_pulse(0,y)
if x==2:
    y=input("initial pulse width(700-23000):")
    y=int(y)
    set_servo_pulse(0,y)
    b=input("target pulse width(700-2300):")
    b=int(b)
    print("initial:",y,"target:",b)
    z=input("If you are ready, press 'y':")
    if z==y:
        a=input("really?:")
	if a==y:
            set_servo_pulse(0,b)
