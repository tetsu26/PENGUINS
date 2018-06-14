
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
    x=input("which servo(0or1):")
    if x==0:
        while True:
            y=input("CH:0 pulse width(700-2300):")
        y=int(y)
        set_servo_pulse(0,y)
    else:
        while True:
            y=input("CH:1 pulse width(700-2300):")
        y=int(y)
        set_servo_pulse(1,y)
    
if x==2:
    i0=input("CH:0 initial pulse width(700-23000):")
    i0=int(i0)
    set_servo_pulse(0,i0)
    t0=input("CH:0 target pulse width(700-2300):")
    t0=int(t0)
    i1=input("CH:1 initial pulse width(700-23000):")
    i1=int(i1)
    set_servo_pulse(1,i1)
    t1=input("CH:1 target pulse width(700-2300):")
    t1=int(t1)
    print("CH:0 initial:",i0,"target:",t0)
    print("CH:1 initial:",i1,"target:",t1)
    z=input("If you are ready, press 'y':")
    if z==y:
        a=input("really?:")
	    if a==y:
            set_servo_pulse(0,t0)
            set_servo_pulse(1,t1)
