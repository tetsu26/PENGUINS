#!/usr/bin/python -u
#coding:utf-8

import commands

check = commands.getoutput("sudo sh /home/pi/PENGUINS/setdatefromgps.sh")

print check
