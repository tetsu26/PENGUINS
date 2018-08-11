#!/usr/bin/python -u
#coding:utf-8

import commands

check = commands.getoutput("./setdatefromgps.sh")

print check
