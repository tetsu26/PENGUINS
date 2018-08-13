#!/usr/bin/python
# -*- coding: utf-8 -*-

#################################################
#     CanSat PENGUIN のコードゾ！！
#				１号、２号ともにこのコードで動くゾ！
#				Python 2.7 で書いてネ！！
#################################################

import time
from picamera import PiCamera
import subprocess
import shlex
from datetime import datetime
from image_recog  import image_recog
