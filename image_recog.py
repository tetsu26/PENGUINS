#!/usr/bin/python
# -*- coding: utf-8 -*-

# 画像認識してLNSに貼ってあるマーカーをよみとるゾ！　read_aruco と out_camera_data.xml が必要だゾ！
# 読み取った結果はread_result.csvに書き込まれるゾ！！

import time
from picamera import PiCamera
import subprocess
import shlex
from datetime import datetime

def image_recog
#カメラのセットアップする
	camera = PiCamera()
	camera.resolution = (3280,2464)
	time.sleep(2)

#まず写真を撮る ファイル名は時刻を入れたものにし、imagesディレクトリに保存する。
	timecode = datetime.now().strftime("%H%M%S")
	ImageName = "images/marker" + timecode  + ".jpg"
	my_file = open(ImageName, 'wb')
	camera.capture(my_file)
	my_file.close()

#C++コードに投げる
	cmd = "./read_aruco " + ImageName + " " +timecode
	cmd = shlex.split(cmd)
	subprocess.call(cmd)
