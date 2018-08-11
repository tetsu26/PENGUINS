#!/usr/bin/python -u
# -*- coding: utf-8 -*-

import serial
import micropyGPS
import threading
import time
import pyproj
import math

LNS=[38.267922, 140.849301] #ローソン南極支店の座標(天野邸)
LNS_xy=[38.28005833,140.85160667] #ローソン南極支店のxy座標
Penguin1_pos=[0.0,0.0] #ペンギン1号の座標
Penguin1_pos_xy=[0.0,0.0] #ペンギン1号のxy座標

gps = micropyGPS.MicropyGPS(9,'dd')

def rungps():
    s = serial.Serial('/dev/serial0', 9600, timeout=10)
    s.readline() # 最初の1行は中途半端なデーターが読めることがあるので、捨てる
    while True:
        sentence = s.readline().decode('utf-8') # GPSデーターを読み、文字列に変換する
        if sentence[0] != '$': # 先頭が'$'でなければ捨てる
            continue
        for x in sentence: # 読んだ文字列を解析してGPSオブジェクトにデーターを追加、更新する
            gps.update(x)

gpsthread = threading.Thread(target=rungps, args=()) # 上の関数を実行するスレッドを生成
gpsthread.daemon = True
gpsthread.start() # スレッドを起動

def getgps():
    while gps.clean_sentences < 20:# ちゃんとしたデーターがある程度たまったら出力する
        time.sleep(0.1)

    h = gps.timestamp[0] if gps.timestamp[0] < 24 else gps.timestamp[0] - 24
    print('%2d:%02d:%04.1f' % (h, gps.timestamp[1], gps.timestamp[2]))
    print('緯度経度: %2.8f, %2.8f' % (gps.latitude[0], gps.longitude[0]))
    print('海抜: %f' % gps.altitude)

    #座標変換
    Penguin1_pos  = [round(gps.latitude[0],8),round(gps.longitude[0],8)] #小数点8桁以上だとエラー出るので7桁まで

    EPSG4612 = pyproj.Proj("+init=EPSG:4612")
    EPSG2452 = pyproj.Proj("+init=EPSG:2452") #東北地方中心平面直角座標系10経

    LNS_xy[1],LNS_xy[0] = pyproj.transform(EPSG4612, EPSG2452, LNS[1], LNS[0] ) #x,yが逆なので注意（ここで2時間溶かした）
    Penguin1_pos_xy[1],Penguin1_pos_xy[0] = pyproj.transform(EPSG4612, EPSG2452, Penguin1_pos[1], Penguin1_pos[0] )       
    dx=LNS_xy[0]-Penguin1_pos_xy[0] #LNSまでのx座標の差
    dy=LNS_xy[1]-Penguin1_pos_xy[1] #LNSまでのy座標の差
    distance=math.sqrt(dx**2+dy**2) #LNSまでの距離
    #LNSへの角度
    if(dy>0):
        orientation_deg=math.degrees(math.acos(dx/distance))
    else:
        orientation_deg=-math.degrees(math.acos(dx/distance))
                
    print Penguin1_pos_xy
    print LNS_xy
    print distance
    print orientation_deg
    print('')
 
if __name__ == "__main__":

    getgps()
    print ("gotgps")
