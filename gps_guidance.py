#!/usr/bin/python -u
# -*- coding: utf-8 -*-

from __future__ import division

import os
import RPi.GPIO as GPIO
import smbus
import time
import math
import serial
import micropyGPS
import threading #gpsデータ取得用
import pyproj    #gps座標変換のパッケージ
import Adafruit_PCA9685
import commands  #gpsから内部時間を補正するために使用
import datetime

f=open('/home/pi/PENGUINS/PENGUIN_log.txt','a')
f.write("\n===============New Log From Here dazo!===============\n")

gps = micropyGPS.MicropyGPS(9,'dd') #gps
pwm = Adafruit_PCA9685.PCA9685()
Heating_wire_pin = 17 #電熱線で使うピン指定

servo_angle=[0,80,90,0,80,120,0,80,150,0,80,120]
pulse_servo=[0,0,0,0,0,0,0,0,0,0,0,0]

GPIO.setmode(GPIO.BCM)
GPIO.setup(Heating_wire_pin,GPIO.OUT)

#####照度センサー用#####
DEBUG = 1
SPICLK = 11
SPIMISO = 9
SPIMOSI = 10
SPICS = 8

# set up the SPI interface pins
GPIO.setup(SPIMOSI, GPIO.OUT)
GPIO.setup(SPIMISO, GPIO.IN)
GPIO.setup(SPICLK, GPIO.OUT)
GPIO.setup(SPICS, GPIO.OUT)

photoTch = 0        # Channel of A/D 

last_read = 0       # thsis keeps track of the last potentiometer value
tolerance = 5       # to keep from being jittery we'll only change
                    # volume when the pot has moved more than 5 'counts'



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

def angle_conv():
    for k in range(12):
                pulse_servo[k]=angle_pulse(servo_angle[k])
    return pulse_servo

def kaikyaku():
    global servo_angle,pulse_servo
    servo_angle=[0,90,120,0,90,120,0,90,120,0,90,120]
    pulse_servo=[0,0,0,0,0,0,0,0,0,0,0,0]
    servo_angle[0] = 90
    servo_angle[1] = 90
    angle_conv()
    for i in range(12):
        set_servo_pulse(i,pulse_servo[i])
        time.sleep(0.1)
        print servo_angle
    servo_angle = [10,10,10,10,90,90,90,90,120,120,120,120]
#    print servo_angle
#    time.sleep(3.0)
    angle_conv()
    for i in range(12):
        set_servo_pulse(i,pulse_servo[i])
        time.sleep(0.1)
#    for i in range(16):
#        pwm.set_pwm(i, 0, 0)

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

def body_move(servo_num,ang_ser):
    servo_angle[servo_num] += ang_ser
    angle_conv()
    #print(servo_angle[servo_num])
    set_servo_pulse(servo_num,pulse_servo[servo_num])
    #time.sleep(0.1)

def walk():
    global servo_angle,pulse_servo
    servo_angle=[10,90,120,10,90,120,10,90,120,10,90,120]
    pulse_servo=[0,0,0,0,0,0,0,0,0,0,0,0]
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

def turn_left():
    ang_turn = 45
    #servo_angle = [10,80,90,10,80,120,10,80,150,10,80,120]
    #pulse_servo = [0,0,0,0,0,0,0,0,0,0,0,0]
    ang_turn = ang_turn / 4.0
    for k in range(4):
        print(ang_turn)
        body_move(2,-ang_turn)
        body_move(5,-ang_turn)
        body_move(8,-ang_turn)
        body_move(11,-ang_turn)
        leg_move(8,ang_turn)
        leg_move(2,60.0+ang_turn)
        body_move(2,-30.0)
        body_move(5,-30.0)
        body_move(8,-30.0)
        body_move(11,-30.0)
        leg_move(5,60.0+ang_turn)
        leg_move(11,ang_turn)
        leg_move(5,-60.0)
        body_move(2,30.0)
        body_move(5,30.0)
        body_move(8,30.0)
        body_move(11,30.0)
        leg_move(2,-60.0)

def turn_right():
    ang_turn = 45
    #servo_angle = [10,80,90,10,80,120,10,80,150,10,80,120]
    #pulse_servo = [0,0,0,0,0,0,0,0,0,0,0,0]
    for k in range(4):
        print(ang_turn)
        body_move(2,ang_turn)
        body_move(5,ang_turn)
        body_move(8,ang_turn)
        body_move(11,ang_turn)
        leg_move(2,-ang_turn)
        leg_move(8,-60.0-ang_turn)
        body_move(2,30.0)
        body_move(5,30.0)
        body_move(8,30.0)
        body_move(11,30.0)
        leg_move(5,-ang_turn)
        leg_move(11,-ang_turn)
        body_move(2,-30.0)
        body_move(5,-30.0)
        body_move(8,-30.0)
        body_move(11,-30.0)
        leg_move(8,60.0)
        print(servo_angle)

#  AD変換奴
# read SPI data from MCP3008 chip, 8 possible adc's (0 thru 7)
def readadc(adcnum, clockpin, mosipin, misopin, cspin):
    if ((adcnum > 7) or (adcnum < 0)):
        return -1
    GPIO.output(cspin, True)

    GPIO.output(clockpin, False)  # start clock low
    GPIO.output(cspin, False)     # bring CS low

    commandout = adcnum
    commandout |= 0x18  # start bit + single-ended bit
    commandout <<= 3    # we only need to send 5 bits here
    for i in range(5):
        if (commandout & 0x80):
            GPIO.output(mosipin, True)
        else:
            GPIO.output(mosipin, False)
        commandout <<= 1
        GPIO.output(clockpin, True)
        GPIO.output(clockpin, False)

    adcout = 0
        # read in one empty bit, one null bit and 10 ADC bits
    for i in range(12):
        GPIO.output(clockpin, True)
        GPIO.output(clockpin, False)
        adcout <<= 1
        if (GPIO.input(misopin)):
            adcout |= 0x1

    GPIO.output(cspin, True)
        
    adcout >>= 1       # first bit is 'null' so drop it
    return adcout     

class SL_MPU9250:
    # 定数宣言
    REG_PWR_MGMT_1      = 0x6B
    REG_INT_PIN_CFG     = 0x37
    REG_GYRO_CONFIG     = 0x1B
    REG_ACCEL_CONFIG1   = 0x1C
    REG_ACCEL_CONFIG2   = 0x1D

    MAG_MODE_POWERDOWN  = 0         # 磁気センサpower down
    MAG_MODE_SERIAL_1   = 1         # 磁気センサ8Hz連続測定モード
    MAG_MODE_SERIAL_2   = 2         # 磁気センサ100Hz連続測定モード
    MAG_MODE_SINGLE     = 3         # 磁気センサ単発測定モード
    MAG_MODE_EX_TRIGER  = 4         # 磁気センサ外部トリガ測定モード
    MAG_MODE_SELF_TEST  = 5         # 磁気センサセルフテストモード

    MAG_ACCESS          = False     # 磁気センサへのアクセス可否
    MAG_MODE            = 0         # 磁気センサモード
    MAG_BIT             = 14        # 磁気センサが出力するbit数

    offsetRoomTemp      = 0
    tempSensitivity     = 333.87
    gyroRange           = 250       # dps 00:250, 01:500, 10:1000, 11:2000
    accelRange          = 2         # g 00:±2, 01:±4, 10:±8, 11:±16
    magRange            = 4912      # μT

    offsetAccelX        = 0.0
    offsetAccelY        = 0.0
    offsetAccelZ        = 0.0
    offsetGyroX         = 0.0
    offsetGyroY         = 0.0
    offsetGyroZ         = 0.0
    manual_offsetMagX   =-12.0
    manual_offsetMagY   =-4.0
    manual_offsetMagX   =0.0

    # コンストラクタ
    def __init__(self, address, channel):
        self.address    = address
        self.channel    = channel
        self.bus        = smbus.SMBus(self.channel)
        self.addrAK8963 = 0x0C

        # Sensor initialization
        self.resetRegister()
        self.powerWakeUp()

        self.gyroCoefficient    = self.gyroRange  / float(0x8000)   # センシングされたDecimal値をdpsに変換する係数
        self.accelCoefficient   = self.accelRange / float(0x8000)   # センシングされたDecimal値をgに変換する係数
        self.magCoefficient16   = self.magRange   / 32760.0         # センシングされたDecimal値をμTに変換する係数(16bit時)
        self.magCoefficient14   = self.magRange   / 8190.0          # センシングされたDecimal値をμTに変換する係数(14bit時)


    # レジスタを初期設定に戻します。
    def resetRegister(self):
        if self.MAG_ACCESS == True:
            self.bus.write_i2c_block_data(self.addrAK8963, 0x0B, [0x01])    
        self.bus.write_i2c_block_data(self.address, 0x6B, [0x80])
        self.MAG_ACCESS = False
        time.sleep(0.1)     

    # レジスタをセンシング可能な状態にします。
    def powerWakeUp(self):
        # PWR_MGMT_1をクリア
        self.bus.write_i2c_block_data(self.address, self.REG_PWR_MGMT_1, [0x00])
        time.sleep(0.1)
        # I2Cで磁気センサ機能(AK8963)へアクセスできるようにする(BYPASS_EN=1)
        self.bus.write_i2c_block_data(self.address, self.REG_INT_PIN_CFG, [0x02])
        self.MAG_ACCESS = True
        time.sleep(0.1)

    # 磁気センサのレジスタを設定する
    def setMagRegister(self, _mode, _bit):      
        if self.MAG_ACCESS == False:
            # 磁気センサへのアクセスが有効になっていないので例外を上げる
            raise Exception('001 Access to a sensor is invalid.')

        _writeData  = 0x00
        # 測定モードの設定
        if _mode=='8Hz':            # 連続測定モード１
            _writeData      = 0x02
            self.MAG_MODE   = self.MAG_MODE_SERIAL_1
        elif _mode=='100Hz':        # 連続測定モード２
            _writeData      = 0x06
            self.MAG_MODE   = self.MAG_MODE_SERIAL_2
        elif _mode=='POWER_DOWN':   # パワーダウンモード
            _writeData      = 0x00
            self.MAG_MODE   = self.MAG_MODE_POWERDOWN
        elif _mode=='EX_TRIGER':    # 外部トリガ測定モード
            _writeData      = 0x04
            self.MAG_MODE   = self.MAG_MODE_EX_TRIGER
        elif _mode=='SELF_TEST':    # セルフテストモード
            _writeData      = 0x08
            self.MAG_MODE   = self.MAG_MODE_SELF_TEST
        else:   # _mode='SINGLE'    # 単発測定モード
            _writeData      = 0x01
            self.MAG_MODE   = self.MAG_MODE_SINGLE

        #出力するbit数 
        if _bit=='14bit':           # 14bit出力
            _writeData      = _writeData | 0x00
            self.MAG_BIT    = 14
        else:   # _bit='16bit'      # 16bit 出力
            _writeData      = _writeData | 0x10
            self.MAG_BIT    = 16

        self.bus.write_i2c_block_data(self.addrAK8963, 0x0A, [_writeData])

    # 加速度の測定レンジを設定します。広レンジでは測定粒度が荒くなります。
    # val = 16, 8, 4, 2(default)
    def setAccelRange(self, val, _calibration=False):
        # ±2g (00), ±4g (01), ±8g (10), ±16g (11)
        if val==16 :
            self.accelRange     = 16
            _data               = 0x18
        elif val==8 :
            self.accelRange     = 8
            _data               = 0x10
        elif val==4 :
            self.accelRange     = 4
            _data               = 0x08
        else:
            self.accelRange     = 2
            _data               = 0x00

        self.bus.write_i2c_block_data(self.address, self.REG_ACCEL_CONFIG1, [_data])
        self.accelCoefficient   = self.accelRange / float(0x8000)
        time.sleep(0.1)

        # オフセット値をリセット(過去のオフセット値が引き継がれないように)
        self.offsetAccelX       = 0
        self.offsetAccelY       = 0
        self.offsetAccelZ       = 0

        # 本当はCalibrationしたほうが良いと思うけれど、時間もかかるし。
        if _calibration == True:
            self.calibAccel(1000)
        return

    # ジャイロの測定レンジを設定します。広レンジでは測定粒度が荒くなります。
    # val= 2000, 1000, 500, 250(default)
    def setGyroRange(self, val, _calibration=False):
        if val==2000:
            self.gyroRange      = 2000
            _data               = 0x18
        elif val==1000:
            self.gyroRange      = 1000
            _data               = 0x10
        elif val==500:
            self.gyroRange      = 500
            _data               = 0x08
        else:
            self.gyroRange      = 250
            _data               = 0x00

        self.bus.write_i2c_block_data(self.address, self.REG_GYRO_CONFIG, [_data])
        self.gyroCoefficient    = self.gyroRange / float(0x8000)
        time.sleep(0.1)

        # オフセット値をリセット(過去のオフセット値が引き継がれないように)
        self.offsetGyroX        = 0
        self.offsetGyroY        = 0
        self.offsetGyroZ        = 0

        # 本当はCalibrationしたほうが良いのだが、時間もかかるし。
        if _calibration == True:
            self.calibGyro(1000)
        return

    # 加速度センサのLowPassFilterを設定します。
    # def setAccelLowPassFilter(self,val):      

    #センサからのデータはそのまま使おうとするとunsignedとして扱われるため、signedに変換(16ビット限定）
    def u2s(self,unsigneddata):
        if unsigneddata & (0x01 << 15) : 
            return -1 * ((unsigneddata ^ 0xffff) + 1)
        return unsigneddata

    # 加速度値を取得します
    def getAccel(self):
        data    = self.bus.read_i2c_block_data(self.address, 0x3B ,6)
        rawX    = self.accelCoefficient * self.u2s(data[0] << 8 | data[1]) + self.offsetAccelX
        rawY    = self.accelCoefficient * self.u2s(data[2] << 8 | data[3]) + self.offsetAccelY
        rawZ    = -(self.accelCoefficient * self.u2s(data[4] << 8 | data[5]) + self.offsetAccelZ)
        return rawX, rawY, rawZ

    # ジャイロ値を取得します。
    def getGyro(self):
        data    = self.bus.read_i2c_block_data(self.address, 0x43 ,6)
        rawX    = self.gyroCoefficient * self.u2s(data[0] << 8 | data[1]) + self.offsetGyroX
        rawY    = self.gyroCoefficient * self.u2s(data[2] << 8 | data[3]) + self.offsetGyroY
        rawZ    = self.gyroCoefficient * self.u2s(data[4] << 8 | data[5]) + self.offsetGyroZ
        return rawX, rawY, rawZ

    def getMag(self):
        if self.MAG_ACCESS == False:
            # 磁気センサが有効ではない。
            raise Exception('002 Access to a sensor is invalid.')

        # 事前処理
        if self.MAG_MODE==self.MAG_MODE_SINGLE:
            # 単発測定モードは測定終了と同時にPower Downになるので、もう一度モードを変更する
            if self.MAG_BIT==14:                # 14bit出力
                _writeData      = 0x01
            else:                               # 16bit 出力
                _writeData      = 0x11
            self.bus.write_i2c_block_data(self.addrAK8963, 0x0A, [_writeData])
            time.sleep(0.01)

        elif self.MAG_MODE==self.MAG_MODE_SERIAL_1 or self.MAG_MODE==self.MAG_MODE_SERIAL_2:
            status  = self.bus.read_i2c_block_data(self.addrAK8963, 0x02 ,1)
            if (status[0] & 0x02) == 0x02:
                # データオーバーランがあるので再度センシング
                self.bus.read_i2c_block_data(self.addrAK8963, 0x09 ,1)

        elif self.MAG_MODE==self.MAG_MODE_EX_TRIGER:
            # 未実装
            return

        elif self.MAG_MODE==self.MAG_MODE_POWERDOWN:
            raise Exception('003 Mag sensor power down')

        # ST1レジスタを確認してデータ読み出しが可能か確認する。
        status  = self.bus.read_i2c_block_data(self.addrAK8963, 0x02 ,1)
        while (status[0] & 0x01) != 0x01:
            # データレディ状態まで待つ
            time.sleep(0.01)
            status  = self.bus.read_i2c_block_data(self.addrAK8963, 0x02 ,1)

        # データ読み出し
        data    = self.bus.read_i2c_block_data(self.addrAK8963, 0x03 ,7)
        rawX    = self.u2s(data[1] << 8 | data[0])  # 下位bitが先
        rawY    = self.u2s(data[3] << 8 | data[2])  # 下位bitが先
        rawZ    = self.u2s(data[5] << 8 | data[4])  # 下位bitが先
        st2     = data[6]

        # オーバーフローチェック
        if (st2 & 0x08) == 0x08:
            # オーバーフローのため正しい値が得られていない
            raise Exception('004 Mag sensor over flow')

##############################ここは手動で入力#######################################
        manual_offsetMagX  = -24
        manual_offsetMagY  = 7.5
        manual_offsetMagZ  = 0.0

        # μTへの変換
        if self.MAG_BIT==16:    # 16bit出力の時
            rawX    = rawX * self.magCoefficient16 + manual_offsetMagX
            rawY    = rawY * self.magCoefficient16 + manual_offsetMagY
            rawZ    = rawZ * self.magCoefficient16 + manual_offsetMagZ
        else:                   # 14bit出力の時
            rawX    = raw*0.9+(rawX * self.magCoefficient14 + manual_offsetMagX)*0.1
            rawY    = raw*0.9+(rawY * self.magCoefficient14 + manual_offsetMagY)*0.1
            rawZ    = raw*0.9+(rawZ * self.magCoefficient14 + manual_offsetMagZ)*0.1

        return rawX, rawY, rawZ

    def getTemp(self):
        data    = self.bus.read_i2c_block_data(self.address, 0x65 ,2)
        raw     = data[0] << 8 | data[1]
        return ((raw - self.offsetRoomTemp) / self.tempSensitivity) + 21

    def selfTestMag(self):
        print "start mag sensor self test"
        self.setMagRegister('SELF_TEST','16bit')
        self.bus.write_i2c_block_data(self.addrAK8963, 0x0C, [0x40])
        time.sleep(1.0)
        data = self.getMag()

        print data

        self.bus.write_i2c_block_data(self.addrAK8963, 0x0C, [0x00])
        self.setMagRegister('POWER_DOWN','16bit')
        time.sleep(1.0)
        print "end mag sensor self test"
        return

    # 加速度センサを較正する
    # 本当は緯度、高度、地形なども考慮する必要があるとは思うが、簡略で。
    # z軸方向に正しく重力がかかっており、重力以外の加速度が発生していない前提
    def calibAccel(self, _count=1000):
        print "Accel calibration start"
        _sum    = [0,0,0]

        # 実データのサンプルを取る
        for _i in range(_count):
            _data   = self.getAccel()
            _sum[0] += _data[0]
            _sum[1] += _data[1]
            _sum[2] += _data[2]

        # 平均値をオフセットにする
        self.offsetAccelX   = -1.0 * _sum[0] / _count
        self.offsetAccelY   = -1.0 * _sum[1] / _count
        self.offsetAccelZ   = -1.0 * ((_sum[2] / _count ) - 1.0)    # 重力分を差し引く

        # オフセット値をレジスタに登録したいけれど、動作がわからないので実装保留

        print "Accel calibration complete"
        return self.offsetAccelX, self.offsetAccelY, self.offsetAccelZ

    # ジャイロセンサを較正する
    # 各軸に回転が発生していない前提
    def calibGyro(self, _count=1000):
        print "Gyro calibration start"
        _sum    = [0,0,0]

        # 実データのサンプルを取る
        for _i in range(_count):
            _data   = self.getGyro()
            _sum[0] += _data[0]
            _sum[1] += _data[1]
            _sum[2] += _data[2]

        # 平均値をオフセットにする
        self.offsetGyroX    = -1.0 * _sum[0] / _count
        self.offsetGyroY    = -1.0 * _sum[1] / _count
        self.offsetGyroZ    = -1.0 * _sum[2] / _count

        # オフセット値をレジスタに登録したいけれど、動作がわからないので実装保留

        print "Gyro calibration complete"
        return self.offsetGyroX, self.offsetGyroY, self.offsetGyroZ

###################################################################################

def get_time(): #gpsから内部時計補正
    check = commands.getoutput("sudo sh /home/pi/PENGUINS/setdatefromgps.sh")
    print check

def opening_detect(): #解放検知
    while True:
        trim_pot = readadc(photoTch, SPICLK, SPIMOSI, SPIMISO, SPICS)
#        set_volume = trim_pot / (10.24 * 4) 
#        set_volume = round(set_volume)
#        set_volume = int(set_volume)
#        print(set_volume)
        print(trim_pot)
#        get_data()
        time.sleep(0.2)

        if trim_pot < 500: #明るさのしきい値を100とする
            print('!!!!!!!!!!!!!!!!!!Break!!!!!!!!!!!!!!!!!!!')
            f.write ("Openig detect\n")
            #break
        else:
            print("In the Carrier dazo~")
            f.write ("Carrier now\n")

def rungps(): #gpsやつ
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
    #print('%2d:%02d:%04.1f' % (h, gps.timestamp[1], gps.timestamp[2]))
    print('緯度経度: %2.8f, %2.8f' % (gps.latitude[0], gps.longitude[0]))
    #print('海抜: %f' % gps.altitude)

    #座標変換
    data.Penguin1_pos  = [round(gps.latitude[0],9),round(gps.longitude[0],9)]#小数点8桁以上だとエラー出るので7桁まで

    EPSG4612 = pyproj.Proj("+init=EPSG:4612")
    EPSG2452 = pyproj.Proj("+init=EPSG:2452") #東北地方中心平面直角座標系10経


    if(data.Penguin1_pos[1] != 0.0):
        data.LNS_xy[1],data.LNS_xy[0] = pyproj.transform(EPSG4612, EPSG2452, data.LNS[1], data.LNS[0] )#x,yが逆なので注意（ここで2時間溶かした）
        data.Penguin1_pos_xy[1],data.Penguin1_pos_xy[0] = pyproj.transform(EPSG4612, EPSG2452, data.Penguin1_pos[1], data.Penguin1_pos[0] )
    else:
        print ("We are at Gulf of Guinea")

    dx=data.LNS_xy[0]-data.Penguin1_pos_xy[0]#LNSまでのx座標の差
    dy=data.LNS_xy[1]-data.Penguin1_pos_xy[1]#LNSまでのy座標の差
    data.distance=math.sqrt(dx**2+dy**2)#LNSまでの距離
#    global orientation_deg
    #LNSへの角度
    if(dy>0):
        data.orientation_deg=math.degrees(math.acos(dx/data.distance))
    else:
        data.orientation_deg=-math.degrees(math.acos(dx/data.distance))

    #print Penguin1_pos_xy
    #print LNS_xy
    print ("distance:%f"%data.distance)
    print ("orientation_deg:%f"%data.orientation_deg)
    #print('')
###################################################################################

class Data:

    #変数宣言    
    date              =  datetime.datetime.now()    #日付時間
    now               =  0.0                        #たぶん時間
    acc               =  [0.0,0.0,0.0]              #加速度
    gyr               =  [0.0,0.0,0.0]              #ジャイロ
    mag               =  [0.0,0.0,0.0]              #磁気

    current_deg       =  0.0                        #現在機体が向いてい方向(以後角度は東が0度でプラスマイナス180度)
    distance          =  0.0                        #機体からLNSまでの距離
    orientation_deg   =  0.0                        #機体からLNSへの角度
    current_deg       =  0.0                        #機体が向いている角度
    ddeg              =  0.0                        #LNSへの角度と機体が向いている角度の差
    range             =  30                         #目標角への許容誤差(プラスマイナスrange度)

    #座標
    LNS=[38.26619000, 140.84301667]                 #ローソン南極支店の座標(天野邸)
    LNS_xy=[38.28005833,140.85160667]               #ローソン南極支店のxy座標
    Penguin1_pos=[0.0,0.0]                          #ペンギン1号の座標
    Penguin1_pos_xy=[0.0,0.0]                       #ペンギン1号のxy座標


    gps_flag          =  0                          #gps誘導で近くまで来たか
    date_flag         =  0                          #日付時間を同期したか確認するためのフラグ

    turnover_counter  =  0                          #転んだ状態かを確認するカウンター

    #コンストラクタ
#    def __init__(self):
    #ログ
#       self.f.write("\n===============New Log From Here dazo!===============\n")


def save_data():
    data.date=datetime.datetime.now()
    f.write(str(data.date)+",")
    f.write(str(data.acc[0])+","+str(data.acc[1])+","+str(data.acc[2])+","+ str(data.gyr[0])+"," + str(data.gyr[1]) + "," + str(data.gyr[2]) +","+ str(data.mag[0])+","+ str(data.mag[1])+","+ str(data.mag[2])+",")
    f.write(str(data.LNS[0]) + "," + str(data.LNS[1]) + "," + str(data.LNS_xy[0]) + "," + str(data.LNS_xy[1]) + "," + str(data.Penguin1_pos[0])+","+str(data.Penguin1_pos[1])+","+ str(data.Penguin1_pos_xy[0])+","+str(data.Penguin1_pos_xy[1])+",")
    f.write(str(data.current_deg) + "," + str(data.distance) + "," + str(data.orientation_deg)+","+str(data.current_deg)+","+str(data.ddeg) + ",")

def detouch_para():
    for i in range(3):
        print "para detouch in " + str(3-i)
        time.sleep(1)
    print "HEATING NOW!"
    GPIO.output(Heating_wire_pin,1)
    time.sleep(1.0)
    GPIO.output(Heating_wire_pin,0)    
    print "para detouched dazo~"
    f.write("para detouched\n")
    time.sleep(1.0)

def get_data():
    #各データ取得
    data.now     = time.time()
    data.acc     = sensor.getAccel()
    data.gyr     = sensor.getGyro()
    data.mag     = sensor.getMag()
    getgps()       #gpsデータ取得関数

    #機体の向いている方向を磁気センサーの値から算出
    if(data.mag[1]<0):
        data.current_deg     = math.degrees(math.acos(data.mag[0]/math.sqrt(data.mag[0]**2+data.mag[1]**2))) - mag_correction1 - mag_correction2
    else:
        data.current_deg     = -math.degrees(math.acos(data.mag[0]/math.sqrt(data.mag[0]**2+data.mag[1]**2))) - mag_correction1 - mag_correction2

    #機体の向いている角度をプラスマイナス180に変換
    if (data.current_deg>180):
        data.current_deg = data.current_deg -360
    elif(data.current_deg<-180):
        data.current_deg = data.current_deg + 360

    data.ddeg=data.orientation_deg-data.current_deg #現在向いている方向とLNSへの方向の差を計算

    #角度の差をプラスマイナス180度に変換
    if (data.ddeg>180):
        data.ddeg = data.ddeg -360
    elif(data.ddeg<-180):
       data.ddeg = data.ddeg + 360

    #各データを表示
    print "%+8.7f" % data.acc[0] + " " + "%+8.7f" % data.acc[1] + " " + "%+8.7f" % data.acc[2]
#   print " |   ",
    print "%+8.7f" % data.gyr[0] + " " + "%+8.7f" % data.gyr[1] + " " + "%+8.7f" % data.gyr[2]
#   print " |   ",
    print "%+8.7f" % data.mag[0] + " " + "%+8.7f" % data.mag[1] + " " + "%+8.7f" % data.mag[2]

    print ("current_deg:%f"%data.current_deg)
    print ("ddeg:%f"%data.ddeg)

    save_data() #取得したデータを保存

#進む方向を決定
def orientation():
    if (data.turnover_counter!=0):
        sleep(1)
    elif (data.ddeg<-data.range):
        print ("Turn Rght dazo~\n")
        f.write ("Turn Right\n")
#        turn_right()
    elif (data.ddeg>data.range):
        print ("Turn Left dazo~\n")
        f.write ("Turn Left\n")
#        turn_left()
    else :
        print ("Go Straight dazo~\n")
        f.write ("Go Straight\n")
#        walk()

    #LNSに5mまで近づいた時
    if (data.distance<3):
        print ("LNS is close dane~~~")
        f.write ("LNS is close\n")
        data.gps_flag=1

#ひっくり返ってないかチェック
def turn_over_check():
    if (data.acc[2]<0.4): #Z軸の加速度で評価
        print ("koronda?")
        f.write ("turn over?\n")
        data.turnover_counter+=1
    else :
        data.turnover_counter=0

    if (data.turnover_counter>4):
        print ("korondazo~~~~~")
        f.write ("turn over\n")
#        kaikyaku()
        

if __name__ == "__main__":
    #MPU9250
    sensor  = SL_MPU9250(0x68,1)
    sensor.resetRegister()
    sensor.powerWakeUp()
    sensor.setAccelRange(8,True)
    sensor.setGyroRange(1000,True)
    sensor.setMagRegister('100Hz','16bit')
    # sensor.selfTestMag()
    #get_time()
    mag_correction1 = 8.1 #磁北を真北に補正する奴仙台では8.1　能代では8.9 
    mag_correction2 = 45  #センサーの前方向とペンギンの前方向を一致させる補正

    #データ用class
    data = Data()


    opening_detect()
#    kaikyaku()
#    detouch_para()


    while data.gps_flag==0:

        get_data()
        turn_over_check()
        orientation()
        time.sleep(0.3)


