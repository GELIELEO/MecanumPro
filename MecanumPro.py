#!user/bin/python
# -*-coding:utf-8-*-

#FileName: MecanumPro.py
#Version: 1.0
#Author: Jingsheng Tang
#Date: 2017/8/25
#Email: mrtang@nudt.edu.cn
#Github: trzp

from __future__ import division
from MecanumBase import MecanumBase
from ultrasonic import Ultrasonic
from serial import Serial
import threading
from threading import Event
import numpy as np
import pygame
from copy import deepcopy

TVEL = 30
RVEL = 3
FRAMERATE = 20
STP = 1/FRAMERATE
SAFEDIS = 100

class DrivingInfo:
    destination = np.zeros(2)
    distance = 0
    vel = TVEL
    mode = 'translate'

class MecanumPro(threading.Thread):
    def __init__(self, ultra_com, mec_com):
        '''
        :param ultra_com: serial port for ultrasonic sensors
        :param mec_com: serial port for mecanum platform
        :return:
        '''
        self.ser = Serial(mec_com, 115200)
        self.sensor = Ultrasonic(ultra_com)
        self.env_data = self.sensor.freshdata
        self.mec = MecanumBase()
        self.__ev = Event()
        self.run_count = 0
        self.end_count = 0
        self.Info = DrivingInfo()
        self.res = None
        self.stop_cmd = self.mec.stop()
        self.cmd = deepcopy(self.stop_cmd)
        self.op = np.array([[1,1,1],[-1,-1,-1],[-1,-1,-1],[1,1,1]])
        self.co = np.array([[.1,.1,.1],[-.1,-.1,-.1],[-.1,-.1,-.1],[.1,.1,.1]])
        threading.Thread.__init__(self)
        self.start()

    def run(self):
        clk = pygame.time.tick_busy_loop()
        while True:
            self.env_data = self.sensor.freshdata

            if self.end_count == 0:     #stop
                self.run_count = 0
                self.ser.write(self.stop_cmd)
            else:                       #running
                self.res = self.check(self.Info,self.env_data,SAFEDIS)
                if self.res.size != 0:
                    self.end_count = 0
                    self.__ev.set()
                else:
                    self.ser.write(self.cmd)
                    if self.run_count > self.end_count:
                        self.end_count = 0
                        self.__ev.set()
                    self.run_count += 1

            clk.tick(FRAMERATE)

    def check(self, drivinginfo, envdata,safedis):
        envdata = envdata * self.op
        if drivinginfo.mode == 'translate':
            xstp = STP * drivinginfo.destination[0] * drivinginfo.vel / drivinginfo.distance
            ystp = STP * drivinginfo.destination[1] * drivinginfo.vel / drivinginfo.distance
            envdata[[1,3],] += (np.ones((2,3)) * xstp)
            envdata[[0,2],] += (np.ones((2,3)) * ystp)
            return np.where(abs(envdata) < safedis)[0]
        elif drivinginfo.mode == 'rotate':
            envdata += np.sign(drivinginfo.destination) * drivinginfo.vel * self.co * STP
            return np.where(abs(envdata) < safedis)[0]
        else:
            return 0,

    def __resolve(self, vec):
        '''
        :param vec: destination position
        :return: actual length of the vec, angle for driving mecanum
        '''
        vec = np.array(vec)
        mo = np.sqrt(np.sum(vec*vec))
        if mo == 0: return mo, 0
        ang = np.arcsin(vec[1]/mo)*180/np.pi

        if vec[0] > 0:      ang -= 90
        else:               ang = 90 - ang
        if ang >= 0:        ang = ang
        else:               ang += 360
        return mo, int(ang)

    def move(self,pos,vel=TVEL):  #mm
        vel = abs(vel)
        dis,ang = self.__resolve(pos)
        self.Info.destination = pos
        self.Info.mode = 'translate'
        self.Info.distance = dis
        self.Info.vel = vel
        self.cmd = self.mec.translateV(vel,ang)
        self.end_count = int(dis/(vel*STP))
        self.__ev.wait()
        self.__ev.clear()
        return self.res.size,self.res

    def rotate(self,angle,vel=RVEL):  #mm
        vel = abs(vel)
        self.Info.destination = angle
        self.Info.mode = 'rotate'
        self.Info.distance = abs(angle)
        self.Info.vel = vel
        self.cmd = self.mec.rotateV(vel * np.sign(angle))
        self.end_count = int(abs(angle)/(vel*STP))
        self.__ev.wait()
        self.__ev.clear()
        return self.res.size,self.res
