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
import pygame
import numpy as np
from copy import deepcopy


class MecanumPro(threading.Thread):
    def __init__(self,ultra_com,mec_com):
        self.ser = Serial(mec_com,115200)
        self.sensor = Ultrasonic(ultra_com)
        self.mec = MecanumBase()
        self.stop_cmd = self.mec.stop()
        self.cmd = deepcopy(self.stop_cmd)
        self.dis = None
        self.dir = np.zeros((4,3))
        self.current_sts = ['stop',]

        threading.Thread.__init__(self)
        self.start()

    def run(self):
        self.dis = self.sensor.freshdata
        clk = pygame.time.Clock()
        ind = 0
        while True:
            self.ser.write(self.cmd)
            ind %= 20
            if not ind:
                tm = self.sensor.freshdata
                self.dir = np.sign(tm - self.dis)        #方向向量
                self.dis = tm

            if self.cmd == self.stop_cmd:   self.dir = np.zeros((4,3))
            ind+=1
            clk.tick(30)

    def check(self,cmd,threshold):

        if cmd[0]=='move':
            y = self.dir[[0,2],]
            x = self.dir[[1,3],]
            cm = np.sign(cmd[1])
            xx,yy = cm
            dx = xx*x
            dy = yy*y






    def __resolve(self,vec):
        vec = np.array(vec)
        mo = np.sqrt(np.sum(vec*vec))
        if mo==0:return mo,0
        ang = np.arcsin(vec[1]/mo)*180/np.pi
        if vec[0]>0:ang = ang-90
        else:   ang = 90-ang
        if ang>=0:   ang = ang
        else:   ang = 360+ang
        return mo,int(ang)

    def move(self,pos,vel=30,timeout=100): #mm
        '''
        pos:[x,y] x: -right y:-forward /mm
        '''

        self.current_sts = ['move',pos]
        dis,ang = self.__resolve(pos)
        self.cmd = self.mec.translateV(vel,ang)
        p = 1000*dis/vel
        pygame.time.delay(p)
        self.current_sts = ['stop',]
        self.cmd = self.stop_cmd
        pygame.time.wait(timeout)

    def rotate(self,ang,vel=2.5,timeout=100):
        self.current_sts = ['rotate',ang]
        self.cmd = self.mec.rotateV(np.sign(ang)*vel)
        p = 1000*ang/vel
        pygame.time.delay(p)
        self.current_sts = ['stop',]
        self.cmd = self.stop_cmd
        pygame.time.wait(timeout)



