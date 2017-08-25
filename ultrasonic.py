#coding:utf-8
from serial import Serial
from struct import unpack,pack
import numpy as np


class Ultrasonic(object):
    """
    author:mrtang
    date:2017.5
    version:1.0
    email:mrtang@nudt.edu.cn

    sensors:
                         forward
                    1       2       3
                1                       1
        left    2                       2   right
                3                       3
                    1       2       3
                           back
    """
    def __init__(self,com):
        self.ser = Serial(com,9600)
        self.ser.setTimeout(0.1)
        self.r = np.zeros([4,3])
        self.rdata = np.zeros([3,4,3])
        self.ST = np.array([[297,254,300],
                            [109,69,77],
                            [70,69,63],
                            [115,81,83]])


    def __get_byID(self,ID): # 1# forward 2#left 3#back 4#right
        kk = -1*np.ones(3)
        self.ser.write(pack('B',ID))
        buf = self.ser.read(7)
        if len(buf)==7:
            BUF = unpack('7B',buf)
            if 255-BUF[0]==ID:
                for k in range(3):
                    kk[k] = int((BUF[1+2*k]+BUF[2+2*k]*256)*1.08507*172/1000.)
        return kk

    def __get_all(self):
        for ID in range(1,5):
            kk = self.__get_byID(ID)
            if kk[0]>0:
                self.r[ID-1]=kk
        t = self.r-self.ST #4x3
        self.rdata[0:2]=self.rdata[1:]
        self.rdata[2]=t
        return np.median(self.rdata,axis=0)

    @property
    def freshdata(self):
        return self.__get_all()