#coding:utf-8
from struct import pack,unpack
import numpy as np

class MecanumBase():
    '''
    author:mrtang
    date:2015.8
    version:3.0
    email:mrtang@nudt.edu.cn

    note:
    The unit of AngleVel in stm32 chip is changed to 0.1rad/s.
    However, in the application level the unit of v is still rad/s.
    In another word, in application functions, the smallest v could be 0.1. 
    The charge port is on the back of Platform.
    '''

    def __encode__(self,vel,angle,angle_v,angle_vd):
        if vel<0:	vel = 0
        vel = int(vel)%3000
        angle_v = abs(int(angle_v))
        angle = abs(angle)%360		# safty parameters

        Lv = vel%256				#low byte of vel
        Hv = (vel>>8)%256			#hight byte of vel
        La = angle%256				#low byte of angle
        Ha = angle>>8				#hgih byte of angle
        av = angle_v%256			#angle vel. unit 0.1degree/s 
        avd = abs(angle_vd)%2		#direction of angle vel. 0 counter clockwise, 1 clockwise
        check = 255-(161+Lv+Hv+La+Ha+av+avd)%256	#check byte

        cmd = np.array([85,170,30,253,8,161,Lv,Hv,La,Ha,av,avd,0,check],dtype=np.uint8)
        # buf = map(lambda i:pack('B',i),cmd)
        # buffer = ''.join(buf)
        buffer = cmd.tostring()
        return buffer

    def stop(self):
        return self.__encode__(0,0,0,0)

    def translateV(self,v,d): #mm/s
        '''translate with vel v, angle d, directly
        d: 0-forward 90 left 180 back 270 right
        '''
        if v<0:v=0
        v = 0.815*v    #calibrate the vel
        return self.__encode__(v,d,0,0)

    def __encode__A(self,v): #v度/s 分辨率0.1du/s
        '''	v: angle velocity, - counter clockwise, - counter clockwise		'''
        if v<0:		d = 0
        elif v>0:	d = 1
        else:	return self.__encode__(0,0,0,0)
        v = 0.8*v
        return self.__encode__(0,0,abs(v)*10,d)

    def rotateV(self,v): #度/s
        '''rotate with vel v, rudely
        v positive->counterclockwise'''
        
        return self.__encode__A(v)

    def _dir(self,v):
        if v>=0:	return 0
        elif v<0:	return 180

    def __encode__T(self,v,r):
        '''	v: velocity		r: the radius of the turn		'''
        self.c_tv = v
        V = abs(v)
        R = abs(r)
        av = 180*V/(3.14*R)
        VV = int(V+0.5*av)

        if v>0:
            if r<=0:    d = 0
            else:   d = 1
        else:
            if r<=0:    d = 1
            else:       d = 0

        return self.__encode__(VV,self._dir(v),av*10,d)

    def turn(self,v,r):
        '''turn with car style. v-vel: +forward;-backward; r-radius of turn: -left turn; + right turn.'''
        return self.__encode__T(v,r)

    def setPort(self,port='wireless'):
        #控制选择，支持有线和蓝牙端口。任何时候任何一个端口都可以通过该命令抢占控制权。
        if port == 'wire':
            cmd = [85,170,30,253,8,188,0,0,0,0,0,0,0,67]
            return ''.join([pack('B',i) for i in cmd])
        elif port == 'wireless':
            cmd = [85,170,30,253,8,187,0,0,0,0,0,0,0,68]
            return ''.join([pack('B',i) for i in cmd])