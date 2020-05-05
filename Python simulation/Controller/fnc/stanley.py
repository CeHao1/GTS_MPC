import numpy as np


class stanley():
    def __init__(self):
        self.ks=1
        self.lamb=0.5
        self.steerlimit=6/180*np.pi
        self.gap=2
        self.maxa=2
        self.u=np.zeros((2,1))
        self.testflag=0


    def update(self,car,reference):
        # input: car.states
        idealVx=reference.intp_Vx(car.s)
        Vx=car.states[3]
        
        # lateral steer
        steer=-(self.ks*car.epsi+np.arctan2(self.lamb*car.ey,Vx))
        if steer>self.steerlimit:
            steer=self.steerlimit
        elif steer<-self.steerlimit:
            steer=-self.steerlimit
            
        # longitudinal a
        fb=idealVx-Vx
        if abs(fb)<self.gap:
            a=self.maxa*fb/self.gap
        else:
            a=self.maxa*np.sign(fb)
        
        self.u[0]=a
        self.u[1]=steer