from matplotlib import pyplot as plt
import csv
import numpy as np

class plotfun:

    def __init__(self,readpath,sum_states,sum_error,sum_u):
        self.Xc=[]
        self.Yc=[]
        self.Xl=[]
        self.Yl=[]
        self.Xr=[]
        self.Yr=[]
        self.states=sum_states
        self.error=sum_error
        self.u=sum_u
        self.read(readpath)
        
    def read(self,readpath):
        f=open (readpath,'r')
        f_csv = csv.reader(f)
        for row0 in f_csv:
            row=list(map(lambda x: float(x), row0))
            self.Xc.append(row[0])
            self.Yc.append(row[1])
            self.Xl.append(row[2])
            self.Yl.append(row[3])
            self.Xr.append(row[4])
            self.Yr.append(row[5])
        
    
    def plot(self,reference):
        
        states=self.states
        error=self.error
        u=self.u
        
        plt.close()
        # X-Y
        plt.figure(1)
        plt.plot(states[:,0],states[:,1],'k','linewidth',2)
        # plt.plot(self.Xc,self.Yc,'b--')
        plt.plot(reference.X,reference.Y,'b--')
        plt.plot(self.Xl,self.Yl,'r--')
        plt.plot(self.Xr,self.Yr,'m--')
        
        plt.title('X-Y position')
        
        # vx,vy,epsi,ey,a,steer
        plt.figure(2)
        
        plt.subplot(221) # should hold on
        plt.plot(states[:,3])
        plt.plot(reference.intp_Vx(error[:,0]),'r') # use s to interpolate
        plt.title('Vx')
        
        plt.subplot(222)
        plt.plot(states[:,4])
        plt.title('Vy')
        
        plt.subplot(223)  # should hold on
        plt.plot(states[:,5])
        plt.plot(reference.intp_Vx(error[:,0])*reference.intp_kap(error[:,0]),'r') # use s to interpolate
        plt.title('dpsi')
        
        plt.subplot(224)
        plt.plot(error[:,0])
        plt.title('course s')
        
        
        plt.figure(3)
        plt.subplot(221)
        plt.plot(error[:,1]/np.pi*180)
        plt.title('epsi/ degree')
        
        plt.subplot(222)
        plt.plot(error[:,2])
        plt.title('ey')
        
        plt.subplot(223)
        plt.plot(u[:,0])
        plt.title('a')
        
        plt.subplot(224)
        plt.plot(u[:,1]/np.pi*180)
        plt.title('steer/ degree')
        
        
        plt.show()
        
        
        
    
        
    
    
    
    
    
    
    
    