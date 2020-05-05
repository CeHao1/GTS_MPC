# this is the vehicle dynamic function
import numpy as np


class vehicle_dynamic():
    def __init__(self,reference_states):
        # X,Y, psi, vx,vy,dpsi 6
        self.states=[]
        self.s=0
        self.epsi=0
        self.ey=0
        
        self.m=1060
        self.Iz=1493.4
        self.lf=1.0462
        self.lr=1.2638
        # self.Caf=33240*2
        # self.Car=33240*2
        self.a11=4800*2
        self.a12=7.0
        self.a21=3720*2
        self.a22=10.3
        self.w_wind=2.7e-4
        self.dt=1/60
        self.ds=0.01
        
        
        
        self.initial(reference_states)
        
    def initial(self,reference_states):
        self.states=reference_states
        
        
    def update(self,u):
        #u=a,steer
        a=u[0,0]
        delta=u[1,0]
        
        psi=self.states[2]
        vx=self.states[3]
        vy=self.states[4]
        dpsi=self.states[5]
        m=self.m
        Iz=self.Iz
        lf=self.lf
        lr=self.lr
        # Caf=self.Caf
        # Car=self.Car
        a11=self.a11
        a12=self.a12
        a21=self.a21
        a22=self.a22
        w_wind=self.w_wind
        dt=self.dt
        
        
        alpf=delta-np.arctan2(vy+lf*dpsi,vx)
        alpr=-np.arctan2(vy-lr*dpsi,vx)
        
        # linear tire
        # Fyf=Caf*alpf
        # Fyr=Car*alpr
        
        # tanh tire
        Fyf=a11*np.tanh(a12*alpf)
        Fyr=a21*np.tanh(a22*alpr)
        a_wind=w_wind*vx**2
        
        
        #update
        self.states[0]+=(vx*np.cos(psi)-vy*np.sin(psi))*dt                  #X      0
        self.states[1]+=(vx*np.sin(psi)+vy*np.cos(psi))*dt                  #Y      1
        self.states[2]+=(dpsi)*dt                                           #psi    2
        self.states[3]+=(a-(Fyf*np.sin(delta))/m+dpsi*vy-a_wind)*dt         #vx     3
        self.states[4]+=((Fyf*np.cos(delta)+Fyr-dpsi*vx)/m)*dt              #vy     4
        self.states[5]+=((lf*Fyf*np.cos(delta)-lr*Fyr)/Iz)*dt               #dpsi   5
        
        
    def get_e(self,reference):
        ds=self.ds
        predict_points=40
        predict_s=self.s+self.states[3]*self.dt # Vx*dt+s0
        possible_course=predict_s+(np.arange(predict_points)-predict_points/2) \
            /predict_points*ds
        
        # exceed 
        index1=(possible_course>max(reference.course))
        #below
        index2=(possible_course<min(reference.course))
        
        possible_course[index1]-=max(reference.course)
        possible_course[index2]+=max(reference.course)
        
        X_p=reference.intp_X(possible_course)
        Y_p=reference.intp_Y(possible_course)
        dist=(X_p-self.states[0])**2+(Y_p-self.states[1])**2
        min_val=np.min(dist)
        min_num=np.argmin(dist)
        self.s=possible_course[min_num]
        
        psi_ref=reference.intp_psi(self.s)
        self.epsi=wrapToPi(self.states[2]-psi_ref)
        ang_ey=np.arctan2(self.states[1]-Y_p[min_num],self.states[0]-X_p[min_num])
        self.ey=np.sqrt(min_val)*np.sign(wrapToPi(ang_ey-psi_ref))
        
        
def wrapToPi(x):
    x=x%(2*np.pi)
    if x<=-np.pi:
        x+=2*np.pi
    elif x>np.pi:
        x-=2*np.pi  
    return x

    
    

