import numpy as np
from scipy.sparse import csr_matrix 


import gurobipy as gp
from gurobipy import GRB


class MPC:
    def __init__(self):
        self.testflag=0  # default testfalg=0 no test
        
        self.Ns=6
        self.Nc=2
        self.Np=30
        self.dt=1/30
        self.Q=np.array([1,0.05,0,0.03,100,0.001])
        self.R=np.array([0.01,0.1])
        self.steer_change_limit=100/180*np.pi
        self.steerlimit=30/180*np.pi
        self.m=1060
        self.Iz=1493.4
        self.lf=1.0462
        self.lr=1.2638
        self.a11=4800*2
        self.a12=7.0
        self.a21=3720*2
        self.a22=10.3
        self.w_wind=2.7e-4
        self.amax=1.3
        self.amin=-9.5
        
        
        self.u=np.zeros((2,1)) # a, steer
        self.Vx_all=[]
        self.kap=[]
        self.x0=np.zeros((self.Ns,1))
        
        self.G_sparse=[]
        self.h_m=[]
        self.Q_qp=[]
        self.w_kap=self.Q[3]
        
        self.initialize()
        
        
    def initialize(self):
               
         # *********build  constraints of states lb_qp<= x <= ub_qp*************
         Np=self.Np
         Ns=self.Ns
         Nc=self.Nc
         steerlimit=self.steerlimit
         steer_change_limit=self.steer_change_limit
         amax=self.amax
         amin=self.amin
         Q=self.Q
         R=self.R
         
         ind_lx=5+np.arange(Np+1)*Ns  # steer is the 6th
         ind_lu=1+(Np+1)*Ns+np.arange(Np+1)*Nc  # d_steer is the 2nd
         ind_a=0+(Np+1)*Ns+np.arange(Np+1)*Nc  # acc  
         
         # steer limit
         Gx=csr_matrix((np.ones(Np+1),(np.arange(Np+1),ind_lx)), 
          shape=(Np+1,(Np+1)*(Ns+Nc))).toarray()
         
         # d_steer limit
         Gu=csr_matrix((np.ones(Np+1),(np.arange(Np+1),ind_lu)),
          shape=(Np+1,(Np+1)*(Ns+Nc))).toarray()
         
         # acceleration limit
         Ga=csr_matrix((np.ones(Np+1),(np.arange(Np+1),ind_a)),
          shape=(Np+1,(Np+1)*(Ns+Nc))).toarray()
         
         G_qp=np.vstack((Gx,-Gx,Gu,-Gu,Ga,-Ga))
         
         hx=np.ones((Np+1,1))*steerlimit
         
         hu=np.ones((Np+1,1))*steer_change_limit
         
         ha_up=np.ones((Np+1,1))*amax
         ha_down=-np.ones((Np+1,1))*amin
         h_qp=np.vstack((hx,hx,hu,hu,ha_up,ha_down))
         
         
         self.G_sparse=csr_matrix(G_qp)
         # self.h_m=csr_matrix(np.squeeze(h_qp))
         self.h_m=np.squeeze(h_qp)
         
         # *********build weight matrix*********
         
         Q_w=np.tile(Q,(1,Np+1))
         R_w=np.tile(R,(1,Np+1))
         
         self.Q_qp=np.diag(np.hstack((Q_w,R_w))[0])
         
         
         
        
         
        
        
    def get_horizon(self,car,reference):
        Ns=self.Ns
        Np=self.Np
        horizon=car.s+np.arange(Np+1)*self.dt*car.states[3]/(Np+1)
        # if np.max(horizon)>np.max(reference.course):
        index=(horizon>np.max(reference.course))
        horizon[index]-=np.max(reference.course)
       
        self.Vx_all=reference.intp_Vx(horizon)
        self.kap=reference.intp_kap(horizon)
        # vx,vy,dpsi,epsi,ey,steer
        self.x0=np.reshape(np.array([car.states[3],car.states[4],car.states[5],
                          car.epsi,car.ey,self.u[1,0]]),(Ns,1))

        
        
    def update(self,car,reference):
        self.get_horizon(car,reference)
        
        # retrive value
        Ns=self.Ns
        Nc=self.Nc
        Np=self.Np
        dt=self.dt
        # Q=self.Q
        # R=self.R
        # steerlimit=self.steerlimit
        # steer_change_limit=self.steer_change_limit
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
        # amax=self.amax
        # amin=self.amin
        
        Vx_all=self.Vx_all
        kap=self.kap
        Vx=car.states[3]
        Vy=car.states[4]
        dpsi=car.states[5]
        delta=self.u[1,0]
        x0=self.x0
        
        
        G_sparse=self.G_sparse
        h_m=self.h_m
        Q_qp=self.Q_qp+0.0
        w_kap=self.w_kap
        
        # *************build sequential state space A_qp*x=b_qp*************
        
        alpf=delta-np.arctan2(Vy+lf*dpsi,Vx)
        alpr=-np.arctan2(Vy-lr*dpsi,Vx)
        Fyf=a11*np.tanh(a12*alpf)
        Fyr=a21*np.tanh(a22*alpr)
        Kf=a11*a12*(1-np.tanh(a12*alpf)**2)
        Kr=a21*a22*(1-np.tanh(a22*alpr)**2)
        bf=Fyf-Kf*alpf
        br=Fyr-Kr*alpr
        
        A22=-(Kf+Kr)/(m*Vx);
        A23=-Vx-(Kf*lf-Kr*lr)/(m*Vx);
        A32=-(Kf*lf-Kr*lr)/(Iz*Vx);
        A33=-(Kf*lf**2+Kr*lr**2)/(Iz*Vx);
        B26=Kf/m;
        B36=Kf*lf/Iz;
        
        a_wind=w_wind*Vx**2;
        D1=-(Fyf*np.sin(delta))/m+dpsi*Vy-a_wind;
        D2=(bf+br)/m;
        D3=(lf*bf-lr*br)/Iz;
        
        Ac=csr_matrix(([A22,A23,A32,A33,B26,B36,1,1,Vx],([1,1,2,2,1,2,3,4,4],[1,2,1,2,5,5,2,1,3])),
                      shape=(Ns,Ns)).toarray()
        # Ac=csr_matrix(([A22,A23,A32,A33,B26,B36,1,1],([1,1,2,2,1,2,3,4],[1,2,1,2,5,5,2,1])),
        #               shape=(Ns,Ns)).toarray()
        Bc=csr_matrix(([1,1],([0,5],[0,1])),shape=(Ns,Nc)).toarray()
        
        B0=csr_matrix(([1],([5],[1])),shape=(Ns,Nc)).toarray()*dt
        
        Ad=np.eye(Ns)+Ac*dt
        Bd=Bc*dt
        bd=np.zeros((Ns,1))
        x_target=np.zeros((Ns,1))          
        
        A_combined1=np.eye((Np+1)*Ns)
        A_combined2=np.zeros(((Np+1)*Ns,(Np+1)*Nc))
        b_qp=np.zeros(((Np+1)*Ns,1))
        
        b_qp[np.ix_(np.arange(Ns),[0])]=x0
        x_target_qp=np.zeros(((Np+1)*Ns+(Np+1)*Nc,1))
        
        
        # initial update
        
        A_combined2[np.ix_(np.arange(Ns), np.arange(Nc))]=-B0
        
        x_target[0,0]=Vx_all[0]
        x_target_qp[np.ix_(np.arange(Ns), [0])]=x_target   
        
        #  build state-space
        for index in range(Np):
            ind1x=Ns+index*Ns+np.arange(Ns)
            ind1u=Ns+index*Ns+np.arange(Ns)
            ind2x=index*Ns+np.arange(Ns)
            ind2u=Nc+index*Nc+np.arange(Nc)
            
            # Ad[4,3]=Vx_all[index]*dt
            bd[0,0]=D1*dt
            bd[1,0]=D2*dt
            bd[2,0]=D3*dt
            # bd[3,0]=-Vx_all[index]*kap[index]*dt
            bd[3,0]=-Vx*kap[index]*dt
            x_target[0,0]=Vx_all[index+1]
            
            A_combined1[np.ix_(ind1x, ind2x)]=-Ad
            A_combined2[np.ix_(ind1u, ind2u)]=-Bd
            b_qp[np.ix_(ind1x, [0])]=bd
            x_target_qp[np.ix_(ind1x, [0])]=x_target   
        
        A_qp = np.hstack((A_combined1, A_combined2))
        
                

       #=========== weight 2======================
       
        # Q=self.Q
        # R=self.R
       
        # Q_w=np.tile(Q,(1,Np+1))
        # R_w=np.tile(R,(1,Np+1))
         
        # Q_qp=np.diag(np.hstack((Q_w,R_w))[0])
       
        ind_q2=1+np.arange(Np+1)*Ns
        ind_q4=3+np.arange(Np+1)*Ns
         
         
        
        Lshape=(Np+1)*(Ns+Nc)
        Q22=csr_matrix((np.ones(Np+1)*w_kap/Vx**2,(ind_q2,ind_q2)),shape=(Lshape,Lshape)).toarray()
        Q24=csr_matrix((np.ones(Np+1)*w_kap/Vx**2,(ind_q2,ind_q4)),shape=(Lshape,Lshape)).toarray()
        Q42=csr_matrix((np.ones(Np+1)*w_kap/Vx**2,(ind_q4,ind_q2)),shape=(Lshape,Lshape)).toarray()
         
        Q_qp+=Q22+Q24+Q42
        P_qp=-np.dot(x_target_qp.T,Q_qp)
        
                 
        # ================gurobi===================
        
        A_sparse=csr_matrix(A_qp)
        b_m=np.squeeze(b_qp)
           
        Q_sparse=csr_matrix(Q_qp)
        P_m=csr_matrix(np.squeeze(P_qp))
   
                
        m = gp. Model (" matrix1 ")
        m. Params . OutputFlag = 0

        x = m. addMVar ( shape =(Np+1)*(Ns+Nc),lb=-GRB.INFINITY, vtype =GRB.CONTINUOUS , name ="x")
        
        m. setObjective (x @ Q_sparse @ x + 2*P_m @ x, GRB. MINIMIZE )
        
        m. addConstr (G_sparse @ x <= h_m , name ="inequ")
        
        m. addConstr (A_sparse @ x == b_m , name ="equ")

        
        m. optimize ()

        # acc=x.X[(Np+1)*Ns+Nc]
        steer=x.X[5]
        
      


        #============== get u=================
        
        sl=6/180*np.pi
        if np.abs(steer)>sl:
            steer=sl*np.sign(steer)
        
        # self.u[0,0]=acc
        self.u[0,0]=x.X[(Np+1)*Ns+Nc]
        self.u[1,0]=steer


        
        
        
        
        
