import numpy as np
from scipy.sparse import csr_matrix
from cvxopt.solvers import qp
from cvxopt import spmatrix, matrix, solvers
from matplotlib import pyplot as plt

solvers.options['show_progress'] = False

class MPC:
    def __init__(self):
        self.testflag=0  # default testfalg=0 no test
        
        self.Ns=6
        self.Nc=2
        self.Np=30
        self.dt=1/20
        self.Q=np.array([10,0,0,100,50,1])
        self.R=np.array([0.1,500])
        self.steer_change_limit=10/180*np.pi
        self.steerlimit=6/180*np.pi
        self.Kca=1
        self.m=1060
        self.Iz=1493.4
        self.lf=1.0462
        self.lr=1.2638
        self.Caf=33240*2
        self.Car=33240*2
        
        self.u=np.zeros((2,1)) # a, steer
        self.Vx_all=[]
        self.kap=[]
        self.x0=np.zeros((self.Ns,1))
        
    def get_horizon(self,car,reference):
        Ns=self.Ns
        Np=self.Np
        horizon=car.s+np.arange(Np)*self.dt*car.states[3]/Np
        self.Vx_all=reference.intp_Vx(horizon)
        self.kap=reference.intp_kap(horizon)
        # vx,vy,dpsi,epsi,ey,steer
        self.x0=np.reshape(np.array([car.states[3],car.states[4],car.states[5],
                          car.epsi,car.ey,self.u[1,0]]),(Ns,1))
        
        '''
        test MPC when testflag=1
        '''
        if self.testflag==1:
            self.Vx_all=np.arange(Np)*0.002+30
            self.kap=np.arange(Np)*0.0001
        
        
    def update(self,car,reference):
        self.get_horizon(car,reference)
        
        # retrive value
        Ns=self.Ns
        Nc=self.Nc
        Np=self.Np
        dt=self.dt
        Q=self.Q
        R=self.R
        steerlimit=self.steerlimit
        steer_change_limit=self.steer_change_limit
        Kca=self.Kca
        m=self.m
        Iz=self.Iz
        lf=self.lf
        lr=self.lr
        Caf=self.Caf
        Car=self.Car
        Vx_all=self.Vx_all
        kap=self.kap
        Vx=car.states[3]
        x0=self.x0
        
        # *************build sequential state space A_qp*x=b_qp*************
        
        A11=-(Caf+Car)/(m*Vx);
        A12=-Vx-(Caf*lf-Car*lr)/(m*Vx);
        A21=-(Caf*lf-Car*lr)/(Iz*Vx);
        A22=-(Caf*lf**2+Car*lr**2)/(Iz*Vx);
        B1=Caf/m;
        B2=Caf*lf/Iz;
        
        Ac=csr_matrix(([A11,A12,A21,A22,B1,B2,1,1],([1,1,2,2,1,2,3,4],[1,2,1,2,5,5,2,1])),
                      shape=(Ns,Ns)).toarray()
        Bc=csr_matrix(([1,1],([0,5],[0,1])),shape=(Ns,Nc)).toarray()
        
        Ad=np.eye(Ns)+Ac*dt
        Bd=Bc*dt
        bd=np.zeros((Ns,1))
        x_target=np.zeros((Ns,1))          
        
        A_combined1=np.eye((Np+1)*Ns)
        A_combined2=np.zeros(((Np+1)*Ns,Np*Nc))
        b_qp=np.zeros(((Np+1)*Ns,1))
        
        b_qp[np.ix_(np.arange(Ns),[0])]=x0
        x_target_qp=np.zeros(((Np+1)*Ns+Np*Nc,1))
        
        
        for index in range(Np):
            ind1=Ns+index*Ns+np.arange(Ns)
            ind2x=index*Ns+np.arange(Ns)
            ind2u=index*Nc+np.arange(Nc)
            
            Ad[4,3]=Vx_all[index]*dt
            bd[3,0]=-Vx_all[index]*kap[index]*dt
            x_target[0,0]=Vx_all[index]
            
            A_combined1[np.ix_(ind1, ind2x)]=-Ad
            A_combined2[np.ix_(ind1, ind2u)]=-Bd
            b_qp[np.ix_(ind1, [0])]=bd
            x_target_qp[np.ix_(ind1, [0])]=x_target   
        
        A_qp = np.hstack((A_combined1, A_combined2))
        
        A_sparse = spmatrix(A_qp[np.nonzero(A_qp)], np.nonzero(A_qp)[0].astype(int), 
                            np.nonzero(A_qp)[1].astype(int), A_qp.shape)  
        b_m=matrix(b_qp)

        # *********build  constraints of states lb_qp<= x <= ub_qp*************

        # Gx=np.zeros((Np+1,(Np+1)*Ns+Np*Nc))
        # Gu=np.zeros((Np,(Np+1)*Ns+Np*Nc))
        ind_lx=5+np.arange(Np+1)*Ns  # steer is the 6th
        ind_lu=1+(Np+1)*Ns+np.arange(Np)*Nc  # d_steer is the 2nd
        # Gx[np.ix_(np.arange(Np+1),ind_lx)]=1
        # Gu[np.ix_(np.arange(Np),ind_lu)]=1
        
        Gx=csr_matrix((np.ones(Np+1),(np.arange(Np+1),ind_lx)),
                      shape=(Np+1,(Np+1)*Ns+Np*Nc)).toarray()
        Gu=csr_matrix((np.ones(Np),(np.arange(Np),ind_lu)),
                      shape=(Np,(Np+1)*Ns+Np*Nc)).toarray()
        
        G_qp=np.vstack((Gx,-Gx,Gu,-Gu))
        
        hx=np.ones((Np+1,1))*steerlimit
        hu=np.ones((Np,1))*steer_change_limit
        h_qp=np.vstack((hx,hx,hu,hu))
        
        G_sparse = spmatrix(G_qp[np.nonzero(G_qp)], np.nonzero(G_qp)[0].astype(int),
                            np.nonzero(G_qp)[1].astype(int), G_qp.shape)
        h_m=matrix(h_qp)
        
        # print(h_qp.shape)
        
        
        # *********build weight matrix*********
        
        # here change the last weight!
        Q_w0=np.tile(Q,(1,Np))
        Q_end=Q.reshape(1,Ns)
        Q_end[0,3]*=1
        Q_w=np.hstack((Q_w0,Q_end))
        
        R_w=np.tile(R,(1,Np))
        
        Q_qp=np.diag(np.hstack((Q_w,R_w))[0])
        P_qp=-np.dot(x_target_qp.T,Q_qp)
        
        Q_sparse = spmatrix(Q_qp[np.nonzero(Q_qp)], np.nonzero(Q_qp)[0].astype(int), 
                            np.nonzero(Q_qp)[1].astype(int), Q_qp.shape)
        P_m=matrix(P_qp.T)
            
        
        # *********solver qp*********
        sol=qp(Q_sparse, P_m, G_sparse, h_m, A_sparse, b_m)
        x=np.reshape(np.squeeze(sol['x']),((Np+1)*Ns+Np*Nc,1))
        
        xprd=x[np.arange((Np+1)*Ns)]
        uprd=x[(Np+1)*Ns+np.arange(Np*Nc)]
        
        self.u[0,0]=uprd[0,0]
        self.u[1,0]=xprd[5+Ns,0]
        
        if self.testflag==1:
            self.plot(xprd,uprd)
        
    def plot(self,xprd,uprd):
        Ns=self.Ns
        Np=self.Np
        Nc=self.Nc
        
        name_x={1:'vx',2:'vy',3:'dpsi',4:'epsi',5:'ey',6:'steer'}
        name_u={1:'a',2:'dsteer'}
        
        plt.figure(1)
        for plti in range(Ns):
            plt.subplot(2,4,plti+1)
            if plti==5:
                K=180/np.pi
                # K=1
            else:
                K=1
                
            plt.plot(xprd[plti+np.arange(Np+1)*Ns,0]*K)
            plt.title(name_x[plti+1])
            if (plti==2):
                plt.plot(self.Vx_all*self.kap,'r')
            elif plti==0:
                plt.plot(self.Vx_all,'r')
            
        # plt.figure(2)
        for plti in range(Nc):
            plt.subplot(2,4,plti+7)
            plt.plot(uprd[plti+np.arange(Np)*Nc,0])
            plt.title(name_u[plti+1])
            
        plt.show()
        
        
        
        
        
        
