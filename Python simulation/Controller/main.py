import numpy as np
import copy
import time

import os
import sys

current_dir = os.getcwd()    # obtain work dir
sys.path.append(current_dir+'\\fnc') # add work dir to sys path

# car1 is the dynamic model (nonlinear)
# MPC1 is the MPC controller
# stanley is the stanley controller
# initialize is the tokyo expressway data
# plotfun1 is the ploting function


# self function
# from initialize import centerline_ini, reference_ini
from initialize import reference_ini
from car1 import vehicle_dynamic
# from stanley import stanley
# from MPC_g import MPC
# from MPC_s2 import MPC
from MPC_simply_g import MPC
# from MPC_cvxpy import MPC
# from plotfun1 import plotfun
from sv_csv import save_restul


# read centerline
# centerline=centerline_ini('reference4000_tokyo.csv')
# reference=reference_ini('reference1.csv')

reference_name='reference2000_93.csv'
ref_name=current_dir+'\\reference\\'+reference_name
# name='reference4000_tokyo.csv'
reference=reference_ini(ref_name)

# initialize car states X,Y,psi,Vx,vy,dpsi
car=vehicle_dynamic([reference.X[0],reference.Y[0],
                     reference.psi[0],15,0,0])
# reference.Vx[0]

#---------------------------------here select a controller you want, stanley or MPC-----------------
# and you can set the weight matrix here or use the defult value
# controller=stanley()  
controller=MPC()
# controller.Q=np.array([1,0.05,0,0.03,10,0.001])
# controller.R=np.array([0.01,100])

#------------------if testflag==1, run the testing mode of MPC-----------

if controller.testflag==0:
    # simulation times with 60 Hz
    # sim_number=60*203
    sim_number=60*100
else:
    sim_number=1
    car.states=np.array([1,0.05,0,0.03,100,0.001])


#-----------------------save the path data for visualization--------------------------
    
# save path
sum_states=[copy.copy(car.states)]
sum_error=[copy.copy([car.s,car.epsi,car.ey])]
sum_u=[copy.copy(controller.u)]
sum_t=np.zeros((sim_number+1,1))

start=time.perf_counter()
# runing update
#--------------------here is the main iteration and simulation of dyamic and controller
for t in range(sim_number):
    if t%60==0:
        print('run time: '+str(t/60)+' s')
    
    controller.update(car,reference)
    car.update(controller.u)
    car.get_e(reference)
    # append states 
    
    sum_states.append(copy.copy(car.states))
    sum_error.append(copy.copy([car.s,car.epsi,car.ey]))
    sum_u.append(copy.copy(controller.u))
    sum_t[t+1,0]=t/60
 # calculate the totall running time
end=time.perf_counter()
print('running time: ',str(end-start))


# save result to matlab

sum_states=np.array(sum_states)
sum_error=np.array(sum_error)
sum_u=np.array(sum_u)

svfilename=current_dir+'\\matplot\\'+'save_data.mat'
save_restul(svfilename,sum_states,sum_error,sum_u,sum_t)


#  inner plot
# controller.testflag==1
# if controller.testflag==0:
#     # to plot
#     sum_states=np.array(sum_states)
#     sum_error=np.array(sum_error)
#     sum_u=np.array(sum_u)
    
#     # define plotmap
#     map_name=current_dir+'\\reference\\'+'centerline_map.csv'
#     plot_map=plotfun(map_name,sum_states,sum_error,sum_u)
#     plot_map.plot(reference)
    
#     # find loop time
#     ds=np.diff(sum_error[:,0])
#     max_s=np.min(ds)
#     max_si=np.argmin(ds)
#     if max_s<0:
#         print('lap time= '+str(sum_t[max_si,0]))
#     else:
#         print('less than 1 loop')
    













