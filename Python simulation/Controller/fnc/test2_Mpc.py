from initialize import centerline_ini
from car1 import vehicle_dynamic
from stanley import stanley
from MPC2 import MPC
from plotfun1 import plotfun

import numpy as np


centerline=centerline_ini('centerline_topi.csv')
car=vehicle_dynamic([0,0,0,30,0,0])
controller=MPC()

controller.Np=30
controller.testflag=1
controller.Q=np.array([1e4,0,0,1e6,50,0.1])
controller.R=np.array([0.1,1e3])

controller.update(car,centerline)