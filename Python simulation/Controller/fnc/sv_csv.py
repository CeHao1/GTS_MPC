
import numpy as np
import scipy.io as sio # mat

def save_restul(filename,sum_states,sum_error,sum_u,sum_t):

    sio.savemat(filename, {'states':sum_states,'error':sum_error,'input':sum_u,'t':sum_t})
    print('done')
