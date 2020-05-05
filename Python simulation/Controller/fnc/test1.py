
import numpy as np
# import scipy . sparse as sp
# from scipy import sparse
from scipy.sparse import csr_matrix 


a=np.array([[1,2,0],[0,0,5]])
b=csr_matrix(a)
c=b.toarray()

print(c)