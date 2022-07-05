# Digital Control Systems Design project, generation of the C OSQP library.
# Nicola ROMANO         0622701549  N.ROMANO24@STUDENTI.UNISA.IT
# Antonio ROTONDO       0622701489  A.ROTONDO9@STUDENTI.UNISA.IT
# Catello SORRENTINO    06227001490 C.SORRENTINO61@STUDENTI.UNISA.IT

from math import inf
import numpy as np
from scipy import sparse
import osqp
from scipy.io import loadmat

# Import the workspace to get the matrices
workspace= loadmat('ny10nu2.mat')

S=workspace['S']
X=workspace['X']
CC=workspace['CC']
dd=workspace['dd']
du=workspace['du']
ddu=workspace['ddu']
dy=workspace['dy']

# Change format according the required one
P = sparse.csc_matrix(S)
q = np.array(np.zeros(len(S)))
A = sparse.csc_matrix(CC) 

# Load the bounds
l = np.array(-inf*np.ones(len(dd)))
u = np.array(dd) 

# Create the OSQP object
problem = osqp.OSQP()

# Setup workspace
problem.setup(P=P, q=q, A=A, l=l, u=u, alpha = 1.0, max_iter = 5, eps_rel = 0.05, warm_start = False)

# generation of C code in the "code" folder
problem.codegen('code')