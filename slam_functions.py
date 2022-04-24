import numpy as np
from scipy.sparse import csr_matrix, csc_matrix
from scipy.sparse.linalg import spsolve
from scipy.sparse.linalg import lsqr
from numpy.linalg import inv


def determine_angle(angle):
    ang1 = angle + 2 * np.pi
    ang2 = angle - 2 * np.pi
    angles = [ang2, angle, ang1]
    abs_angles = [abs(ang2), abs(angle), abs(ang1)]
    idx = abs_angles.index(min(abs_angles))
    return angles[idx]


def wraptopi(x):
    return (x + np.pi) % (2 * np.pi) - np.pi


def solve(A, b, sparse_solve):
    """
    Solve sparse linear system A * dz = b
    """
    # zero_row = np.where(~A.any(axis=1))[0]
    # print('zero rows of A', zero_row)
    if sparse_solve:
        # Transformation to sparse matrix form
        A_sparse = csr_matrix(A, dtype=float)
        # Solve the sparse linear system Ax=b
        dz = spsolve(A_sparse, b)
        # dz = lsqr(A_sparse, b, damp=0.8)[0]
        # print("First 10 components of dz:", np.array(dz)[:10])
        # print("Last 10 components of dz:", np.array(dz)[-10:])
    else:
        # Solve the linear system
        dz = np.linalg.solve(A, b)

    # Check NAN
    dz[np.isnan(dz)] = 0

    return dz
