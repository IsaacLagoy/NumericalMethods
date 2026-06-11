# null space

# used for equality constraints
# search for a minimum without leaving the feasible space

# null space of matrix A is the set if vectors x such that Ax = 0
# we can move in these direction and produce zero change in Ax

# any step, delta x, can be decomposed into two parts
# range component - corrects violations
# null space component - moves in the feasible surface

# find a basis Z whose columns span the null space of A
# x = x_particular + Zp
# x_particular is any feasible point
# p is a free vector to optimize over

# computing the null space basis is usually done with QR or SVD

# problem set up
# f(x) = (x_0 - 4)^2 + (x_1 - 3)^2 + (x_2 - 2)^2
# x_0 + x_2 + x_3 = 6
# x_1 - x_2 = 0

import numpy as np

def f(x: np.ndarray) -> float:
    return (x[0] - 4) ** 2 + (x[1] - 3) ** 2 + (x[2] - 2) ** 2

def grad_f(x: np.ndarray) -> np.ndarray:
    return np.array([2 * (x[0] - 4), 2 * (x[1] - 3), 2 * (x[2] - 2)])

# TODO learn SVD and QR decomposition to compute the null space
def null_space(A: np.ndarray) -> np.ndarray:
    U, s, VT = np.linalg.svd(A, full_matrices=True)
    tol = max(A.shape) * np.finfo(float).eps * s[0]
    rank = np.sum(s > tol)
    return VT[rank:].T

# constraints in matrix form
A = np.array([
    [1, 1, 1],
    [1, -1, 0]
])

b = np.array([6, 0])

Z = null_space(A)
print(Z)

# find a feasible point
x = np.linalg.lstsq(A, b, rcond=None)[0]

# gradient descent
alpha = 0.1
for i in range(10):
    g = grad_f(x)
    g_null = Z @ Z.T @ g
    x = x - alpha * g_null
    print(x, '\t', np.sum(x), '\t', x[0] - x[1])
