# projection methods

# core idea:
# 1) take a full step
# 2) project the step onto the feasible set

# the "projection" of a point x onto a set C
# P_C(x) = argmin_{y in C} ||x - y||

# the update rule is simply
# x_{k+1} = P_C(x_k + step_k)

# problem set up
# f(x) = (x_0 - 3)^2 + (x_1 - 5)^2
# x_0 + x_1 = 4
# x_1 >= 1

import numpy as np

def f(x: np.ndarray) -> float:
    return (x[0] - 3) ** 2 + (x[1] - 5) ** 2

def grad_f(x: np.ndarray) -> np.ndarray:
    return np.array([2 * (x[0] - 3), 2 * (x[1] - 5)])

# TODO derive this by hand
def project_equality(x: np.ndarray, A: np.ndarray, b: np.ndarray) -> np.ndarray:
    return x - A.T @ np.linalg.inv(A @ A.T) @ (A @ x - b)

# TODO derive this
def project_inequality(x: np.ndarray, A: np.ndarray, b: np.ndarray) -> np.ndarray:
    if x[1] >= 1: return x
    
    # constraint was violated
    return project_equality(x, A, b)

# gradient descent 
alpha = 0.1
x = np.array([0.0, 0.0])
for i in range(10):
    x = x - alpha * grad_f(x)
    x = project_equality(x, np.array([[1, 1]]), np.array([4]))
    x = project_inequality(x, np.array([[0, 1]]), np.array([1]))
    print(x, '\t', np.sum(x))
    
# oh yeah!