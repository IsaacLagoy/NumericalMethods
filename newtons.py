# approximate f(x) = 0, only finds one solution
# requires function to be differentiable
# x_{n+1} = x_n - f(x_n) / f'(x_n)
# works by IVT
# if f'(x) = 0 then we get division by 0
# second-order optimization
# iterative nonlinear solving
# good local solving, bad global solving
# newton's method can take steps that are too aggressive

# gradient says "Which direction is uphill?"
# hessian says "How does the slope itself change?"

# ------------------------------------------------
# key idea:

# gradient - derivative of a scalar valued function
# jacobian - derivative of a vector valued function

# hessian - derivative of a gradient
# third-order tensor - derivative of a jacobian
# ------------------------------------------------

import numpy as np

# ------------------------------------------------
# 1D Nonlinear
# ------------------------------------------------

def f(x: float)  -> float: return x ** 3 - 4 * x ** 2 + 1
def df(x: float) -> float: return 3 * x ** 2 - 8 * x

def getNewtonStep(x: float) -> float: return x - f(x) / df(x)
def solve(x0: float, iter: int) -> float:
    x = x0
    for _ in range(iter):
        print(x)
        x = getNewtonStep(x)
    return x

def solveR(x0: float, iter: int) -> float: 
    if iter == 0: return x0
    return solveR(getNewtonStep(x0), iter - 1)

print(solve(0.5, 10))
print(solveR(0.5, 10))

# ------------------------------------------------
# 2D Nonlinear (Similar structure to nD)
# ------------------------------------------------

# x_{n+1} = x_n - J(x_n)^{-1} f(x_n)
# Intersection of a parabola and a circle

def f1(x: float, y: float) -> float: return x ** 2 + y ** 2 - 4
def f2(x: float, y: float) -> float: return x ** 2 - y - 1

def df1_dx(x: float, y: float) -> float: return 2 * x
def df1_dy(x: float, y: float) -> float: return 2 * y

def df2_dx(x: float, y: float) -> float: return 2 * x
def df2_dy(x: float, y: float) -> float: return -2 * y

# define the vector function F(x)
def F(x: np.ndarray) -> np.ndarray: return np.array([
    f1(x[0], x[1]), 
    f2(x[0], x[1])
])

# define the Jacobian matrix J(x)
def J(x: np.ndarray) -> np.ndarray: return np.array([
    [ df1_dx(x[0], x[1]), df1_dy(x[0], x[1]) ],
    [ df2_dx(x[0], x[1]), df2_dy(x[0], x[1]) ]
])

def getNewtonStep(x: np.ndarray) -> np.ndarray: return x - np.linalg.inv(J(x)) @ F(x)
def solve(x0: np.ndarray, iter: int) -> np.ndarray:
    x = x0
    for _ in range(iter):
        print(x)
        x = getNewtonStep(x)
    return x

print(solve(np.array([1, 1]), 10))