# trust regions

# define a closed ball around the current iterate
# only allow steps within the ball
# clip any step that occurs outside the ball

# use the gamma disciminator to evaluate how well the model did
# shrink the ball is we did poorly

# min m_k(p) = f_k + grad(f_k)^T p + 1/2 p^T H_k p
# note that H_k can be an approximation of the Hessian

# where does the unconstrained minimizer land? 

# Case 1: Lands inside the ball ||p^*|| <= delta_k
# use the minimizer as is

# Case 2: H_k is indefinite (like a saddle)
# this means there is no interior minimum in the region
# the solution must lie on the boundary

# Case 3: Lands outside the ball ||p^*|| > delta_k
# The solution can be found on the boundary
# we must constrain the minimizer

# ------------------------------------------------
# Cauchy Point
# ------------------------------------------------

# cheap and simple 

import numpy as np
from exp_problem_setup import *
from gauss_newton import newton_step

rng = np.random.default_rng(42)

a_true, b_true = 1.2, -0.8
t = np.linspace(0.5, 3.0, 20)
y = a_true * np.exp(b_true * t) + rng.normal(0, 0.02, size=len(t))

delta = 1.0
delta_max = 10.0
eta = 0.1 # TODO what is eta?

def cauchy_point(t: np.ndarray, a: float, b: float, y: np.ndarray, delta: float) -> np.ndarray:
    # get unconstrained minimizer
    grad_f = grad_F(t, a, b, y)
    hess = H(t, a, b, y) # can be replaced with J^T J
    
    gTg = grad_f.T @ grad_f
    gTHg = grad_f.T @ hess @ grad_f
    
    # indefinite/negative curvature — skip to boundary
    if gTHg <= 0:
        return -delta * grad_f / np.linalg.norm(grad_f)
    
    alpha_SD = gTg / gTHg
    
    p_SD = -alpha_SD * grad_f
    
    # clip the minimizer to the trust region
    if np.linalg.norm(p_SD) <= delta:
        return p_SD
    
    return -delta * grad_f / np.linalg.norm(grad_f)

def compute_rho(t: np.ndarray, a: float, b: float, y: np.ndarray, delta: float, p_C: np.ndarray) -> float:
    # compute predicted reduction
    # m_k(0) - m_k(p^C)
    grad_f = grad_F(t, a, b, y)
    hess = H(t, a, b, y)
    m0_mc = -grad_f.T @ p_C - 1/2 * p_C.T @ hess @ p_C
    
    # get gamma ratio
    f = F(t, a, b, y)
    f_C = F(t, a + p_C[0], b + p_C[1], y)
    rho = (f - f_C) / m0_mc
    return rho

def update_delta(delta: float, rho: float, p_C: np.ndarray, delta_max: float) -> float:
    if rho < 0.25:
        return delta * 0.25
    
    # expand only if we hit the boundary
    elif rho > 0.75 and abs(np.linalg.norm(p_C) - delta) < 1e-6: # and |p_C| == delta accept its epsilon eq
        return min(delta * 2, delta_max)
    
    return delta

# full cauchy loop
a, b = 1.0, -0.5
k_accepted = 0
for k in range(200):
    grad_f = grad_F(t, a, b, y)
    hess = H(t, a, b, y) # can be replaced with J^T J
    
    if np.linalg.norm(grad_f) < 1e-6:
        break
    
    p = cauchy_point(t, a, b, y, delta)
    
    rho = compute_rho(t, a, b, y, delta, p)
    
    delta = update_delta(delta, rho, p, delta_max)
    
    if rho > eta:
        a += p[0]
        b += p[1]
        k_accepted += 1
        
    # regular timeout at 100 iterations like everything else
    if k_accepted > 100:
        break
    
print(f"a: {a}, b: {b}, k: {k}, k_accepted: {k_accepted}")

# ------------------------------------------------
# Dogleg Method
# ------------------------------------------------

# Combines Cauchy with a Newton's step
def dogleg(t: np.ndarray, a: float, b: float, y: np.ndarray, delta: float) -> np.ndarray:
    p_B = newton_step(t, a, b, y)
    p_C = cauchy_point(t, a, b, y, delta)
    grad_f = grad_F(t, a, b, y)
    
    # if newton's step fits, use it
    if np.linalg.norm(p_B) <= delta:
        return p_B
    
    # cauchy is outside so clip to boundary
    if np.linalg.norm(p_C) >= delta:
        return -delta * grad_f / np.linalg.norm(grad_f)
    
    # otherwise, find tau where dogleg path hits the boundary
    # check tau in [1, 2]
    # solve ||p_C + s*(p_B - p_C)||^2 = delta^2 for s in [0, 1], the quadratic
    v = p_B - p_C
    a_coef  = v.T @ v
    b_coef = 2 * (p_C.T @ v)
    c_coef = p_C.T @ p_C - delta ** 2
    
    s = (-b_coef + np.sqrt(b_coef ** 2 - 4 * a_coef * c_coef)) / (2 * a_coef)
    return p_C + s * v

a, b = 1.0, -0.5
k_accepted = 0
for k in range(200):
    grad_f = grad_F(t, a, b, y)
    hess = H(t, a, b, y) # can be replaced with J^T J
    
    if np.linalg.norm(grad_f) < 1e-6:
        break
    
    p = dogleg(t, a, b, y, delta)
    
    rho = compute_rho(t, a, b, y, delta, p)
    
    delta = update_delta(delta, rho, p, delta_max)
    
    if rho > eta:
        a += p[0]
        b += p[1]
        k_accepted += 1
        
    # regular timeout at 100 iterations like everything else
    if k_accepted > 100:
        break
    
print(f"a: {a}, b: {b}, k: {k}, k_accepted: {k_accepted}")

# ------------------------------------------------
# Secular Equation
# ------------------------------------------------

# finds the exact solution
# this can also handle indefinite Hessians correctly without muI regularization

# TODO come back to finish this