# gradient gF(x) = J^T r
# hessian g^2F(x) = J^T J + sum_{i=1}^m r_i g^2r_i
# gauss-newton approximation of Hessian: g^2F(x) ≈ J^T J
#
# -------      Newton's Method      -------
# Quadratic model of F: 
# m(p) = F(x) + p^T gF + 1/2 p^T H p
# F(x) is the scalar to shift the model to the current point
# p^T gF is the linear term that says how much the function changes due to the first-order
# 1/2 p^T H p is the quadratic term that says how much the gradient changes due to the second-order
#
# H = g^2F(x) = J^T J + sum_{i=1}^m r_i g^2r_i, which is expensive to compute
#
# -------    Gauss-Newton Method    -------
# At each iteration, GN finds a step p by solving the normal equations:
# (J^T J) p = -J^T r TODO what is a normal equation? (1)
# then x <- x + p
# 
# Converges quadratically near the solution
# J^T J must be invertible
# Can diverge if step is too large
# No trust region or damping - pure newton style step TODO what is a trust region?
#
# In short, we replace the expensive H with a cheap J^T J
#
# ------- Levenberg-Marquardt Method -------
# Augments the normal equations with a damping term: mu > 0
# (J^T J + mu I) p = -J^T r
# Damping term is also called a Levenberg factor
# mu = 0 -> pure newton step
# mu -> inf -> short gradient descent step
# mu is adjusted dynamically during the iteration
#
# LM can be derived by solving a trust region subproblem
# minimize the quadratic model of F
# m(p) = F(x) + p^T gF + 1/2 p^T (J^T J) p
# subject to ||p|| <= delta, where delta is the trust region radius
# mu is the lagrange multiplier enforcing this constraint TODO how?
#
# compute gain ratio
# gamma = (F(x) - F(x + p)) / (m(0) - m(p))
# if gamma is close to 1, then p is a good step
# if gamma is small or negative, then p is a bad step
# common update rule:
# on success: mu <- mu / 10
# on failure: mu <- mu * 10
#
# ----------------   TODO   ----------------
# (1) what is a normal equation?
# Given an overdetermined linear system Ax = b, we need to find x that minimizes the residual ||Ax - b||^2
# Expand the objective: F(x) = (Ax - b)^T (Ax - b)
# Set the gradient to 0: gF(x) = 2A^T (Ax - b) = 0 
# ^ find critical points, necessary but not sufficient for a minimum
# Solve for x: A^T Ax = A^T b <-- this is a normal equation
#
# (2) what is a trust region?
# A trust region is a ball around the current iterate x that are considered to be "trustworthy"
# Only search for a step within the ball
# If the prediction was good, we expand the ball
# If the prediction was bad, we contract the ball
#
# (3) what is a lagrange multiplier?
# We use mu to constrain the quadratic model to the trust region. 

# ------------------------------------------------
# Data generation
# ------------------------------------------------

import numpy as np
from exp_problem_setup import *

rng = np.random.default_rng(42)

a_true, b_true = 1.2, -0.8
t = np.linspace(0.5, 3.0, 20)
y = a_true * np.exp(b_true * t) + rng.normal(0, 0.02, size=len(t))

# ------------------------------------------------
# Newton's method
# ------------------------------------------------

def newton_step(t: np.ndarray, a: float, b: float, y: np.ndarray) -> tuple[float, float]:
    hessian = H(t, a, b, y)
    gF = grad_F(t, a, b, y)
    p = np.linalg.solve(hessian, -gF)
    return p

# solve newton
a, b = 1.0, -0.5
gF = grad_F(t, a, b, y)
for k in range(100):
    if np.linalg.norm(gF) < 1e-8:
        break
    p = newton_step(t, a, b, y)
    a += p[0]
    b += p[1]
    gF = grad_F(t, a, b, y)
print(f"Newton: a={a:.6f}, b={b:.6f}")

# ------------------------------------------------
# Gauss-Newton method
# ------------------------------------------------

def GN_H(t: np.ndarray, a: float, b: float, y: np.ndarray) -> np.ndarray: 
    return 2 * J(t, a, b, y).T @ J(t, a, b, y)

def GN_step(t: np.ndarray, a: float, b: float, y: np.ndarray) -> tuple[float, float]:
    hessian = GN_H(t, a, b, y)
    gF = grad_F(t, a, b, y)
    p = np.linalg.solve(hessian, -gF)
    return p

a, b = 1.0, -0.5
gF = grad_F(t, a, b, y)
for k in range(100):
    if np.linalg.norm(gF) < 1e-8:
        break
    p = GN_step(t, a, b, y)
    a += p[0]
    b += p[1]
    gF = grad_F(t, a, b, y)
print(f"Gauss-Newton: a={a:.6f}, b={b:.6f}")

# ------------------------------------------------
# Levenberg-Marquardt method
# ------------------------------------------------

v = 10.0
    
def LM_step(t: np.ndarray, a: float, b: float, y: np.ndarray, mu: float) -> tuple[tuple[float, float], float]:
    # compute the variable delta
    jacobian = J(t, a, b, y)
    residual = r(t, a, b, y)
    lhs = jacobian.T @ jacobian + mu * np.eye(jacobian.shape[1])
    rhs = -jacobian.T @ residual
    p = np.linalg.solve(lhs, rhs)
    
    # compute the predicted reduction
    predicted = p.T @ (2 * mu * p - 2 * jacobian.T @ residual)
    
    # compute gain ratio
    F_current = np.sum(residual**2) # F(x)
    F_new = np.sum(r(t, a + p[0], b + p[1], y)**2) # F(x + p)
    gamma = (F_current - F_new) / predicted
    
    if gamma > 0.75:
        mu = mu / v
    elif gamma < 0.25:
        mu = mu * v
        
    # accept the step
    if gamma > 0.0:
        return (p, mu) 
    
    # reject the step
    # return 0, 0 to keep current a & b, remember p is delta a & b
    else:
        return ([0, 0], mu) 

a, b = 1.0, -0.5
mu = 1.0
gF = grad_F(t, a, b, y)
for k in range(100):
    if np.linalg.norm(gF) < 1e-8:
        break
    p, mu = LM_step(t, a, b, y, mu)
    a += p[0]
    b += p[1]
    gF = grad_F(t, a, b, y)
print(f"Levenberg-Marquardt: a={a:.6f}, b={b:.6f}")