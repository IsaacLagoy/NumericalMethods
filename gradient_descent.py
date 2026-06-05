# A^T A x = A^T b

# TODO positive definite matrix

# ------------------------------------------------
# Key idea:
# non-linear literals r(x)
# we need to minimize ||r(x)||^2
# r(x + delta x) = r(x) + J(x) dx
# substitute into least squares ||r(x) + J(x) delta x||^2
# J^T J delta x = -J^T r(x)
# Since gradient descent uses the gradient, we need to define a scalar valued cost function to take the gradient of
# ------------------------------------------------

# TODO look into inverse kinematics

import numpy as np
import matplotlib.pyplot as plt

# ------------------------------------------------
# Linear Regression
# ------------------------------------------------

# generate dataset - note there is also a closed form solution for this
rng = np.random.default_rng()
X = rng.uniform(low=0.0, high=10.0, size=100)
true_slope = 2.0
true_intercept = 5.0
noise = rng.normal(loc=0.0, scale=1.0, size=100)
y = true_slope * X + true_intercept + noise

# perform gradient descent
step_size = 0.01
m = 0.0
b = 0.0

# ------------------------------------------------
# residual
# r = y - (m * X + b)
# r^2 (least squares) = (y - mx - b)^2

# cost function <- we want to minimize this
# J(m, b) = 1/n * sum(r^2) = 
# 1/n * sum_{i=1}^n (y_i - (m * x_i + b))^2 = 
# 1/n * sum_{i=1}^n (y_i - mx_i - b)^2
# ------------------------------------------------

for _ in range(1000):
    r = y - (m * X + b)
    dJ_dm = -2 / X.shape[0] * X.T @ r # X.T @ r = sum(X * r)
    dJ_db = -2 / X.shape[0] * np.sum(r)
    m = m - step_size * dJ_dm
    b = b - step_size * dJ_db

# plot
plt.plot(X, y, 'o')
plt.plot(X, m * X + b)
plt.scatter(X, y)
plt.show()

# ------------------------------------------------
# Multivariate Linear Regression (2 features, 2 outputs)
# ------------------------------------------------

# I derived these equations by hand :)

X = rng.uniform(low=0.0, high=10.0, size=(100, 2))
true_slope = np.array([
    [2.0, 3.0],
    [4.0, 5.0]
])
true_intercept = np.array([5.0, 10.0])
noise = rng.normal(loc=0.0, scale=1.0, size=(100, 2))
y = X @ true_slope.T + true_intercept + noise

# perform gradient descent
step_size = 0.01
W = np.array([
    [0.0, 0.0],
    [0.0, 0.0]
])
b = np.array([0.0, 0.0])

for _ in range(1000):
    r = y - X @ W.T - b
    dJ_dW = -2 / X.shape[0] * r.T @ X
    dJ_db = -2 / X.shape[0] * r.sum(axis=0)
    W = W - step_size * dJ_dW
    b = b - step_size * dJ_db

print(W, b, sep='\n')