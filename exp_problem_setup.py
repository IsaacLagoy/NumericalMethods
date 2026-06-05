import numpy as np

# residual function
def r(t: np.ndarray, a: float, b: float, y: np.ndarray) -> np.ndarray: 
    return y - a * np.exp(b * t)

def F(t: np.ndarray, a: float, b: float, y: np.ndarray) -> float:
    return np.sum(r(t, a, b, y) ** 2)

# residual first order
def dr_da(t: np.ndarray, a: float, b: float, y: np.ndarray) -> np.ndarray: 
    return -np.exp(b * t)

def dr_db(t: np.ndarray, a: float, b: float, y: np.ndarray) -> np.ndarray: 
    return -a * t * np.exp(b * t)

# residual second order
def d2r_da2(t: np.ndarray, a: float, b: float, y: np.ndarray) -> np.ndarray: 
    return np.zeros_like(t)

def d2r_db2(t: np.ndarray, a: float, b: float, y: np.ndarray) -> np.ndarray: 
    return -a * t ** 2 * np.exp(b * t)

def d2r_dadb(t: np.ndarray, a: float, b: float, y: np.ndarray) -> np.ndarray: 
    return -t * np.exp(b * t)

# jacobian
def J(t: np.ndarray, a: float, b: float, y: np.ndarray) -> np.ndarray: 
    return np.column_stack([
        dr_da(t, a, b, y),
        dr_db(t, a, b, y)
    ])
    
# hessian
def H(t: np.ndarray, a: float, b: float, y: np.ndarray) -> np.ndarray: 
    result = 2 * J(t, a, b, y).T @ J(t, a, b, y)
    for i in range(len(t)):
        grad2r = np.array([
            [d2r_da2(t[i], a, b, y[i]), d2r_dadb(t[i], a, b, y[i])],
            [d2r_dadb(t[i], a, b, y[i]), d2r_db2(t[i], a, b, y[i])]
        ])
        result += 2 * r(t[i], a, b, y[i]) * grad2r
    return result

def grad_F(t: np.ndarray, a: float, b: float, y: np.ndarray) -> np.ndarray: 
    return 2 * J(t, a, b, y).T @ r(t, a, b, y)