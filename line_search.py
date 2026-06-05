# Line search methods
#
# Do some searching along the step direction to find the optimal step size.
# These methods wrap around any descent method (Newton, GN, LM, etc.) and
# govern only the step length alpha, not the direction p.

import numpy as np
from exp_problem_setup import *
from gauss_newton import newton_step

rng = np.random.default_rng(42)

a_true, b_true = 1.2, -0.8
t = np.linspace(0.5, 3.0, 20)
y = a_true * np.exp(b_true * t) + rng.normal(0, 0.02, size=len(t))

# ------------------------------------------------
# Shared helpers
# ------------------------------------------------

def eval_at_step(t, a, b, y, p, alpha):
    """Evaluate F and the directional derivative at x + alpha * p."""
    a_new = a + alpha * p[0]
    b_new = b + alpha * p[1]
    F_new = F(t, a_new, b_new, y)
    slope_new = grad_F(t, a_new, b_new, y) @ p
    return F_new, slope_new

def armijo_satisfied(F_new, F_old, alpha, slope, c1):
    """Check the Armijo sufficient-decrease condition.
    
    f(x + alpha * p) <= f(x) + c1 * alpha * grad_f(x)^T p
    
    Rejects a step unless the function decreases by at least a margin
    proportional to the step size and the local slope.
    Note: slope is always negative for a descent direction, so the
    right-hand side is strictly less than F_old.
    """
    return F_new <= F_old + c1 * alpha * slope

def curvature_satisfied(slope_new, slope, c2):
    """Check the Wolfe curvature condition.
    
    grad_f(x + alpha*p)^T p >= c2 * grad_f(x)^T p
    
    Ensures alpha is not too small: the slope at the new point must be
    less negative than at the start, meaning we've walked far enough that
    the function is starting to flatten out.
    """
    return slope_new >= c2 * slope

# ------------------------------------------------
# The Armijo condition
# ------------------------------------------------

# f(x_k + alpha * p_k) <= f(x_k) + c1 * alpha * gF_k^T p_k
# where c1 is a constant between 0 and 1, typically 1.0e-4
# p_k is the direction of the step, alpha is the step size

def get_armijo_alpha(t, a, b, y, p, F_old, slope, c1, gamma):
    """Find alpha satisfying the Armijo condition via backtracking.
    
    Start at alpha=1 (the full step) and shrink by gamma until the
    sufficient-decrease condition is met.
    """
    alpha = 1.0
    for _ in range(50):
        F_new, _ = eval_at_step(t, a, b, y, p, alpha)
        if armijo_satisfied(F_new, F_old, alpha, slope, c1):
            break
        alpha *= gamma
    else:
        print("WARNING: Armijo line search did not converge")
    return alpha

# ------------------------------------------------
# The Wolfe conditions
# ------------------------------------------------

# Builds upon Armijo by adding a curvature condition to also prevent
# steps that are too small:
#
#   grad_f(x + alpha*p)^T p >= c2 * grad_f(x)^T p
#
# where c2 is a constant between c1 and 1, typically 0.9.
# We want to ensure that the slope at the new point starts to flatten out.
#
# Implemented via bracket-and-zoom:
#   Phase 1 — Bracket: find [alpha_lo, alpha_hi] guaranteed to contain a valid alpha
#   Phase 2 — Zoom:    cubic-interpolate the bracket down to the valid alpha

def cubic_interpolation(alpha_lo, alpha_hi, F_lo, F_hi, slope_lo, slope_hi):
    """Find the minimum of the cubic interpolant through the bracket endpoints.
    
    Uses both function values and directional derivatives at each end,
    converging in ~2-3 iterations vs ~10-15 for bisection.
    """
    d = slope_lo + slope_hi - 3 * (F_hi - F_lo) / (alpha_hi - alpha_lo)
    beta = np.sqrt(d ** 2 - slope_lo * slope_hi)
    return alpha_hi - (alpha_hi - alpha_lo) * (slope_hi + beta - d) / (slope_hi - slope_lo + 2 * beta)

def _bracket_phase(t, a, b, y, p, F_old, slope, c1):
    """Phase 1: walk forward from alpha=0, growing alpha until we can bracket a valid point.
    
    Stop when either:
      (1) Armijo is violated — we've overshot, valid alpha is behind us
      (2) Slope is non-negative — function is going back uphill, minimum is behind us
    
    Returns (alpha_lo, alpha_hi) if a bracket is found, or (None, alpha) if
    the curvature condition was already satisfied (no zoom needed).
    """
    alpha_prev, alpha = 0.0, 1.0
    F_prev = F_old

    for i in range(100):
        F_new, slope_new = eval_at_step(t, a, b, y, p, alpha)

        # armijo violated or function increased — valid alpha is in [alpha_prev, alpha]
        if not armijo_satisfied(F_new, F_old, alpha, slope, c1) or (i > 0 and F_new > F_prev):
            return alpha_prev, alpha

        # both conditions satisfied — no zoom needed
        if curvature_satisfied(slope_new, slope, c2=0.9):
            return None, alpha

        # slope went non-negative — minimum is behind us, bracket [alpha_prev, alpha]
        if slope_new >= 0:
            return alpha_prev, alpha

        F_prev = F_new
        alpha_prev = alpha
        alpha = min(2 * alpha, 1.0) # 1.0 is alpha_max

    return None, alpha

def _zoom_phase(t, a, b, y, p, F_old, slope, alpha_lo, alpha_hi, c1, c2):
    """Phase 2: cubic-interpolate within [alpha_lo, alpha_hi] to find a valid alpha.
    
    The bracket is guaranteed to contain a valid alpha. Shrink it by updating
    whichever endpoint the trial point replaces, until both Wolfe conditions hold.
    """
    F_lo, slope_lo = eval_at_step(t, a, b, y, p, alpha_lo)
    F_hi, slope_hi = eval_at_step(t, a, b, y, p, alpha_hi)

    for _ in range(100):
        alpha_trial = cubic_interpolation(alpha_lo, alpha_hi, F_lo, F_hi, slope_lo, slope_hi)
        F_trial, slope_trial = eval_at_step(t, a, b, y, p, alpha_trial)

        if not armijo_satisfied(F_trial, F_old, alpha_trial, slope, c1):
            # trial overshot - shrink upper bound
            alpha_hi, F_hi, slope_hi = alpha_trial, F_trial, slope_trial
            continue

        if curvature_satisfied(slope_trial, slope, c2):
            return alpha_trial  # both conditions satisfied

        # armijo ok but curvature not — raise lower bound
        alpha_lo, F_lo, slope_lo = alpha_trial, F_trial, slope_trial

    # fallback if zoom didn't fully converge
    return alpha_lo

def get_wolfe_alpha(t, a, b, y, p, F_old, slope, c1, c2):
    """Find alpha satisfying the strong Wolfe conditions via bracket-and-zoom."""
    alpha_lo, alpha_or_hi = _bracket_phase(t, a, b, y, p, F_old, slope, c1)

    if alpha_lo is None:
        return alpha_or_hi  # bracket phase already found a valid alpha

    return _zoom_phase(t, a, b, y, p, F_old, slope, alpha_lo, alpha_or_hi, c1, c2)

# ------------------------------------------------
# Shared outer solver loop
# ------------------------------------------------

def solve(t, y, a_init, b_init, get_alpha, label, **ls_kwargs):
    """Newton solver with a pluggable line search.
    
    get_alpha(t, a, b, y, p, F_old, slope, **ls_kwargs) -> float
    """
    a, b = a_init, b_init
    gF = grad_F(t, a, b, y)

    for k in range(100):
        if np.linalg.norm(gF) < 1e-8:
            break

        p = newton_step(t, a, b, y)
        F_old = F(t, a, b, y)
        
        # always negative for a valid descent direction
        slope = gF @ p

        alpha = get_alpha(t, a, b, y, p, F_old, slope, **ls_kwargs)

        a += alpha * p[0]
        b += alpha * p[1]
        gF = grad_F(t, a, b, y)

        print(f"{label}: k={k}, alpha={alpha:.6f}, a={a:.6f}, b={b:.6f}")

    print(f"{label}: a={a:.6f}, b={b:.6f}\n")
    return a, b

# ------------------------------------------------
# Run both solvers
# ------------------------------------------------

c1 = 1.0e-4
c2 = 0.9
gamma = 0.5

solve(t, y, a_init=0.4, b_init=-0.5,
      get_alpha=get_armijo_alpha,
      label="Armijo",
      c1=c1, gamma=gamma)

solve(t, y, a_init=0.4, b_init=-0.5,
      get_alpha=get_wolfe_alpha,
      label="Wolfe",
      c1=c1, c2=c2)