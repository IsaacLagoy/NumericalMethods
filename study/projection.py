# projection methods

# core idea:
# 1) take a full step
# 2) project the step onto the feasible set

# the "projection" of a point x onto a set C
# P_C(x) = argmin_{y in C} ||x - y||

# the update rule is simply
# x_{k+1} = P_C(x_k + step_k)
