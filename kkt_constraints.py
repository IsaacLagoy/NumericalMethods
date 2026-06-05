# KKT coditions

# fundamental theory for constrained optimization
# what does a solution look like at the boundary of a feasible region?
# grad F can't just = 0, at constraint minimum, the gradient of f must be balanced by the gradient of the constraints

# There are four KKT conditions:

# take an example problem:
# min f(x) st g_i(x) <= 0, i = 1, ..., m and h_j(x) = 0, j = 1, ..., p
# Define the lagrangian:
# L(x, mu, lambda) = f(x) + sum_{i=1}^m lambda_i g_i(x) + sum_{j=1}^p mu_j h_j(x)
# at solution x^*, exists multipliers mu_i, lambda_i >= 0 st:

# 1. Stationary
# grad_x L = grad f(x^*) + sum_{i=1}^m lambda_i grad g_i(x^*) + sum_{j=1}^p mu_j grad h_j(x^*) = 0
# the gradient of f is balanced by the gradients of the constraints

# 2. Primal Feasibility
# g_i(x^*) <= 0, i = 1, ..., m
# h_j(x^*) = 0, j = 1, ..., p
# the solution must satisfy the constraints

# 3. Dual Feasibility
# mu_i >= 0, i = 1, ..., m
# the multipliers must be non-negative
# this is to enforce the geometric requirement that constraint gradients can only push against descent, not help it
# since h is not an inequality constraint, it doesn't have to be non-negative

# 4. Complementary Slackness
# mu_i * g_i(x^*) = 0, i = 1, ..., m
# for each inequality constraint, only one of the following can be true:
# (1) the multiplier is zero (constraint is inactive, doesn't affect the solution)
# (2) the constraint is active g_i = 0 (constraint is binding)
# they cannot both be non-zero simultaneously

# NOTE KKT conditions are only necessary, not sufficient for optimality
# KKT points can be local extrema or saddle points
# We need second-order conditions to distinguish between these
# we need Constraint Qualifications (CQ) to guarantee the multipliers exist

# Linear Independence CQ (LICQ)
# gradients of the active constraint at x^* are linearly independent
# this is the standard assumption and holds generally

# Slater's Condition
# For convex problems, if there exists a strictly feasible point (g_i(x) < 0, h_j(x) = 0), then KKT holds
# usually easy to verify

# if CQ fails, KKT multipliers may not exist

# The lagrangian
# L(x, mu, lambda) = f(x) + lambda^T g(x) + mu^T h(x)
# at the constrained minimum, the lagrangian is stationary with respect to x^* and the multipliers
# (1) grad_x L = 0
# (2) grad_mu L = h(x) = 0
# (3) grad_lambda L = g(x) <= 0
# finding constrained minima is equivalent to finding saddle points of the lagrangian (related to duality theory)

# note that when x^* is minimum, the only remaining direction is the normal to the constraint surface. 
# grad F must be parallel to grad h
# grad f(x^*) = -lambda grad h(x^*)
# this is equivalent to L(x, lambda) = f(x) + lambda * h(x)
# take both partial derivatives and set them to 0:
# (1) grad_x L = grad f(x) + lambda grad h(x) = 0 (stationary condition)
# (2) grad_lambda L = h(x) = 0 (dual feasibility)

# equality constraints - the solutions sits on the surface created by the plane
# grad f(x) = -lambda grad h(x)

# I completed the following problem:
# f(x, y) = (x - 3)^2 + (y - 2)^2
# h(x, y) = x + 2y - 4 = 0 <- equality constraint
# note lambda can be either positive or negative

# inequality constraints - two types
# (1) inactive g_i(x^*) < 0 (constraint is not binding)
# (2) active g_i(x^*) = 0 (constraint is binding)
# complementary slackness: mu_i * g_i(x^*) = 0 # only one can be 0
# if inactive, mu_i = 0
# if active, g_i(x^*) = 0
# note lambda must be non-negative for inequality constraints
# grad f must be expressible as a non-negative combination of the constraint gradients
# grad f(x^*) = - sum_{i=1}^m mu_i grad g_i(x^*)

# Full KKT conditions for inequalities:
# (stationary) grad f(x^*) + sum_{i=1}^m mu_i grad g_i(x^*) = 0
# (primal feasibility) g_i(x^*) <= 0
# (dual feasibility) mu_i >= 0
# (complementary slackness) mu_i * g_i(x^*) = 0
# trust regions ||p|| <= delta can be formulated as constraints: mu(||p||^2 - delta^2) <= 0

# I completed the following problem:
# f(x, y) = (x - 2)^2 + (y - 3)^2
# g(x, y) = x^2 + y^2 - 4 <= 0 <- inequality constraint

# KKT conditions are necessary but not sufficient for optimality in the non-convex case
# We need some way of distinguishing maxima and saddle points from minima

