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
# grad^2 f(x^*) > 0 -> local minimum
# grad^2 f(x^*) < 0 -> local maximum
# grad^2 f(x^*) is indefinite -> saddle point
# at the constrained minimum x^*, we only need H to be pos definite on the tangent space of the active constraints
# T(x^*) = {v : grad h^j(x^*)^T v = 0 \forall j, grad g_i(x^*)^T v = 0 \forall i active}

# if x* is a local min and KKT holds, then the Lagrangian Hessian is positive semidefinite on the tangent space
# v^T grad^2_{xx} L(x^*, lambda^*, mu^*) v >= 0 \forall v in T(x^*)
# if KKT holds at x^* and the Lagrangian Hessian is strictly positive definite on the tangent space: 
# TODO what is the tangent space? what is positive semidefinite?
# v^T grad^2_{xx} L(x^*, lambda^*, mu^*) v > 0 \forall v in T(x^*)
# then x^* is a local minimum

# bordered hessian
# structure matrix that encodes the objective curvature and the constraint gradients
# \bar H = [0        grad h^T     ]
#          [grad h   grad^2_{xx} L]
# TODO how is this handled with multiple constraints?

# the dual function is always concave
# the dual always underpredicts the primal
# dual gap is p* - d*
# strong duality: p* = d*

# primal vs dual
# primal - minimize f over x (search over original space)
# fual - maximize over mu, lambda, etc (search over the multiplier space)
# they solve the same problem but from different perspectives
# the dual problem's solution is a lower bound on the primal problem's solution

# duality allows us to solve many primal constraints at the same time
# this is most beneficial for inequality constraints, duality does no simplify things for equality constraints
# in concavity, weak duality always holds but strong duality usually fails. This means that there is a duality gap and duality is not the infinum of the primal.
# tight non-convex problems have strong duality, TODO look into this more (active research area)
# feasible x with f(x) = d(mu, lambda) is provably globally optimal

# dual problem:
# minimize f(x, y) = (x - 4)^2 + (y - 4)^2
# with constraints
# g_1(x, y) = x + y - 4 <= 0
# g_2(x, y) = -x <= 0
# g_3(x, y) = -y <= 0

# construct the lagrangian
# L(x, y, mu) = (x - 4)^2 + (y - 4)^2 + mu_1 (x + y - 4) + mu_2 (-x) + mu_3 (-y)

# dual function - minimize over x and y
# dL/dx = 2(x - 4) + mu_1 - mu_2 = 0
# dL/dy = 2(y - 4) + mu_1 - mu_3 = 0

# 2x - 8 + mu_1 - mu_2 = 0
# 2x = 8 - mu_1 + mu_2
# x = (8 - mu_1 + mu_2) / 2

# 2y - 8 + mu_1 - mu_3 = 0
# 2y = 8 - mu_1 + mu_3
# y = (8 - mu_1 + mu_3) / 2

# substitute back into the lagrangian
# d(mu) = ( (8 - mu_1 + mu_2) / 2 )^2 + ( (8 - mu_1 + mu_3) / 2 )^2 + mu_1 ( (8 - mu_1 + mu_2) / 2 + (8 - mu_1 + mu_3) / 2 - 4) + mu_2 (- (8 - mu_1 + mu_2) / 2 ) + mu_3 (- (8 - mu_1 + mu_3) / 2 )

# simplify
# d(mu) = (8 - mu_1 + mu_2)^2 / 4 + (8 - mu_1 + mu_3)^2 / 4 + (mu_1 / 2) * (8 - 2 * mu_1 + mu_2 + mu_3) + (mu_2 / 2) * (mu_1 - 8 - mu_2) + (mu_3 / 2) * (mu_1 - 8 - mu_3)

# take the derivative
# dd/dmu_1 = -(8 - mu_1 + mu_2) - (8 - mu_1 + mu_3) + 4 = 0
# dd/dmu_2 = -(8 - mu_1 + mu_2) / 2 = 0
# dd/dmu_3 = -(8 - mu_1 + mu_3) / 2 = 0

# solve for mu_1, mu_2, mu_3
# 8 - mu_1 + mu_2 = 0
# mu_1 = 8 + mu_2
# 8 - mu_1 + mu_3 = 0
# mu_1 = 8 + mu_3
# mu_2 = mu_3
# -8 + mu_1 - mu_2 - 8 + mu_1 - mu_2 + 4 = 0
# mu_1 - mu_2 = 6
# mu_1 = 6 + mu_2
# mu_2 - mu_1 = -8
# mu_2 - (6 + mu_2) = -8
# 2 * mu_2 = -2
# mu_2 = -1 = mu_3 # but mu >= 0 so the maximum is on the boundary
# remember, mu >= 0 since we can only push inward or sit on the boundary
# 0 - mu_1 = -8
# mu_1 = 8

# plug back into x and y
# x = (8 - 8 + 0) / 2 = 0
# y = (8 - 8 + 0) / 2 = 0

# somewhere I messed up lowk
# mu_1 = 4
# (x*, y*) = (2, 2)
# d* = 4(4) - 16/2 = 8
# p* = f(2, 2) = 8
# duality gap = p* - d* = 0
# strong duality holds