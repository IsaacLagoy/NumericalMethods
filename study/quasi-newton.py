# quasi-newton

# another methods that uses an approximation of the Hessian
# store last step taken and change in gradient
# the true hessian satisfies H_sk ~= y_k (secant condition)
# we call the hessian approximation B_k
# run regular newton's with B_k instead of H_k
# TODO is y_k = grad f_{k+1} - grad f_k a matrix? 
# wouldn't it be a vector?

# BFGS (Broyden-Fletcher-Goldfarb-Shanno)
# update rule for B_k preserves symmetry and positive definiteness
# B_{k+1} = B_k - (B_k s_k s_k^T B_k) / (s_k^T B_k s_k) + (y_k y_k^T) / (y_k^T s_k)
# (two rank 1 corrections) subtract the outdated curvature and add the new observed curvature
# -> (rank 2 update)
# in practice, we store the inverse hessian instead using the Sherman-Morrison-Woodbury formula
# search direction d_k = -H_k grad f_k

# L-BFGS (Limited-memory BFGS)
# used for when storing the full n x n B_k is too expensive
# this method never forms B_k at all
# store the last m updates of s_k and y_k (typically 5 to 20)
# compute H_k grad f implicitly from the pairs
# storage becomes O(mn) instead of O(n^2)
# this is the standard optimizer for ML and large scale scientific computing
