"""Copyright (c) 2022 Juraj Zelman"""

import numpy as np

p_w = 1 / 3  # Probability of the disturbance w being 1
n_timesteps = 10  # Number of timesteps
n_states = 11  # Number of states
x = np.arange(0, n_states)  # State vector

J_opt = np.zeros((n_states, n_timesteps + 1))  # Optimal cost-to-go matrix
u_opt = np.zeros((n_states, n_timesteps))  # Optimal controls matrix
J_opt[:, -1] = np.power(x, 2)  # Terminal cost

for k in range(n_timesteps - 1, -1, -1):
    for state in range(n_states):
        xk = x[state]  # Current state value
        Uk = np.arange(-xk, 10 - xk + 1)  # Set of admissible controls

        # Compute cost-to-go for all possible controls
        cost_to_go = np.zeros(Uk.shape)
        for m in range(Uk.shape[0]):
            uk = Uk[m]

            x_ref = (k - 5) ** 2
            if (xk + uk in x) and (xk in x):
                cost_to_go[m] = (
                    (xk - x_ref) ** 2
                    + uk ** 2
                    + p_w * J_opt[xk + uk, k + 1]
                    + (1 - p_w) * J_opt[xk, k + 1]
                )

        # Find optimal cost-to-go and control
        J_opt[state, k] = np.min(cost_to_go)
        u_opt[state, k] = Uk[np.argmin(cost_to_go)]
