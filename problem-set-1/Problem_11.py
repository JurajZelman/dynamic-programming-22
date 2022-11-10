"""Copyright (c) 2022 Juraj Zelman"""

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

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

# Save results to pandas DataFrame
row_labels = [f"x_{i}" for i in range(n_states)]
column_labels = [f"k = {i}" for i in range(n_timesteps)]
df_controls = pd.DataFrame(u_opt, index=row_labels, columns=column_labels)
column_labels.append(f"k = {n_timesteps}")
df_cost = pd.DataFrame(J_opt, index=row_labels, columns=column_labels)

# Print the results
print("Optimal final cost")
print(df_cost)
print()
print("Optimal controls")
print(df_controls)

# Plot the optimal policy (for w_k = 1)
p, q = np.meshgrid(np.arange(n_timesteps + 1), x)
plt.figure(figsize=(7, 4))
plt.scatter(p, q, marker="o")

# Plot the optimal transitions
x_plus_u = x + np.transpose(u_opt[:, 1])
for i in range(n_timesteps):
    x_plus_u = x + np.transpose(u_opt[:, i])
    plt.plot([i, i + 1], [x, x_plus_u], "k")

# Plot x_ref(k)
k = np.arange(n_timesteps + 1)
plt.plot(k, np.power(k - 5, 2), "r")

# Format the plot
plt.xlabel("Time step k")
plt.ylabel("State x_k")
plt.xlim((-1, n_timesteps + 1))
plt.ylim((x[0] - 1, x[-1] + 1))

p1 = mpatches.Patch(color="red", label="x_ref(k)")
p2 = mpatches.Patch(color="black", label="x_k+1 = x_k + u_k")
plt.legend(handles=[p1, p2], loc="upper left", bbox_to_anchor=(1.04, 1))
plt.tight_layout()

plt.show()
