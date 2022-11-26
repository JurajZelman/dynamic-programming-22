"""Copyright (c) 2022 Juraj Zelman"""

import matplotlib.pyplot as plt
import numpy as np


def main():
    # Set parameters
    p, q = np.zeros(2), np.zeros(2)
    p[1] = 0.95  # Probability of landing in bounds (p_S)
    q[0] = 0.6  # Probability of winning (q_F)
    q[1] = 0.4  # Probability of winning (q_S)
    tol = 1e-100  # Tolerance for convergence of the value function

    p_f = np.linspace(0, 1, 21)
    p_win = np.zeros(p_f.shape)
    S_dim = [4, 4, 2]  # Dimensions of the state space

    for p_f_iter in p_f:
        p[0] = p_f_iter  # Probability of landing in bounds (p_F)

        # Initialize costs, optimal control policy and value function
        J, U, costToGo = np.ones(S_dim), np.zeros(S_dim), np.zeros(S_dim)

        # Value iteration derived from the Bellman equation
        counter = 0
        while True:
            counter += 1

            for i in range(3):
                costToGo[3, i, 0] = np.max(
                    q * p + (1 - q) * p * J[3, i + 1, 0] + (1 - p) * J[3, i, 1]
                )
                U[3, i, 0] = np.argmax(
                    q * p + (1 - q) * p * J[3, i + 1, 0] + (1 - p) * J[3, i, 1]
                )
                costToGo[3, i, 1] = np.max(
                    q * p + (1 - q) * p * J[3, i + 1, 0]
                )
                U[3, i, 1] = np.argmax(q * p + (1 - q) * p * J[3, i + 1, 0])
                costToGo[i, 3, 0] = np.max(
                    q * p * J[i + 1, 3, 0] + (1 - p) * J[i, 3, 1]
                )
                U[i, 3, 0] = np.argmax(
                    q * p * J[i + 1, 3, 0] + (1 - p) * J[i, 3, 1]
                )
                costToGo[i, 3, 1] = np.max(q * p * J[i + 1, 3, 0])
                U[i, 3, 1] = np.argmax(q * p * J[i + 1, 3, 0])
                for j in range(3):
                    costToGo[i, j, 0] = np.max(
                        q * p * J[i + 1, j, 0]
                        + (1 - q) * p * J[i, j + 1, 0]
                        + (1 - p) * J[i, j, 1]
                    )
                    U[i, j, 0] = np.argmax(
                        q * p * J[i + 1, j, 0]
                        + (1 - q) * p * J[i, j + 1, 0]
                        + (1 - p) * J[i, j, 1]
                    )
                    costToGo[i, j, 1] = np.max(
                        q * p * J[i + 1, j, 0] + (1 - p * q) * J[i, j + 1, 0]
                    )
                    U[i, j, 1] = np.argmax(
                        q * p * J[i + 1, j, 0] + (1 - p * q) * J[i, j + 1, 0]
                    )
                costToGo[3, 3, 0] = np.max(
                    q * p * J[3, 2, 0]
                    + (1 - q) * p * J[2, 3, 0]
                    + (1 - p) * J[3, 3, 1]
                )
                U[3, 3, 0] = np.argmax(
                    q * p * J[3, 2, 0]
                    + (1 - q) * p * J[2, 3, 0]
                    + (1 - p) * J[3, 3, 1]
                )
                costToGo[3, 3, 1] = np.max(
                    q * p * J[3, 2, 0] + (1 - p * q) * J[2, 3, 0]
                )
                U[3, 3, 1] = np.argmax(
                    q * p * J[3, 2, 0] + (1 - p * q) * J[2, 3, 0]
                )

            # Check for convergence
            if np.max(np.abs(J - costToGo) / np.max(np.abs(costToGo))) < tol:
                J = np.copy(costToGo)
                break
            else:
                J = np.copy(costToGo)

        # Save and print results
        p_win[p_f_iter == p_f] = J[0, 0, 0]
        print(f"Terminated after {counter} iterations.")
        print(f"   p_f = {p_f_iter:.2f}, p_win = {J[0, 0, 0]:.2f}")

    # Plot the results
    plt.rcParams["text.usetex"] = True
    plt.plot(p_f, p_win, linestyle="-", marker="*", color="b")
    plt.title("Probability of winning a game by the person serving")
    plt.xlabel(r"$p_F$")
    plt.ylabel("Probability of winning")
    plt.show()


if __name__ == "__main__":
    main()
