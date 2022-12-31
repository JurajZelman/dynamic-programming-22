"""Copyright (c) 2022 Juraj Zelman"""

import matplotlib.pyplot as plt
import numpy as np
import scipy.optimize as spo

from scipy.integrate import solve_ivp


def main():
    """
    Problem 5 - Landing maneuver of a blimp.

    Numerical solution of the optimal control problem using single shooting
    method.
    """

    T = 1  # Time horizon
    alpha_x, alpha_z, beta = 5, 10, 8  # Drag coefficients and wind speed
    const = [alpha_x, alpha_z, beta]  # Constants for blimp dynamics
    s_0 = np.array([-40, 20, 2, 0])  # Initial state
    s_T = np.array([0, 0, 0, 0])  # Terminal state

    # Define residual function
    f_res = lambda p_0: np.linalg.norm(F(p_0, s_0, T, const) - s_T)

    # Optimize the residual function to find the optimal co-state at time 0
    p_0_star, _, _, _, warnflag = spo.fmin(
        func=f_res,
        x0=np.zeros(4),
        xtol=1e-7,
        ftol=1e-7,
        maxfun=1e5,
        maxiter=1e5,
        full_output=True,
    )

    # Recover the optimal solution
    sol = solve_ivp(
        lambda t, y: f_tilde(t, y, const),
        [0, T],
        np.concatenate((s_0, p_0_star)),
        t_eval=np.linspace(0, T, 100),
    )
    s_star, p_star = sol.y[:4, :], sol.y[4:, :]  # Optimal state and co-state
    u_star = np.vstack((-p_star[1, :], -p_star[3, :]))  # Optimal control

    # Plot the results
    if warnflag != 0:
        print("Optimization failed!")
    else:
        print("Plotting the results.")
        fig, (ax1, ax2) = plt.subplots(2, figsize=(8, 6))

        # Plot optimal blimp position trajectory
        ax1.plot(s_star[0, :], s_star[2, :])
        ax1.set_title("Optimal Blimp Position Trajectory")
        ax1.set_xlabel(r"Position $x(t)$ [km]")
        ax1.set_ylabel(r"Height $z(t)$ [km]")

        # Plot corresponding optimal control inputs
        ax2.plot(sol.t, u_star[0, :], "b")
        ax2.plot(sol.t, u_star[1, :], "r")
        ax2.set_title("Optimal Control Inputs")
        ax2.set_xlabel("Time $t$ [h]")
        ax2.set_ylabel(r"Control input u(t) [km/h$^2$]")
        ax2.legend([r"$u_1$", r"$u_2$"])

        plt.tight_layout()
        plt.show()


def f_tilde(t: float, state: np.ndarray, const: list[float]) -> np.ndarray:
    """
    First-order differential equation describing the state and co-state of the
    blimp dynamics derived by the Potryagin's minimum principle.

    Args:
        t: Time.
        state: State vector containing the state and co-state variables.
        const: List of three constants [alpha_x, alpha_z, beta] describing the
            blimp dynamics.

    Returns:
        State dynamics and co-state dynamics concatenated into a single vector.
    """
    alpha_x, alpha_z, beta = const  # Constants from blimp dynamics

    s = state[:4]  # State variables
    p = state[4:]  # Co-state variables

    # Control
    u_star = np.zeros(2)
    u_star[0] = -p[1]
    u_star[1] = -p[3]

    # State dynamics
    s_dot = np.zeros(4)
    s_dot[0] = s[1]
    s_dot[1] = -alpha_x * (s[1] - beta * s[2] ** 2) + u_star[0]
    s_dot[2] = s[3]
    s_dot[3] = -alpha_z * s[3] + u_star[1]

    # Co-state dynamics
    p_dot = np.zeros(4)
    p_dot[0] = 0
    p_dot[1] = -p[0] + p[1] * alpha_x
    p_dot[2] = -2 * p[1] * alpha_x * beta * s[2]
    p_dot[3] = -p[2] + p[3] * alpha_z

    return np.concatenate((s_dot, p_dot))


def F(p_0: np.ndarray, s_0: np.ndarray, T: float, const: list[float]) -> np.ndarray:
    """
    Simulate the blimp dynamics for a given time horizon and initial state.

    Args:
        p_0: Co-state at time 0.
        s_0: State at time 0.
        T: Terminal time.
        const: List of three constants [alpha_x, alpha_z, beta] describing the
            blimp dynamics.

    Returns:
        The terminal state s(T) of the blimp dynamics.
    """
    # Simulate the blimp dynamics
    state = np.concatenate((s_0, p_0))
    sol = solve_ivp(lambda t, y: f_tilde(t, y, const), [0, T], state)

    # Extract the terminal state
    s_T_bar = sol.y[:4, -1]

    return s_T_bar


if __name__ == "__main__":
    main()
