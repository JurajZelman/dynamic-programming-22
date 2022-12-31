# Problem 5

The goal of this programming exercise is to solve an optimal control problem, which has no closed-form solution, numerically.

Consider the landing maneuver of a blimp. The blimp can move in a vertical plane and its
position is denoted by $(x,z)$. A wind, of which the velocity is given by $w(z) = \beta z^2$, is blowing in $x$-direction. The blimp is controlled by means of two thrusters $u_1$ and $u_2$ pointing in $x$- and $z$-direction, respectively. Furthermore, a linear drag force acts on the blimp in each direction, resulting in the following system dynamics:

$
\quad \ddot{x} = - \alpha_x (\dot{x} - w(z)) + u_1, \\
\quad \ddot{z} = - \alpha_z \dot{z} + u_2,
$

where $\alpha_x$ and $\alpha_z$ denote the drag coefficients in $x$- and $y$-direction, respectively. The problem is to find the horizontal and vertical thrust input $u_1(t)$ and $u_2(t)$, $0 \le t \le T$, which minimize the control effort

$$ \int_0^T \frac{1}{2} (u_1(t)^2 + u_2(t)^2)dt$$

required for the blimp to move from an arbitrary initial state to a given landing point in time $T$.

1. For the state vector $s$ being defined as $$s = (x, \dot{x}, z, \dot{z}),$$ write down the Hamiltonian function of the above optimal control problem as the function of $s \in \mathbb{R}^4, p \in \mathbb{R}^4$ and $u \in \mathbb{R}^2$: $$H(s,p,u) = g(s,u) + p^Tf(s,u).$$

2. Recall that the control $u^*$ that minimizes the above optimal control problem can be obtained by minimizing the Hamiltonian, i.e. $$u^*(s,p) = \arg \min_u H(s, p, u).$$ Compute the optimal $u^*$ by setting the gradient of the Hamiltonian with respect to $u$ to zero, i.e. solve $$\frac{\partial H}{\partial u} (p, s, u^*) = 0$$ for $u^*= (u_1^*, u_2^*)$.

3. Write down the adjoint equations $$\dot{p}(t) = - \nabla_s H(s, p, u^*(s,p)).$$

4. Collect the blimp state $s$ and the costate $p$ in a new vector $y = (s, p)$. The evolution of $y$
is then described by the first-order differential equation $$y = \tilde{f}(y) = \begin{pmatrix} f(s, u^*(s, p)) \\ - \nabla_s H(s, p, u^*(s,p))\end{pmatrix}.$$
The boundary value problem (BVP) that needs to be solved is given by $$s(0) = s_0 = \begin{pmatrix} x_0 \\ \dot{x}_0 \\ z_0 \\ \dot{z}_0 \end{pmatrix},$$ $$s(T) = s_T = \begin{pmatrix} x_T \\ \dot{x}_T \\ z_T \\ \dot{z}_T \end{pmatrix},$$ $$y = \tilde{f}(y), \quad 0 \le t \le T.$$ There are no constraints on the adjoints. We will solve the BVP by single shooting and using _scipy.optimize.fmin_ in Python. The first step is to solve the initial value problem $$y(0) = y_0,$$ $$\dot{y}(t) = \tilde{f}(y(t)), \quad 0 \le t \le T.$$ Note that $y_0 = (s_0, p_0)$. As the initial value for the blimp state is fixed to $s_0$, we only need to find the correct initial value for the adjoints, $p_0$. Write a MATLAB or a Python function $F(p_0)$, using _scipy.integrate.solve_ivp_  in Python, that takes $p_0$ as an input and returns the final blimp state $s(T)$, $F: \mathbb{R}^4 \to \mathbb{R}^4$. Use the following numerical values:

| <!-- -->    | <!-- -->    | <!-- -->    | <!-- -->    |
|-------------|-------------|-------------|-------------|
| Time horizon $T$                  | 1 [h]      | Drag coefficient $\alpha_x$                 | 5 [1/h]      |
| Wind speed constant $\beta$       | 8 [h/km]   | Drag coefficient $\alpha_z$                 | 10 [1/h]      |
| Initial position $x_0$            | -40 [km]   | Final position $x_T$                 | 0 [km]      |
| Initial position $z_0$            | 2 [km]     | Final position $z_T$                 | 0 [km]  
| Initial velocity $\dot{x}_0$      | 20 [km/h]  | Final velocity $\dot{x}_T$                 | 0 [km/h]  
| Initial velocity $\dot{z}_0$      | 0 [km/h]   | Final velocity $\dot{z}_T$                 | 0 [km/h]  

5. The solution of the BVP is found if we have found $p_0^*$ such that $F(p_0^*) = s_T$. Use _scipy.optimize.fmin_ in Python to find $p_0^*$ by minimizing $||F(p_0) - s_T||_2$. Plot the optimal control input and blimp trajectory and comment on the results.
