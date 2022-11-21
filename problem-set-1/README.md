# Problem 1

Consider the system 

$$`x_{k+1} = x_k + u_k w_k, \quad k = 0,1,\dots, 9,`$$

with initial state $`x_0`$ being an integer and constrained to $`0 \le x_0 \le 10`$. The control constraint set is given by $`U_k(x_k) := \{u | 0 \le x_k + u \le 10, u: \text{integer}\}`$ for all $`x_k`$ and $`k`$, and the disturbance $`w_k`$ takes the value $`1`$ with probability $`\frac{1}{3}`$ and the value $`0`$ with probability $`\frac{2}{3}`$ for all $`x_k`$ and $`u_k`$. The cost function is given by

$$`x_{10}^2 + \sum_{k=0}^{9}((x_k - x_{ref}(k))^2 + u_k^2),`$$

with $`x_{ref}(k) = (k - 5)^2.`$ Implement a script that computes the optimal cost $`J_0(x_0)`$ for all initial states $`x_0, 0 \le x_0 \le 10,`$ by applying the DP algorithm.

### Solution

Can be found in `Problem_1.py`.