# Problem 2

This is an Exercise 7 from the book *Dynamic Programming and Optimal Control by Dimitri P. Bertsekas, Vol. I, 3rd edition, 2005.*

A tennis player has a Fast serve and a Slow serve, denoted $F$ and $S$ respectively. The probability of $F$ (or $S$) landing in bounds is $p_F$ (or $p_S$, respectively). The probability of winning the point
assuming the serve landed in bounds is $q_F$ (or $q_S$ , respectively). We assume that $p_F < p_S$ and $q_F > q_S$. The problem is to find the serve to be used at each possible scoring situation during a single game in order to maximize the probability of winning that game.
1. Formulate this as a stochastic shortest path problem, and write Bellman’s equation.
2. Computer assignment: Assume that $q_F = 0.6$, $q_S = 0.4$, and $p_S = 0.95$. Use value iteration to calculate and plot (in increment of $0.05$) the probability of the server winning a game with optimal serve selection as a function of $p_F$.

### Solution

Can be found in `Problem_2.py`.