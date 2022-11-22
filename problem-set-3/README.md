# Problem 4

Consider a graph with $N$ nodes. The transition costs between node $i$ and $j$ are given by $c_{i,j}$, where

$$c_{i,j} =
\begin{cases}
0, \quad \text{if } i = j,\\
\infty, \quad \text{if $i$ and $j$ are not connected,}\\
c_{i,j} \in \mathbb{R}^+, \quad \text{otherwise.}
\end{cases} $$

The objective is to find the shortest path from starting node $s=1$ to terminal node $T=100$ and its associated cost. 

a) Implement a Python function that solves the shortest path problem using the Label Correcting algorithm.

b) A positive underestimate of the cost to move from node $i$ to node $j$ is given by 

$$c_{i,j} > |i-j|.$$

Modify your implementation of a) by taking the lower bound on the costs into account to obtain A* algorithm.

The associated cost matrix can be loaded from `A.mat`.

### Solution

Can be found in `Problem_4.py`.