"""Copyright (c) 2022 Juraj Zelman"""

import time

import numpy as np
import scipy.io


def lca(A, startNode, terminalNode):
    """
    Executes Label Correcting algorithm (Book Dynamic Programming and Optimal
    Control, Bertsekes, page 81) using the depth-first method.

    Args:
        A: [NxN] matrix, where the element A(i,j) = a_ij is the cost to move
            from node i to j.
        startNode: Start node of desired shortest path, scalar from 1 to N.
        terminalNode: Terminal node of desired shortest path, scalar from
            1 to N.

    Returns:
        optCost: Cost of the shortest path(s), scalar:
        optPath: Row vector containing the shortest path,
            e.g. optPath = [0 32 44 43 78 99].
    """
    N = len(A)
    d = np.ones(N) * np.inf  # Labels for each node
    d[startNode] = 0
    parent = np.ones(N) * np.inf  # Parent of the shortest path for each node
    parent[startNode] = 0
    OPEN = np.zeros(N)  # List containing nodes that are to be processed
    pointerOPEN = 1  # Pointer which always points to the last element in OPEN
    OPEN[pointerOPEN] = startNode
    UPPER = np.inf  # Representing the shortest path to the end so far

    if startNode == terminalNode:
        return 0, [startNode, terminalNode]

    if (startNode >= N or terminalNode >= N) or (
        startNode < 0 or terminalNode < 0
    ):
        return np.inf, None

    while True:
        # STEP 1: Remove the node i from OPEN and find its children
        i = int(OPEN[pointerOPEN])
        OPEN[pointerOPEN] = 0
        pointerOPEN = pointerOPEN - 1

        children = np.where(A[i, :] != np.inf)
        children = children[0]
        if i in children:
            children = np.delete(children, np.where(children == i))

        # STEP 2: Update the labels of the children
        for j in children:
            if d[i] + A[i, j] < min(d[j], UPPER):
                d[j] = d[i] + A[i, j]
                parent[j] = int(i)

                if j != terminalNode:
                    pointerOPEN = pointerOPEN + 1
                    OPEN[pointerOPEN] = j
                else:
                    UPPER = d[j]

        if not pointerOPEN:
            break

    optCost = UPPER

    # Reconstruct the shortest path
    optPath = [terminalNode]
    while optPath[-1] != startNode:
        optPath.append(int(parent[int(optPath[-1])]))
    optPath.reverse()

    return optCost, optPath


def astar(A, startNode, terminalNode):
    """
    Executes A* algorithm (Book Dynamic Programming and Optimal Control,
    Bertsekes, page 87) using the depth-first method.

    Args:
        A: [NxN] matrix, where the element A(i,j) = a_ij is the cost to move
            from node i to j.
        startNode: Start node of desired shortest path, scalar from 1 to N.
        terminalNode: Terminal node of desired shortest path, scalar from
            1 to N.

    Returns:
        optCost: Cost of the shortest path(s), scalar:
        optPath  Row vector containing the shortest path,
            e.g. optPath = [0 33 45 43 79 99].
    """
    N = len(A)
    d = np.ones(N) * np.inf  # Labels for each node
    d[startNode] = 0
    parent = np.ones(N) * np.inf  # Parent of the shortest path for each node
    parent[startNode] = 0
    OPEN = np.zeros(N)  # List containing nodes that are to be processed
    pointerOPEN = 1  # Pointer which always points to the last element in OPEN
    OPEN[pointerOPEN] = startNode
    UPPER = np.inf  # Representing the shortest path to the end so far

    if startNode == terminalNode:
        return 0, [startNode, terminalNode]

    if (startNode >= N or terminalNode >= N) or (
        startNode < 0 or terminalNode < 0
    ):
        return np.inf, None

    while True:
        # STEP 1: Remove the node i from OPEN and find its children
        i = int(OPEN[pointerOPEN])
        OPEN[pointerOPEN] = 0
        pointerOPEN = pointerOPEN - 1

        children = np.where(A[i, :] != np.inf)
        children = children[0]
        if i in children:
            children = np.delete(children, np.where(children == i))

        # STEP 2: Update the labels of the children
        for j in children:
            if (d[i] + A[i, j] < d[j]) and (
                d[i] + A[i, j] + abs(j - terminalNode) < UPPER
            ):
                d[j] = d[i] + A[i, j]
                parent[j] = int(i)

                if j != terminalNode:
                    pointerOPEN = pointerOPEN + 1
                    OPEN[pointerOPEN] = j
                else:
                    UPPER = d[j]

        if not pointerOPEN:
            break

    optCost = UPPER

    # Reconstruct the shortest path
    optPath = [terminalNode]
    while optPath[-1] != startNode:
        optPath.append(int(parent[int(optPath[-1])]))
    optPath.reverse()

    return optCost, optPath


def main():
    """Compute the shortest path using LCA and A* algorithms."""

    mat = scipy.io.loadmat("A.mat")
    A = mat["A"]  # Load matrix A that contains all the transition costs
    N = len(A)  # Dimension of the problem: N = total number of nodes

    # Default values:
    startNode = 0
    terminalNode = 99

    # Solve shortest path problem using the Label Correcting Algorithm
    t = time.time()
    [optCost1, optPath1] = lca(A, startNode, terminalNode)
    time1 = time.time() - t

    # Solve shortest path problem using the A* Algorithm
    t = time.time()
    [optCost2, optPath2] = astar(A, startNode, terminalNode)
    time2 = time.time() - t

    # Print results
    print(f"Results of the Problem with {N} nodes.")
    print("Optimal path from node ", startNode, " to ", terminalNode, ":")
    print()
    print("\033[1mLabel Correcting Algorithm\033[0m")
    print(f"Execution time: {time1:.3f} s.")
    print(f"Minimum path length (minimum total cost): {optCost1}")
    print(f"Path: {optPath1}")
    print()
    print("\033[1mA* Algorithm\033[0m")
    print(f"Execution time: {time2:.3f} s.")
    print(f"This is {time2 / time1:.3f} times the time for method 1.")
    print(f"Minimum path length (minimum total cost): {optCost2}")
    print(f"Path: {optPath2}")


if __name__ == "__main__":
    main()
