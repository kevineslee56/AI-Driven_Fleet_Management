# A star (or any other path planning algorithm here)

import sys
import Node
from queue import PriorityQueue

def astar(start_node: Node, end_node: Node):
    # separate f, g, h variables for larger maps? use matrices, np

    pq = PriorityQueue()
    pq.put(start_node)
    while not pq.empty():
        curr_node = pq.get()
        adjacentNodes = curr_node.adj_nodes
        for n, cost in adjacentNodes:
            total_cost = curr_node.f + cost
            if n.f > total_cost:
                n.f = total_cost
                pq.put(n)
                n.prev_node = curr_node

    path = []
    if end_node.f < sys.maxsize:
        # found path
        n = end_node
        while not n == start_node:
            path.insert(0, n)
            n = n.prev_node
        path.insert(0, n)
    
    return path
