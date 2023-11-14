# A star (or any other path planning algorithm here)

import sys
import Node
from queue import PriorityQueue

def astar(start_node: Node, end_node: Node):
    # separate f, g, h variables for larger maps? use matrices, np

    pq = PriorityQueue()
    start_node.cost = 0
    pq.put(start_node)
    while not pq.empty():
        curr_node = pq.get()
        adjacentNodes = curr_node.adj_nodes
        for n, cost in adjacentNodes:
            total_cost = curr_node.cost + cost
            if n.cost > total_cost:
                n.cost = total_cost
                if n.h_dist == -1:
                    n.updateH(end_node)
                n.f = n.cost + n.h_dist
                pq.put(n)
                n.prev_node = curr_node

    path = []
    if end_node.cost < 1000000000:
        # found path
        print(end_node.cost)
        n = end_node
        while n != start_node:
            path.insert(0, n)
            n = n.prev_node
        path.insert(0, n)
    
    return path
