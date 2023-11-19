# A star (or any other path planning algorithm here)

from Node import Node, FindNodeDist, NODE_MAX_VALUE

import sys
from queue import PriorityQueue
import numpy as np

class PathPlanner:
    def __init__(self, nodes_list):
        self.nodes_list = nodes_list
        self.num_nodes = len(nodes_list)
        self.nodes_f = NODE_MAX_VALUE*np.ones(self.num_nodes)
        self.nodes_cost = NODE_MAX_VALUE*np.ones(self.num_nodes)
        self.nodes_h = -1*np.ones(self.num_nodes)
        self.nodes_prev_node = [Node] * self.num_nodes
        self.current_start_node_id = -1

    def reset_graph(self):
        self.nodes_f = np.ones(self.num_nodes)
        self.nodes_cost = NODE_MAX_VALUE*np.ones(self.num_nodes)
        self.nodes_h = -1*np.ones(self.num_nodes)
        self.nodes_prev_node = [Node] * self.num_nodes
        self.current_start_node_id = -1

    def astar(self, start_node: Node, end_node: Node):
        # separate f, g, h variables for larger maps? use matrices, np

        pq = PriorityQueue()
        if self.current_start_node_id != start_node.id:
            self.reset_graph()
            self.current_start_node_id = start_node.id

        if self.nodes_cost[end_node.id] < NODE_MAX_VALUE:
            # path has already been found before
            path = []
            n = end_node
            while n != start_node:
                path.insert(0, n)
                n = self.nodes_prev_node[n.id]
            path.insert(0, n)
            return path, self.nodes_cost[end_node.id]

        self.nodes_cost[start_node.id] = 0
        pq.put(start_node)
        while not pq.empty():
            curr_node = pq.get()
            adjacentNodes = curr_node.adj_nodes
            for n, cost in adjacentNodes:
                total_cost = self.nodes_cost[curr_node.id] + cost
                if self.nodes_cost[n.id] > total_cost:
                    self.nodes_cost[n.id] = total_cost
                    if self.nodes_h[n.id] == -1:
                        self.nodes_h[n.id] = FindNodeDist(n, end_node)
                    n.f = self.nodes_cost[n.id] + self.nodes_h[n.id]
                    self.nodes_f[n.id] = n.f
                    pq.put(n)
                    self.nodes_prev_node[n.id] = curr_node

        path = []
        final_cost = self.nodes_cost[end_node.id]
        if final_cost < NODE_MAX_VALUE:
            # found path
            n = end_node
            while n != start_node:
                path.insert(0, n)
                n = self.nodes_prev_node[n.id]
            path.insert(0, n)
        else:
            #likely because it has not been found before, try working backwards
            print("uh oh")
            pass

        return path, final_cost
    
    # find shortest path between two nodes
    def get_path(self, start_node: Node, end_node: Node):
        if self.current_start_node_id != start_node.id:
            self.explore()

        path = []
        final_cost = self.nodes_cost[end_node.id]
        if final_cost < NODE_MAX_VALUE:
            # found path
            n = end_node
            while n != start_node:
                path.insert(0, n)
                n = self.nodes_prev_node[n.id]
            path.insert(0, n)

        return path, final_cost
    
    # dijkstra to explore every node from the start point
    def explore(self, start_node: Node):
        if self.current_start_node_id != start_node.id:
            self.reset_graph()
            self.current_start_node_id = start_node.id

        pq = PriorityQueue()
        self.nodes_cost[start_node.id] = 0
        pq.put(start_node)
        while not pq.empty():
            curr_node = pq.get()
            adjacentNodes = curr_node.adj_nodes
            for n, cost in adjacentNodes:
                total_cost = self.nodes_cost[curr_node.id] + cost
                if self.nodes_cost[n.id] > total_cost:
                    self.nodes_cost[n.id] = total_cost
                    n.f = total_cost
                    pq.put(n)
                    self.nodes_prev_node[n.id] = curr_node

    
# once a truck has a set of nodes to deliver to, use tabu search or simulated annealing for pathing to each


