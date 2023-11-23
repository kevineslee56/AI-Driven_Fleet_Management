from Node import Node, FindNodeDist, NODE_MAX_VALUE

import sys
import numpy as np
from sys import maxsize 
from queue import PriorityQueue
from itertools import permutations


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
            # likely because it has not been found before, try working backwards
            print("uh oh")
            pass

        return path, final_cost
    
    # dijkstra to explore every node from the start point
    def dijkstra(self, start_node: Node):
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

    # find shortest path between two nodes
    def get_path(self, start_node: Node, end_node: Node):
        if self.current_start_node_id != start_node.id:
            self.dijkstra(start_node)

        path = []
        if self.nodes_cost[end_node.id] < NODE_MAX_VALUE:
            # found path
            n = end_node
            while n != start_node:
                path.insert(0, n)
                n = self.nodes_prev_node[n.id]
            path.insert(0, start_node)

        return path, self.nodes_cost[end_node.id]
 
    def tsp(self, start_node, delivery_nodes, paths_from_start_to_delivery, paths_from_delivery_to_delivery): 
        min_cost = maxsize
        best_path = []
        next_permutations = permutations(delivery_nodes)
        for i in next_permutations:
            current_cost = 0
            current_path = []
            k = start_node
            for j in i: 
                cost = 0
                path = []
                if k == start_node:
                    path, cost = paths_from_start_to_delivery[j.id] 
                else:
                    path, cost = paths_from_delivery_to_delivery[(k.id,j.id)]
                
                current_cost += cost
                current_path.extend(path)
                k = j 

            path, cost = paths_from_start_to_delivery[k.id]
            current_cost += cost
            current_path.extend(reversed(path))
    
            # update minimum 
            if current_cost < min_cost:
                min_cost = current_cost
                best_path = current_path

        return best_path, min_cost
    