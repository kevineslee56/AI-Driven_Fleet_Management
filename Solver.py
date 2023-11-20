# implements non path planning logic
# eg: 
#   which trucks take which deliveries (clustering of delivery locations)
#   how many trucks should be used
#   balance distance and time (cost function)

from Node import Node
from PathPlanner import PathPlanner

import numpy as np

class Solver:
    def __init__(self, start_node, nodes_list, deliveries, trucks):
        self.start_node = start_node
        self.nodes_list = nodes_list
        self.deliveries = deliveries
        self.trucks = trucks
        self.cost_matrix = np.ones(shape=len(deliveries))
        self.pathPlanner = PathPlanner(nodes_list)
        for i in range(0, len(deliveries) - 1):
            current_delivery = deliveries[i]
            for j in range(i+1, len(deliveries)):
                other_delivery = deliveries[j]
                path, cost = self.pathPlanner.astar(current_delivery, other_delivery)
                print(cost)
                self.cost_matrix[i][j] = cost
                self.cost_matrix[j][i] = cost

    def greed_solve(self):
        pass

    def s1(self):
        ''' 
        dijkstra on start point, get shortest paths to each delivery
            sum cost
        for every delivery run dijkstra again, get shortest distance to every node
        start with closest delivery point
            within a thresh, find all nodes
                for each node, if it is part of a path for another delivery, add
        '''
    
    def s2(self):
        '''
        dijkstra on start point, get shortest paths to each delivery
            sum cost
        for every delivery run dijkstra again, get shortest distance to every other delivery node
        order delivery nodes by distance from start node


        '''

    def s3(self):
        '''
        split into even pie pieces
        assign nodes to pie piece
        for each pie piece:
            for each node:
                estimate cost of it staying in the piece vs moving to an adjacent piece
        '''


    def cluster(self, num_clusters):
        # based on Hartigan-Wong variation of K-means clustering
        k = num_clusters

        

    