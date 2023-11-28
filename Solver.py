# implements non path planning logic
# eg: 
#   which trucks take which deliveries (clustering of delivery locations)
#   how many trucks should be used
#   balance distance and time (cost function)

from Truck import Truck
from Node import Node, NODE_MAX_VALUE, FindNodeDist
from PathPlanner import PathPlanner
from SolutionState import SolutionState, NEW_SOLN, SAME_COST, NO_NEW_SOLN

import numpy as np
import random
import time
import copy

class Solver:
    def __init__(self, start_node : Node, nodes_list : [Node], deliveries : [Node], trucks : [Truck], x_range, y_range, w_dist = 0.5, w_time = 0.5):
        self.w_dist = w_dist
        self.w_time = w_time
        self.start_node = start_node
        self.nodes_list = nodes_list
        self.deliveries = deliveries
        self.trucks = trucks
        self.pathPlanner = PathPlanner(nodes_list)
        self.nodes_cost = self.pathPlanner.explore(self.start_node)
        assert(self.nodes_cost[int(self.start_node.id)] == 0)
        self.cost_dict = {}
        self.cost_dict[start_node.key + ":" + start_node.key] = 0
        for delivery in self.deliveries:
            cost_from_depot = self.nodes_cost[delivery.id]
            self.set_cost(self.start_node, delivery, cost_from_depot)

        self.deliveries.sort(key = lambda x: x.f, reverse=True)

        self.x_range = x_range
        self.y_range = y_range

    def set_cost(self, n1: Node, n2: Node, cost):
        key1 = str(n1.id) + ":" + str(n2.id)
        key2 = str(n2.id) + ":" + str(n1.id)
        if key1 in self.cost_dict:
            return
        self.cost_dict[key1] = cost
        self.cost_dict[key2] = cost

    def get_cost(self, n_from: Node, n_to: Node):
        key1 = str(n_from.id) + ":" + str(n_to.id)
        return self.cost_dict.get(key1, NODE_MAX_VALUE)

    def reset_trucks(self):
        for truck in self.trucks:
            truck.reset()

    def find_costs(self):
        for i in range(0, len(self.deliveries) - 1):
            d1 = self.deliveries[i]
            self.pathPlanner.explore(d1)
            for j in range(i, len(self.deliveries)):
                d2 = self.deliveries[j]
                cost = self.pathPlanner.nodes_cost[int(d2.id)]
                self.set_cost(d1, d2, cost)
        return self.cost_dict

    def find_estimated_costs(self):
        for i in range(0, len(self.deliveries) - 1):
            d1 = self.deliveries[i]
            for j in range(i, len(self.deliveries)):
                d2 = self.deliveries[j]
                cost = FindNodeDist(d1, d2)
                self.set_cost(d1, d2, cost)
        return self.cost_dict

    # above, but each cluster is built at the same time
    def concurrent_even_split(self):
        costs_from_start = {}
        for delivery in self.deliveries:
            costs_from_start[str(delivery.id)] = delivery.f

        casted_ray_x = [float] * len(self.trucks)
        casted_ray_y = [float] * len(self.trucks)
        end_nodes = []
        for i in range(0, len(self.trucks)):
            theta = i*((2*np.pi)/(len(self.trucks)))
            origin_shift = [self.start_node.x, self.start_node.y]
            temp_y = 0
            temp_x = 0
            line_x = 0
            line_y = 0
            if theta >= 0 and theta <= 0.5*np.pi:
                #Q1
                temp_y = self.y_range[1] - origin_shift[1]
                temp_x = self.x_range[1] - origin_shift[0]
            elif theta > 0.5*np.pi and theta <= np.pi:
                #Q2
                temp_y = self.y_range[1] - origin_shift[1]
                temp_x = self.x_range[0] - origin_shift[0]
            elif theta > np.pi and theta <= 1.5*np.pi:
                #Q3
                temp_y = self.y_range[0] - origin_shift[1]
                temp_x = self.x_range[0] - origin_shift[0]
            else:
                #Q4
                temp_y = self.y_range[0] - origin_shift[1]
                temp_x = self.x_range[1] - origin_shift[0]
            line_y = np.tan(theta) * temp_x
            if abs(line_y) > abs(temp_y):
                line_x = temp_y/(np.tan(theta))
                casted_ray_x[i] = line_x + origin_shift[0]
                casted_ray_y[i] = temp_y + origin_shift[1]
            else:
                casted_ray_x[i] = temp_x + origin_shift[0]
                casted_ray_y[i] = line_y + origin_shift[1]
            temp_node = Node(-100, casted_ray_x[i], casted_ray_y[i])
            closest_node = None
            closest_dist = NODE_MAX_VALUE
            for node in self.deliveries:
                dist = FindNodeDist(temp_node, node)
                if dist < closest_dist:
                    closest_dist = dist
                    closest_node = node
            end_nodes.append(closest_node)
        

        radius_adjustment = [0] * len(end_nodes)
        max_radius = 250
        for i in range(0, len(radius_adjustment)):
            radius_adjustment[i] = max_radius - self.get_cost(self.start_node, end_nodes[i])
        for i in range(0, len(end_nodes)):
            end_nodes[i].assigned = True
            self.trucks[i].packages.append(end_nodes[i])

        distance_dict = {}
        for i in range(0, len(end_nodes)):
            end_node = end_nodes[i]
            results = self.pathPlanner.explore(end_node)
            distance_dict[str(i)] = results


        for node in self.deliveries:
            lowest_cost = NODE_MAX_VALUE
            best_i = -1
            for i in range(0, len(end_nodes)):
                end_node = end_nodes[i]
                total_cost = distance_dict[str(i)][node.id] + self.get_cost(self.start_node, node) + radius_adjustment[i]
                if (total_cost < lowest_cost):
                    lowest_cost = total_cost
                    best_i = i
            if best_i == -1 or node.assigned:
                continue
            node.assigned = True
            self.trucks[best_i].packages.append(node)
        return self.trucks

    def random_routes(self):
        for delivery in self.deliveries:
            rand_truck = random.choice(self.trucks)
            rand_truck.packages.append(delivery)
        return self.trucks

    def simulated_annealing(self, init_T, a1, a2, max_iter, max_iter_time=120, initial_trucks=None, all_neighbours_flag=True):
        if initial_trucks == None:
            # quick method of clustering for the initial solution
            if all_neighbours_flag:
                initial_trucks = self.random_routes()
            else:
                initial_trucks = self.concurrent_even_split()

        initial_routes = {}
        for truck in initial_trucks:
            initial_routes[str(truck.id)] = truck.packages

        best_state_cost = NODE_MAX_VALUE
        best_total_dist = None
        best_longest_route = None

        for i in range(0, max_iter):
            st = time.time()
            new_state, new_state_cost, new_total_dist, new_longest_route = self.simulated_annealing_helper(init_T, a1, a2, initial_routes, max_iter_time, all_neighbours_flag)
            if new_state_cost < best_state_cost:
                for truck in initial_trucks:
                    truck.packages = [*new_state.routes[truck.key]]
                best_state_cost = new_state_cost
                best_total_dist = new_total_dist
                best_longest_route = new_longest_route
            print("Completed Simulated Annealing iteration " + str(i+1) + " of " + str(max_iter) + " in " + str(time.time() - st) + " seconds.")

        return initial_trucks, best_state_cost, best_total_dist, best_longest_route
        
    def simulated_annealing_helper(self, init_T, a1, a2, initial_routes, max_iter_time, all_neighbours_flag):
        state = SolutionState(initial_routes, self.cost_dict, self.start_node, self.w_dist, self.w_time)

        # simulated annealing here:
        a = 0.99
        cur_T = init_T

        repeated = 0
        repeated_cost = 0
        st = time.time()
        debug = 0
        while repeated < a1 and repeated_cost < a2 and ((time.time() - st) < float(max_iter_time)):
            
            '''# for each route, try to improve it within just the route
            result1 = NO_NEW_SOLN
            truck = random.choice(self.trucks)
            #for truck in self.trucks:
            result1 = state.create_neighbour_route(truck, cur_T)
            if result1 == NEW_SOLN:
                repeated = 0
                repeated_cost = 0
            elif result1 == SAME_COST:
                repeated = 0
                repeated_cost = repeated_cost + 1
            elif result1 == NO_NEW_SOLN:
                repeated = repeated + 1
                repeated_cost = repeated_cost + 1'''
            
            
            # now also try to improve it by letting the routes exchange packages
            result2 = state.create_neighbour(self.trucks, cur_T, all_neighbours_flag)
            
            if result2 == NEW_SOLN:
                repeated = 0
                repeated_cost = 0
            elif result2 == SAME_COST:
                repeated = 0
                repeated_cost = repeated_cost + 1
            elif result2 == NO_NEW_SOLN:
                repeated = repeated + 1
                repeated_cost = repeated_cost + 1
            
            # decrement temp
            cur_T = cur_T * a

            debug = debug + 1

        print("executions:")
        print(debug)
        return state, state.get_total_cost(), state.total_distance_cost, state.longest_route_cost
        

    