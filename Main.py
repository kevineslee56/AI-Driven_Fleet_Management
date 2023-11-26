import time
import math
import random
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import to_rgb
from matplotlib.collections import LineCollection

from MapMaker import GenerateMap, PlotMap
from PathPlanner import PathPlanner
from Solver import Solver
from Truck import Truck

# helper functions for verifying solution costs
def get_route_dist(start_node, route, cost_dict):
    cost = 0
    prev_node = start_node
    for delivery in route:
        cost = cost + cost_dict[prev_node.key + ":" + delivery.key]
        prev_node = delivery
    cost = cost + cost_dict[prev_node.key + ":" + start_node.key]
    return cost

def cost_checker(start_node, trucks, cost_dict, w_dist, w_time):
    total_dist = 0
    longest_route_dist = 0
    for truck in trucks:
        route = truck.packages
        route_dist = get_route_dist(start_node, route, cost_dict)
        total_dist = total_dist + route_dist
        if route_dist > longest_route_dist:
                longest_route_dist = route_dist
    return (total_dist * w_dist) + (longest_route_dist * w_time)


def main():
    # graph properties:
    num_nodes = 500 # suggested max: 10000, for visibility, 500
    num_neighbours = 4 # suggested for all sized maps: 4-6 (less dense graphs, use higher numbers)
    x_min = 0
    x_max = 350 # suggested max 350
    y_min = 0
    y_max = 350 # suggested max 350
    nodes_list = GenerateMap(num_nodes, num_neighbours, x_min, x_max, y_min, y_max)
    start_node, figure_ax = PlotMap(nodes_list)
    pathPlanner = PathPlanner(nodes_list)

    # problem/delivery properties:
    num_deliveries = 50 # suggested max 1000, for visibility, 50
    deliveries = random.sample(nodes_list, num_deliveries)
    for delivery_node in deliveries:
        delivery_node.is_delivery = True
    # trucks
    num_trucks = 10 # suggested max 10
    trucks = []
    truck_capacity = 10000000
    for i in range(0, num_trucks):
        trucks.append(Truck(i, truck_capacity))

    # solver properties:
    w_dist = 0.5
    w_time = 0.5 * (num_trucks/2) # approximate adjustment to account for total distance usually being a few times larger than the longest route
    initial_temp = 10000
    stop_condition_1 = 20000
    stop_condition_2 = 150000
    max_iter = 1
    max_time_per_iter = 120
    all_neighbours_flag = True

    # plot "warehouse"
    figure_ax.scatter(start_node.x, start_node.y, marker='.', color='lime', s=50, zorder=15)

    # find solution:
    st = time.time()
    solver = Solver(start_node=start_node, nodes_list=nodes_list, deliveries=deliveries, trucks=trucks, x_range=[x_min, x_max], y_range=[y_min, y_max], w_dist=w_dist, w_time=w_time)
    # find list of lowest costs between all the delivery nodes
    print("Analyzing graph, getting distance data between deliveries, estimate " + str(0.1*num_deliveries) + "s.")
    cost_dict = solver.find_costs() # takes about 2 min for 1000 deliveries
    
    print("Running Solver (limited to 4 min max runtime)...")
    if len(deliveries) > 275:
        # first cluster, then TSP
        clustered_trucks, cost, total_dist, longest_route = solver.simulated_annealing(initial_temp, stop_condition_1, stop_condition_2, max_iter, max_time_per_iter, all_neighbours_flag=False)
    else:
        # just run simulated annealing normally
        clustered_trucks, cost, total_dist, longest_route = solver.simulated_annealing(initial_temp, stop_condition_1*(275/len(deliveries)), stop_condition_2*5, max_iter, max_time_per_iter*2, all_neighbours_flag=True)
        
    # solution results
    assert(math.isclose(cost, cost_checker(start_node, clustered_trucks, cost_dict, w_dist, w_time)))
    print("total cost:")
    print(cost)
    print("total distance: ")
    print(total_dist)
    print("longest route distance: ")
    print(longest_route)

    # plot clusters
    delivery_x = []
    delivery_y = []
    delivery_colours = ['black', 'darkorange', 'red', 'saddlebrown', 'aquamarine', 'deepskyblue', 'slategrey', 'navy', 'mediumpurple', 'magenta']
    delivery_markers = [".", "o", "p", "P", "*", "d", "x", "v", "^", "s"]
    delivery_colour = []
    # plot truck routes
    truck_i = 0
    for truck in clustered_trucks:
        color = delivery_colours[truck_i]
        marker = delivery_markers[truck_i]
        truck_i = truck_i + 1
        for package in truck.packages:
            delivery_x.append(package.x)
            delivery_y.append(package.y)
            delivery_colour.append(color)
        figure_ax.scatter(delivery_x, delivery_y, marker=marker, color=delivery_colour, s=[30 for dummy in delivery_x], zorder=15)
        delivery_x = []
        delivery_y = []
        delivery_colour = []


        segs = []
        for i in range(0, len(truck.packages) - 1):
            n1 = truck.packages[i]
            n2 = truck.packages[i+1]
            # TODO: for every segment, run astar and plot the path
            if i == 0:
                temp_path, path_cost = pathPlanner.astar(start_node, n1)
                for j in range(0, len(temp_path) - 1):
                    n3 = temp_path[j]
                    n4 = temp_path[j+1]
                    segs.append(((n3.x, n3.y),(n4.x, n4.y)))
            if i == len(truck.packages) - 2:
                temp_path, path_cost = pathPlanner.astar(n2, start_node)
                for j in range(0, len(temp_path) - 1):
                    n3 = temp_path[j]
                    n4 = temp_path[j+1]
                    segs.append(((n3.x, n3.y),(n4.x, n4.y)))
            temp_path, path_cost = pathPlanner.astar(n1, n2)
            for j in range(0, len(temp_path) - 1):
                n3 = temp_path[j]
                n4 = temp_path[j+1]
                segs.append(((n3.x, n3.y),(n4.x, n4.y)))
        line_collection = LineCollection(segs, linewidths=[1.5], colors=to_rgb(color), zorder=10)
        figure_ax.add_collection(line_collection)

    print(time.time() - st)

    

    plt.show()

if __name__ == "__main__":
    main()
