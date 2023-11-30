import time
import math
import pickle
import matplotlib.pyplot as plt
from matplotlib.colors import to_rgb
from matplotlib.collections import LineCollection

from MapMaker import GenerateMap, PlotMap, choose_start
from PathPlanner import PathPlanner
from DeliveryPoints import DeliveryPoints, RandomDeliveryPoints
from Solver import Solver
from Truck import Truck
from MapReadWrite import read_existing_map, write_existing_map
from Config import Config

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


def main(conf: Config):
    # graph properties:
    USE_EXISTING_MAP = conf.USE_EXISTING_MAP
    map_loaded, nodes_list, start_node, deliveries, map_ranges = read_existing_map(USE_EXISTING_MAP)
    figure_ax = None
    if map_loaded:
        figure_ax = PlotMap(nodes_list)

    x_min = map_ranges[0]
    x_max = map_ranges[1]
    y_min = map_ranges[2]
    y_max = map_ranges[3]
    num_deliveries = 0

    if not map_loaded:
        num_nodes = conf.num_nodes
        num_neighbours = conf.num_neighbours
        x_min = conf.x_min
        x_max = conf.x_max
        y_min = conf.y_min
        y_max = conf.y_max
        nodes_list = GenerateMap(num_nodes, num_neighbours, x_min, x_max, y_min, y_max)
        figure_ax = PlotMap(nodes_list)
        start_node = choose_start(nodes_list)
        # problem/delivery properties:
        num_deliveries = conf.num_deliveries

        deliveries = DeliveryPoints(nodes_list) # allow user to choose delivery nodes
        if not deliveries: # if none chosen, use num_deliveries to randomly choose
            deliveries = RandomDeliveryPoints(nodes_list, num_deliveries, start_node)
    
    num_deliveries = len(deliveries)

    # save map
    write_existing_map(nodes_list, deliveries, start_node, [x_min, x_max, y_min, y_max])
    

    pathPlanner = PathPlanner(nodes_list)

    for delivery_node in deliveries:
        delivery_node.is_delivery = True
    # trucks
    num_trucks = conf.num_trucks
    trucks = []
    truck_capacity = conf.truck_capacity
    for i in range(0, num_trucks):
        trucks.append(Truck(i, truck_capacity))

    # solver properties:
    w_dist = conf.w_dist
    w_time = conf.w_time * (num_trucks/2) # approximate adjustment to account for total distance usually being a few times larger than the longest route
    initial_temp = conf.initial_temp
    stop_condition_1 = conf.stop_condition_1
    stop_condition_2 = conf.stop_condition_2
    max_iter = conf.max_iter 
    max_time_per_iter = conf.max_time_per_iter

    # plot "warehouse"
    figure_ax.scatter(start_node.x, start_node.y, marker='o', color='lime', s=50, zorder=15)

    # find solution:
    st = time.time()
    solver = Solver(start_node=start_node, nodes_list=nodes_list, deliveries=deliveries, trucks=trucks, x_range=[x_min, x_max], y_range=[y_min, y_max], w_dist=w_dist, w_time=w_time)

    # results required from solver
    clustered_trucks = []
    cost = -1
    total_dist = -1
    longest_route = -1

    # find list of lowest costs between all the delivery nodes
    print("Analyzing graph, getting distance data between deliveries, estimate " + str(0.1*num_deliveries) + "s.")
    cost_dict = solver.find_costs() # takes about 2 min for 1000 deliveries

    # condition for using brute force solver:
    if (len(nodes_list) <= 1000) and (len(deliveries) <= 10):
        clustered_trucks, cost, total_dist, longest_route = solver.brute_force(truck_capacity)
    # use simulated annealing:
    else:
        if len(deliveries) > 275:
            # first cluster, then TSP
            print("Running Solver (limited to " + str(max_time_per_iter) + " seconds max runtime)...")
            clustered_trucks, cost, total_dist, longest_route = solver.simulated_annealing(initial_temp, stop_condition_1, stop_condition_2, max_iter, max_time_per_iter, all_neighbours_flag=False)
        else:
            # just run simulated annealing normally
            print("Running Solver (limited to " + str(max_time_per_iter*2) + " seconds max runtime)...")
            clustered_trucks, cost, total_dist, longest_route = solver.simulated_annealing(initial_temp, stop_condition_1*(275/len(deliveries)), stop_condition_2*5, max_iter, max_time_per_iter*2, all_neighbours_flag=True)
    # sanity check of solution
    print(cost)
    print(cost_checker(start_node, clustered_trucks, cost_dict, w_dist, w_time))
    assert(math.isclose(cost, cost_checker(start_node, clustered_trucks, cost_dict, w_dist, w_time)))
    
    # solution results
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
    delivery_colour = []
    # plot truck routes
    truck_i = 0
    for truck in clustered_trucks:
        color = delivery_colours[truck_i]
        truck_i = truck_i + 1
        for package in truck.packages:
            delivery_x.append(package.x)
            delivery_y.append(package.y)
            delivery_colour.append(color)
        figure_ax.scatter(delivery_x, delivery_y, marker="o", color=delivery_colour, s=[30 for dummy in delivery_x], zorder=15)
        delivery_x = []
        delivery_y = []
        delivery_colour = []


        segs = []
        truck_node_order = [*truck.packages]
        truck_node_order.insert(0, start_node)
        truck_node_order.append(start_node)
        for i in range(0, len(truck_node_order)-1):
            n1 = truck_node_order[i]
            n2 = truck_node_order[i+1]
            temp_path, path_cost = pathPlanner.astar(n1, n2)
            for j in range(0, len(temp_path) - 1):
                n3 = temp_path[j]
                n4 = temp_path[j+1]
                segs.append(((n3.x, n3.y),(n4.x, n4.y)))
        line_collection = LineCollection(segs, linewidths=[1.5], colors=to_rgb(color), zorder=10)
        figure_ax.add_collection(line_collection)

    print("solution found in " + str(time.time() - st) + " seconds.")

    

    plt.show()

if __name__ == "__main__":
    config = Config()
    main(config)
