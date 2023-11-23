import time
import random
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import to_rgb
from matplotlib.collections import LineCollection

from MapMaker import GenerateMap, PlotMap
from PathPlanner import PathPlanner
from Solver import Solver
from Truck import Truck

def main():
    # Step 1: Generate map
    num_nodes = 100
    num_neighbours = 5
    x_min = 0
    x_max = 200
    y_min = 0
    y_max = 200
    nodes_list = GenerateMap(num_nodes, num_neighbours, x_min, x_max, y_min, y_max)

    # Step 2: Choose start node and plot map
    start_node, figure_ax = PlotMap(nodes_list)

    # Step 3: Choose delivery nodes
    num_deliveries = 10
    delivery_nodes = random.sample(nodes_list, num_deliveries)
    
    # Step 4: Initialize PathPlanner
    path_planner = PathPlanner(nodes_list)

    # Step 5: Dijkstra from start node to obtain paths from start node to delivery points
    path_planner.dijkstra(start_node)
    paths_from_start_to_delivery = {}
    for delivery_node in delivery_nodes:
        paths_from_start_to_delivery[delivery_node.id] = path_planner.get_path(start_node, delivery_node)
        
    # Step 6.0: Dijkstra from each delivery point to obtain paths from delivery point to other delivery points
    # Step 6.1 Send a truck to each delivery point
    paths_from_delivery_to_delivery = {}
    delivery_distances = []
    trucks = [Truck(i) for i in range(num_deliveries)]
    for i, delivery_node in enumerate(delivery_nodes):
        # Step 6.0
        path_planner.dijkstra(delivery_node)
        for j, other_delivery_node in enumerate(delivery_nodes):
            if j > i:
                path, cost = path_planner.get_path(delivery_node, other_delivery_node)
                paths_from_delivery_to_delivery[(delivery_node.id, other_delivery_node.id)] = (path, cost)
                paths_from_delivery_to_delivery[(other_delivery_node.id, delivery_node.id)] = (path, cost)
                delivery_distances.append((cost, delivery_node, other_delivery_node))
        
        # Step 6.1
        trucks[i].add_delivery_nodes([delivery_node])
        best_path, min_cost = path_planner.tsp(start_node, [delivery_node], paths_from_start_to_delivery, paths_from_delivery_to_delivery)
        trucks[i].update_path(best_path, min_cost)

    delivery_distances = sorted(delivery_distances)

    # Step 7: Calculate the initial best_cost
    weight_dist = 0.4
    weight_time = 1 - weight_dist
    best_cost = 0 # cost = weight1*sum_of_dist + weight2*max_dist(representing time)
    best_paths = []
    
    sum_of_dist = 0
    max_dist = 0
    for truck in trucks:
        sum_of_dist += truck.distance
        max_dist = max(max_dist, truck.distance)
        best_paths.append(truck.path)
    
    best_cost = weight_dist*sum_of_dist + weight_time*max_dist
    print("INITIAL COST: ", best_cost)

    # Step 8.0: Find the nearest pair of delivery points that are in different clusters
    # Step 8.1: Take the nearest pair and assign it to just one truck
    # Step 8.2: Calculate the cost of new path and compare to best_cost
    # Repeat Step 8 until only one truck is used
    min_dist_i = 0
    optimal_num_trucks = num_deliveries
    for i in range(num_deliveries-1, 0, -1):
        # Step 8.0
        cost, node1, node2 = delivery_distances[min_dist_i]
        while node1.truck == node2.truck and min_dist_i < len(delivery_distances):
            min_dist_i += 1
            cost, node1, node2 = delivery_distances[min_dist_i]

        if min_dist_i == len(delivery_distances):
            break

        # Step 8.1
        temp_truck_id = node2.truck.id
        node1.truck.add_delivery_nodes(node2.truck.delivery_nodes)
        trucks[temp_truck_id].clear()
        best_path, min_cost = path_planner.tsp(start_node, node1.truck.delivery_nodes, paths_from_start_to_delivery, paths_from_delivery_to_delivery)
        node1.truck.update_path(best_path, min_cost)

        # Step 8.2
        # Implement cost function and comparison
        sum_of_dist = 0
        max_dist = 0
        curr_paths = []
        for truck in trucks:
            sum_of_dist += truck.distance
            max_dist = max(max_dist, truck.distance)
            curr_paths.append(truck.path)

        curr_cost = 0.5*sum_of_dist + 0.5*max_dist
        if curr_cost < best_cost:
            best_cost = curr_cost
            best_paths = curr_paths
            optimal_num_trucks = i
    
    print("FINAL COST: ", best_cost)
    print("OPTIMAL NUM TRUCKS = ", optimal_num_trucks)

    # Step 9: Plot best paths
    delivery_x = []
    delivery_y = []
    delivery_colours = ['red', 'darkorange', 'yellow', 'aquamarine', 'deepskyblue', 'slategrey', 'navy', 'pink', 'indigo', 'palegreen']
    delivery_colour = []

    path_i = 0
    for path in best_paths:
        color = delivery_colours[path_i]
        path_i += 1
        for node in path:
            delivery_x.append(node.x)
            delivery_y.append(node.y)
            delivery_colour.append(color)

        segs = []
        for i in range(0, len(path) - 1):
            n1 = path[i]
            n2 = path[i+1]
            segs.append(((n1.x, n1.y),(n2.x, n2.y)))
        line_collection = LineCollection(segs, linewidths=[1], colors=to_rgb(color), zorder=10)
        figure_ax.add_collection(line_collection)

    figure_ax.scatter(delivery_x, delivery_y, marker='.', color=delivery_colour, s=[50 for dummy in delivery_x], zorder=15)

    # Step 10: Plot important nodes
    figure_ax.scatter(start_node.x, start_node.y, marker='.', color='black', s=100, zorder=15)

    delivery_x = [node.x for node in delivery_nodes]
    delivery_y = [node.y for node in delivery_nodes]
    figure_ax.scatter(delivery_x, delivery_y, marker='.', color='magenta', s=[100 for node in delivery_nodes], zorder=15)
    
    plt.show()

if __name__ == "__main__":
    main()
