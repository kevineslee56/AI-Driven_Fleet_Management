import time
import random
import matplotlib.pyplot as plt
from matplotlib.colors import to_rgb
from matplotlib.collections import LineCollection

from MapMaker import GenerateMap, PlotMap
from PathPlanner import PathPlanner
from DeliveryPoints import AddDeliveryPoints, DeliveryPoints
from Solver import Solver
from Truck import Truck

def main():
    num_nodes = 200 # 200000
    num_neighbours = 4 # 4
    x_min = 0
    x_max = 200 # 500
    y_min = 0
    y_max = 200 # 500
    nodes_list = GenerateMap(num_nodes, num_neighbours, x_min, x_max, y_min, y_max)

    start_node, figure_ax, figure_fig = PlotMap(nodes_list)
    #AddDeliveryPoints()

    num_deliveries = int(num_nodes/10)
    deliveries = DeliveryPoints(nodes_list)
    for delivery_node in deliveries:
        delivery_node.is_delivery = True

    pathPlanner = PathPlanner(nodes_list)

    # find path between start and end selected nodes
    print("A* to every delivery from start point:")
    st = time.time()
    pathPlanner.explore(start_node)
    print(time.time()-st)
    # for delivery_node in deliveries:
    #     path, cost = pathPlanner.astar(start_node, delivery_node)
    #     segs = []
    #     for i in range(0, len(path) - 1):
    #         n1 = path[i]
    #         n2 = path[i+1]
    #         segs.append(((n1.x, n1.y),(n2.x, n2.y)))
    #     line_collection = LineCollection(segs, linewidths=[2], colors=to_rgb('red'), zorder=10)
    #     figure_ax.add_collection(line_collection)
    # print(time.time() - st)

    # plot all delivery points at once
    # delivery_x = [node.x for node in deliveries]
    # delivery_y = [node.y for node in deliveries]
    # figure_ax.scatter(delivery_x, delivery_y, marker='.', color='magenta', s=[50 for node in deliveries], zorder=15)

    # plot "warehouse"
    figure_ax.scatter(start_node.x, start_node.y, marker='.', color='lime', s=50, zorder=15)

    # trucks
    num_trucks = 10
    trucks = []
    truck_capacity = 10000000
    for i in range(0, num_trucks):
        trucks.append(Truck(i, truck_capacity))

    st = time.time()
    #solver = Solver(start_node=start_node, nodes_list=nodes_list, deliveries=deliveries, trucks=trucks)
    print(time.time() - st)

    delivery_x = [node.x for node in deliveries]
    delivery_y = [node.y for node in deliveries]
    figure_ax.scatter(delivery_x, delivery_y, marker='.', color='magenta', s=[50 for node in deliveries], zorder=15)
    plt.show()

if __name__ == "__main__":
    main()
