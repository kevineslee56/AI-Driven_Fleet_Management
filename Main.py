from MapMaker import GenerateMap, PlotMap
from PathPlanner import astar
import time

import matplotlib.pyplot as plt

def main():
    nodes_list = GenerateMap(20000, 4, 0, 250, 0, 250)
    start_node, end_node = PlotMap(nodes_list)

    # find path between start and end selected nodes
    st = time.time()
    path = astar(start_node, end_node)
    print(time.time() - st)

    # plot path points
    node_x = [node.x for node in path]
    node_y = [node.y for node in path]
    plt.scatter(node_x, node_y, marker='.', color='red', zorder=10)

    plt.show()

if __name__ == "__main__":
    main()
