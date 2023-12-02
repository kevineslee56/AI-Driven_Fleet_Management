# AI-Driven_Fleet_Management
Contributors: Jonathan Chiu, Eric Fu, Joyce Ke, Kevin Lee, Nicole Tao, Charlie Zheng

### Summary
This project focuses on the Vehicle Routing Problem (VRP) and implements two solvers, a brute force approach for small maps, and a simulated annealing approach for larger maps. A variety of other path planning algorithms such as Dijkstra and A* are implemented to aid with the preprocessing of the graph structure and presenting the results of the solvers.

## Introduction
This repo contains the implementations of the brute force solver as well as the simulated annealing solver and is run on a randomly generated map. There is a "hub" node and other delivery nodes. The output is the path each truck takes to do its assigned deliveries, ideally in an optimal or close to optimal manner.

### Features
- Randomly generated map
- Ability to save and load previously generated maps, and other problem parameters (hub location, delivery locations)
- Ability to choose delivery points on the map, or randomly generate a set number
- Tunable cost function
- Tunable simulated annealing solver parameters
- Interactable map for results (route lengths, sequence lengths)

## Getting Started
### Requirements
- Python 3.10+
- numpy, matplotlib

### Run
- Edit Config.py for custom configurations
- Note: changing the number of graph nodes and number of deliveries can change which solver is used
- Note: increasing the number of nodes and/or deliveries can take longer for a solution to be found
- Run Main.py