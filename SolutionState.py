
# functions for randomly generating solutions with simulated annealing
import random
from Truck import Truck
from Node import Node

import math

NEW_SOLN = 0
SAME_COST = 1
NO_NEW_SOLN = 2

class SolutionState:
    def __init__(self, routes, distances_dict, start_node: Node, w_dist = 0.5, w_time = 0.5):
        #initialize new state
        self.w_dist = w_dist
        self.w_time = w_time
        self.start_node = start_node
        self.routes = routes
        self.distances_dict = distances_dict
        self.route_costs = [float] * len(self.routes)
        self.total_distance_cost = 0
        self.longest_route_cost = 0
        self.calc_total_distance_cost()
        self.neighbour_soln_weights = [1.0] * 6

    def key(self, n1, n2):
        return n1.key + ":" + n2.key

    def get_route_cost(self, route):
        cost = 0
        prev_node = self.start_node
        for delivery in route:
            cost = cost + self.distances_dict[self.key(prev_node, delivery)]
            prev_node = delivery
        cost = cost + self.distances_dict[self.key(prev_node, self.start_node)]
        return cost
        
    def get_total_cost(self):
        total_cost = self.total_distance_cost * self.w_dist
        total_cost = total_cost + (self.w_time * self.longest_route_cost)
        return total_cost

    def calc_total_distance_cost(self):
        self.total_distance_cost = 0
        for key, route in self.routes.items():
            route_cost = self.get_route_cost(route)
            self.route_costs[int(key)] = route_cost
            self.total_distance_cost = self.total_distance_cost + route_cost
            if route_cost > self.longest_route_cost:
                self.longest_route_cost = route_cost

    # helper functions to get the nodes before and after a delivery
    def get_node_before(self, n, route):
        return self.start_node if (n - 1) < 0 else route[n - 1]
    
    def get_node_after(self, n, route):
        return self.start_node if (n + 1) >= len(route) else route[n + 1]

    # simulated annealing acceptance function
    def should_accept(self, new_cost, current_cost, current_temp):
        if math.isclose(new_cost, current_cost):
            return SAME_COST
        elif new_cost < current_cost:
            # better solution, accept
            return NEW_SOLN
        else:
            # worse solution, but accept with probability
            if random.uniform(0, 1) <= math.exp(((1/float(new_cost)) - (1/float(current_cost)))/float(current_temp)):
                # accept
                return NEW_SOLN
            else:
                return NO_NEW_SOLN

    # return difference in cost after attempting to generate a neighbour
    def create_neighbour_route(self, truck: Truck, current_temp):
        route = self.routes[truck.key]
        current_route_cost = self.route_costs[truck.id]
        #assert(math.isclose(current_route_cost, self.get_route_cost(route_copy)))
        new_route_cost = 0
        new_route = None
        
        result = None

        if len(route) < 2:
            print("oops")

        while result == None:
            choice = random.choice([0, 1, 2, 3])
            # inverse order of deliveries between 2 random packages
            if (choice == 0) and (len(route) >= 2):
                new_route_cost, result = self.inverse_subroute(route, current_route_cost, current_temp)
                self.route_costs[truck.id] = new_route_cost
            # swap
            elif (choice == 1) and (len(route) >= 2):
                new_route_cost, result = self.swap_deliveries(route, current_route_cost, current_temp)
                self.route_costs[truck.id] = new_route_cost
            # move
            elif (choice == 2) and (len(route) >= 2):
                new_route_cost, result = self.move_delivery(route, current_route_cost, current_temp)
                self.route_costs[truck.id] = new_route_cost
            # move subroute
            elif (choice == 3) and (len(route) >= 3):
                new_route_cost, result = self.move_subroute(route, current_route_cost, current_temp) 
                self.route_costs[truck.id] = new_route_cost
        
        if result == SAME_COST:
            return SAME_COST
        elif result == NEW_SOLN:
            if math.isclose(new_route_cost, self.longest_route_cost):
                self.longest_route_cost = new_route_cost
            elif new_route_cost > self.longest_route_cost:
                self.longest_route_cost = new_route_cost
            elif new_route_cost < self.longest_route_cost:
                self.longest_route_cost = max(self.route_costs)
            return NEW_SOLN
        else:
            return NO_NEW_SOLN
            
    def create_neighbour(self, trucks: [Truck], current_temp, all_neighbours_flag=True):
        current_total_cost = self.get_total_cost()
        result = None
        
        #choice = random.choices(population=[0, 1, 2, 3, 4, 5], weights=[float(i)/max(self.neighbour_soln_weights) for i in self.neighbour_soln_weights], k=1)
        choices = [0, 1, 2, 3, 5]
        if not all_neighbours_flag:
            choices = [5, 5, 5, 5, 5]

        while result == None:
            choice = random.choices(population=choices, weights=[20, 20, 20, 20, 50], k=1)
            choice = choice[0]
            # move 1 node from 1 route to another 
            if choice == 0:
                result = self.give_node(trucks, current_total_cost, current_temp)
            # move a subroute from 1 route to another
            elif choice == 1:
                subchoice = random.uniform(0, 1)
                result = self.give_subroute(trucks, current_total_cost, current_temp, subchoice < 0.5)
            # swap 1 node from 1 route with another
            elif choice == 2:
                result = self.swap_nodes(trucks, current_total_cost, current_temp)
            # swap subroutes (may be beneficial to determine an "optimal" subroute length)
            elif choice == 3:
                subchoice = random.uniform(0, 1)
                result = self.swap_subroutes(trucks, current_total_cost, current_temp, subchoice < 0.5)
            # split route into different lengths and randomly insert into other routes
            elif choice == 4:
                #result = self.split_route(trucks, current_total_cost, current_temp, True)
                result = None
            elif choice == 5 and (len([t1 for t1 in trucks if len(self.routes[t1.key]) >= 3]) > 0):
                truck = random.choice(trucks)
                while len(self.routes[truck.key]) < 3:
                    truck = random.choice(trucks)
                result = self.create_neighbour_route(truck, current_temp)

        if (result == NEW_SOLN) or (result == SAME_COST):
            self.neighbour_soln_weights[choice] = self.neighbour_soln_weights[choice] + 1
        return result


    def inverse_subroute(self, route, current_route_cost, current_temp):
        # select 2 random indices to get the subroute
        n1 = random.randrange(0, len(route))
        n2 = random.randrange(0, len(route))
        while n1 == n2:
            n2 = random.randrange(0, len(route))
        if n1 > n2:
            temp = n1
            n1 = n2
            n2 = temp
        before_n1_node = self.get_node_before(n1, route)
        after_n2_node = self.get_node_after(n2, route)
        
        new_cost = current_route_cost - self.distances_dict[self.key(before_n1_node, route[n1])] - self.distances_dict[self.key(route[n2], after_n2_node)]
        #route[n1:(n2+1)] = route[n1:(n2+1)][::-1]
        new_cost = new_cost + self.distances_dict[self.key(before_n1_node, route[n2])] + self.distances_dict[self.key(route[n1], after_n2_node)]

        result = self.should_accept(new_cost, current_route_cost, current_temp)
        if (result == NEW_SOLN) or (result == SAME_COST):
            route[n1:(n2+1)] = route[n1:(n2+1)][::-1]          
            '''# test
            test_cost = self.get_route_cost(route)
            if not math.isclose(new_cost, test_cost):
                print("inverse wrong cost")
                new_cost = test_cost'''
            self.total_distance_cost = self.total_distance_cost - current_route_cost + new_cost
            return new_cost, result
        # not accepted
        return current_route_cost, result

    def swap_deliveries(self, route, current_route_cost, current_temp):
        n1 = random.randrange(0, len(route))
        n2 = random.randrange(0, len(route))
        while n1 == n2:
            n2 = random.randrange(0, len(route))
        new_cost = current_route_cost

        new_cost = new_cost - self.distances_dict[self.key(self.get_node_before(n1, route), route[n1])]
        new_cost = new_cost - self.distances_dict[self.key(self.get_node_after(n1, route), route[n1])]
        new_cost = new_cost - self.distances_dict[self.key(self.get_node_before(n2, route), route[n2])]
        new_cost = new_cost - self.distances_dict[self.key(self.get_node_after(n2, route), route[n2])]
        route[n1], route[n2] = route[n2], route[n1]
        new_cost = new_cost + self.distances_dict[self.key(self.get_node_before(n1, route), route[n1])]
        new_cost = new_cost + self.distances_dict[self.key(self.get_node_after(n1, route), route[n1])]
        new_cost = new_cost + self.distances_dict[self.key(self.get_node_before(n2, route), route[n2])]
        new_cost = new_cost + self.distances_dict[self.key(self.get_node_after(n2, route), route[n2])]

        result = self.should_accept(new_cost, current_route_cost, current_temp)
        if (result == NEW_SOLN) or (result == SAME_COST):
            '''# test
            test_cost = self.get_route_cost(route)
            if not math.isclose(new_cost, test_cost):
                print("swap wrong cost")
                new_cost = test_cost'''
            self.total_distance_cost = self.total_distance_cost - current_route_cost + new_cost
            return new_cost, result
        # not accepted, swap back
        route[n1], route[n2] = route[n2], route[n1]
        return current_route_cost, result
    
    def move_delivery(self, route, current_route_cost, current_temp):
        n1 = random.randrange(0, len(route))
        new_cost = current_route_cost
        # adjust cost when removing node at n1 position:
        new_cost = new_cost - self.distances_dict[self.key(self.get_node_before(n1, route), route[n1])]
        new_cost = new_cost - self.distances_dict[self.key(self.get_node_after(n1, route), route[n1])]
        new_cost = new_cost + self.distances_dict[self.key(self.get_node_before(n1, route), self.get_node_after(n1, route))]

        # remove node at n1:
        temp_node = route.pop(n1)

        # find index to insert at
        n2 = random.randrange(0, len(route)+1)
        while n1 == n2:
            n2 = random.randrange(0, len(route)+1)
        
        node_before_n2 = self.get_node_before(n2, route)
        n2_node = self.get_node_after(n2-1, route)

        # adjust cost when inserting the node at n2 position:
        new_cost = new_cost - self.distances_dict[self.key(node_before_n2, n2_node)]
        new_cost = new_cost + self.distances_dict[self.key(node_before_n2, temp_node)]
        new_cost = new_cost + self.distances_dict[self.key(temp_node, n2_node)]

        result = self.should_accept(new_cost, current_route_cost, current_temp)
        if (result == NEW_SOLN) or (result == SAME_COST):
            # insert node at n2:
            route.insert(n2, temp_node)
            '''# test
            test_cost = self.get_route_cost(route)
            if not math.isclose(new_cost, test_cost):
                print("move wrong cost")
                new_cost = test_cost'''
            self.total_distance_cost = self.total_distance_cost - current_route_cost + new_cost
            return new_cost, result
        
        # not accepted, insert back at n1
        route.insert(n1, temp_node)
        
        return current_route_cost, result
        
    def move_subroute(self, route, current_route_cost, current_temp):
        # select 2 random indices to get the subroute
        n1 = random.randrange(0, len(route))
        n2 = random.randrange(0, len(route))
        while (n1 == n2) or (abs(n2 - n1) == (len(route) - 1)):
            n2 = random.randrange(0, len(route))
        if n1 > n2:
            temp = n1
            n1 = n2
            n2 = temp
        
        new_cost = current_route_cost
        # adjust cost when decoupling the subroute:
        node_before_n1 = self.get_node_before(n1, route)
        node_after_n2 = self.get_node_after(n2, route)
        new_cost = new_cost - self.distances_dict[self.key(node_before_n1, route[n1])]
        new_cost = new_cost - self.distances_dict[self.key(node_after_n2, route[n2])]
        new_cost = new_cost + self.distances_dict[self.key(node_before_n1, node_after_n2)]

        # decouple the subroute:
        subroute = route[n1:(n2+1)]
        route[n1:(n2+1)] = []

        # select n3 to insert into
        n3 = random.randrange(0, len(route) + 1)
        while n3 == n1:
            n3 = random.randrange(0, len(route) + 1)
        
        node_n3 = self.get_node_after(n3-1, route)
        node_before_n3 = self.get_node_before(n3, route)

        # adjust cost after adding subroute back at n3
        new_cost = new_cost - self.distances_dict[self.key(node_before_n3, node_n3)]
        new_cost = new_cost + self.distances_dict[self.key(node_before_n3, subroute[0])]
        new_cost = new_cost + self.distances_dict[self.key(node_n3, subroute[-1])]

        result = self.should_accept(new_cost, current_route_cost, current_temp)
        if (result == NEW_SOLN) or (result == SAME_COST):
            # insert subroute back at n3
            route[n3:n3] = subroute
            '''# test
            test_cost = self.get_route_cost(route)
            if not math.isclose(new_cost, test_cost):
                print("subroute wrong cost")
                new_cost = test_cost'''

            self.total_distance_cost = self.total_distance_cost - current_route_cost + new_cost
            return new_cost, result
        
        # not accepted, insert subroute back at n1
        route[n1:n1] = subroute
        return current_route_cost, result
    
    def give_node(self, trucks, current_total_cost, current_temp):
        routes = self.routes
        route_costs = self.route_costs
        # choose giver route:
        t1 = random.choice(trucks)
        while not routes[t1.key]: # while empty
            t1 = random.choice(trucks)
        giver_route = routes[t1.key]
        giver_route_cost = route_costs[t1.id]
        # choose receiver route (can be empty or have packages)
        t2 = random.choice(trucks)
        while t1 == t2:
            t2 = random.choice(trucks)
        receiver_route = routes[t2.key]
        receiver_route_cost = route_costs[t2.id]

        # choose package to move/give from t1 to t2
        n1 = random.randrange(0, len(giver_route))
        # choose where in t2 to insert package:
        n2 = random.randrange(0, len(receiver_route)+1)

        new_giver_route_cost = giver_route_cost
        # adjust cost when removing node at n1 position:
        new_giver_route_cost = new_giver_route_cost - self.distances_dict[self.key(self.get_node_before(n1, giver_route), giver_route[n1])]
        new_giver_route_cost = new_giver_route_cost - self.distances_dict[self.key(self.get_node_after(n1, giver_route), giver_route[n1])]
        new_giver_route_cost = new_giver_route_cost + self.distances_dict[self.key(self.get_node_before(n1, giver_route), self.get_node_after(n1, giver_route))]

        # remove node from giver route
        package = giver_route[n1]
        giver_route[n1:n1+1] = []
        receiver_route.insert(n2, package)

        new_receiver_route_cost = receiver_route_cost
        #adjust cost when adding node at n2 position in t2
        new_receiver_route_cost = new_receiver_route_cost - self.distances_dict[self.key(self.get_node_before(n2, receiver_route), self.get_node_after(n2, receiver_route))]
        new_receiver_route_cost = new_receiver_route_cost + self.distances_dict[self.key(self.get_node_before(n2, receiver_route), receiver_route[n2])]
        new_receiver_route_cost = new_receiver_route_cost + self.distances_dict[self.key(self.get_node_after(n2, receiver_route), receiver_route[n2])]
        
        '''# test
        test_cost_r1 = self.get_route_cost(giver_route)
        if not math.isclose(new_giver_route_cost, test_cost_r1):
            print("give node (r1) wrong cost")
            new_giver_route_cost = test_cost_r1
        test_cost_r2 = self.get_route_cost(receiver_route)
        if not math.isclose(new_receiver_route_cost, test_cost_r2):
            print("give node (r2) wrong cost")
            new_receiver_route_cost = test_cost_r2'''

        # update cost dict:
        route_costs[t1.id] = new_giver_route_cost
        route_costs[t2.id] = new_receiver_route_cost
        longest = max(route_costs)
        new_total_dist = self.total_distance_cost - giver_route_cost - receiver_route_cost + new_giver_route_cost + new_receiver_route_cost
        new_total_cost = (new_total_dist * self.w_dist) + (longest * self.w_time)
        result = self.should_accept(new_total_cost, current_total_cost, current_temp)
        if (result == NEW_SOLN) or (result == SAME_COST):            
            self.longest_route_cost = longest
            self.total_distance_cost = new_total_dist
            return result
        
        # not accepted, move back
        receiver_route[n2:n2+1] = []
        giver_route.insert(n1, package)
        route_costs[t1.id] = giver_route_cost
        route_costs[t2.id] = receiver_route_cost
        
        return result

    def give_subroute(self, trucks, current_total_cost, current_temp, inverted):
        routes = self.routes
        route_costs = self.route_costs
        if len([t1 for t1 in trucks if len(routes[t1.key]) >= 3]) < 1:
            return None

        # choose giver route:
        t1 = random.choice(trucks)
        # to form a subroute, need at least 3 elements
        while len(routes[t1.key]) < 3:
            t1 = random.choice(trucks)
        giver_route = routes[t1.key]
        giver_route_cost = route_costs[t1.id]

        # choose receiver route (can be empty or have packages)
        t2 = random.choice(trucks)
        while t1 == t2:
            t2 = random.choice(trucks)
        receiver_route = routes[t2.key]
        receiver_route_cost = route_costs[t2.id]

        # select 2 random indices to get the subroute to move
        n1 = random.randrange(0, len(giver_route))
        n2 = random.randrange(0, len(giver_route))
        while (n1 == n2) or (abs(n2 - n1) == (len(giver_route) - 1)):
            n2 = random.randrange(0, len(giver_route))
        if n1 > n2:
            temp = n1
            n1 = n2
            n2 = temp

        # get subroute
        subroute = giver_route[n1:(n2+1)]
        subroute_cost = 0
        for i in range(0, len(subroute) - 1):
            subroute_cost = subroute_cost + self.distances_dict[self.key(subroute[i], subroute[i+1])]
        
        new_giver_route_cost = giver_route_cost
        # adjust cost when decoupling the subroute:
        node_before_n1 = self.get_node_before(n1, giver_route)
        node_after_n2 = self.get_node_after(n2, giver_route)
        new_giver_route_cost = new_giver_route_cost - self.distances_dict[self.key(node_before_n1, subroute[0])]
        new_giver_route_cost = new_giver_route_cost - self.distances_dict[self.key(node_after_n2, subroute[-1])]
        new_giver_route_cost = new_giver_route_cost + self.distances_dict[self.key(node_before_n1, node_after_n2)]
        new_giver_route_cost = new_giver_route_cost - subroute_cost
        # select n3 to insert into
        n3 = random.randrange(0, len(receiver_route) + 1)
        
        node_n3 = self.get_node_after(n3-1, receiver_route)
        node_before_n3 = self.get_node_before(n3, receiver_route)

        if inverted:
            subroute.reverse()

        new_receiver_route_cost = receiver_route_cost
        # adjust cost after adding subroute back
        new_receiver_route_cost = new_receiver_route_cost - self.distances_dict[self.key(node_before_n3, node_n3)]
        new_receiver_route_cost = new_receiver_route_cost + self.distances_dict[self.key(node_before_n3, subroute[0])]
        new_receiver_route_cost = new_receiver_route_cost + self.distances_dict[self.key(node_n3, subroute[-1])]
        new_receiver_route_cost = new_receiver_route_cost + subroute_cost

        # update cost dict:
        route_costs[t1.id] = new_giver_route_cost
        route_costs[t2.id] = new_receiver_route_cost

        longest = max(route_costs)
        new_total_dist = self.total_distance_cost - giver_route_cost - receiver_route_cost + new_giver_route_cost + new_receiver_route_cost
        new_total_cost = (new_total_dist * self.w_dist) + (longest * self.w_time)
        result = self.should_accept(new_total_cost, current_total_cost, current_temp)
        if (result == NEW_SOLN) or (result == SAME_COST):
            giver_route[n1:(n2+1)] = []
            receiver_route[n3:n3] = subroute

            '''# test
            test_cost_r1 = self.get_route_cost(giver_route)
            if not math.isclose(new_giver_route_cost, test_cost_r1):
                print("give subroute (r1) wrong cost")
                new_giver_route_cost = test_cost_r1
            test_cost_r2 = self.get_route_cost(receiver_route)
            if not math.isclose(new_receiver_route_cost, test_cost_r2):
                print("give subroute (r2) wrong cost")
                new_receiver_route_cost = test_cost_r2
            route_costs[t1.id] = new_giver_route_cost
            route_costs[t2.id] = new_receiver_route_cost'''

            self.longest_route_cost = longest
            self.total_distance_cost = new_total_dist
            return result
        
        # not accepted, revert costs
        route_costs[t1.id] = giver_route_cost
        route_costs[t2.id] = receiver_route_cost
        
        return result
    
    def swap_nodes(self, trucks, current_total_cost, current_temp):
        routes = self.routes
        route_costs = self.route_costs
        if len([t1 for t1 in trucks if len(routes[t1.key]) >= 1]) < 2:
            return None

        # choose first route:
        t1 = random.choice(trucks)
        while not routes[t1.key]: # while empty
            t1 = random.choice(trucks)
        r1 = routes[t1.key]
        r1_cost = route_costs[t1.id]
        # choose second route
        t2 = random.choice(trucks)
        while (t1 == t2) or (not routes[t2.key]): # while empty or same as t1
            t2 = random.choice(trucks)
        r2 = routes[t2.key]
        r2_cost = route_costs[t2.id]

        # find which indices to swap
        n1 = random.randrange(0, len(r1))
        n2 = random.randrange(0, len(r2))

        new_r1_cost = r1_cost
        new_r2_cost = r2_cost

        node_before_n1 = self.get_node_before(n1, r1)
        node_after_n1 = self.get_node_after(n1, r1)
        node_before_n2 = self.get_node_before(n2, r2)
        node_after_n2 = self.get_node_after(n2, r2)
        # adjust cost when removing nodes at n1 and n2 position:
        new_r1_cost = new_r1_cost - self.distances_dict[self.key(node_before_n1, r1[n1])]
        new_r1_cost = new_r1_cost - self.distances_dict[self.key(node_after_n1, r1[n1])]
        new_r2_cost = new_r2_cost - self.distances_dict[self.key(node_before_n2, r2[n2])]
        new_r2_cost = new_r2_cost - self.distances_dict[self.key(node_after_n2, r2[n2])]

        # adjust cost when adding nodes at n1 and n2 position:
        new_r1_cost = new_r1_cost + self.distances_dict[self.key(node_before_n1, r2[n2])]
        new_r1_cost = new_r1_cost + self.distances_dict[self.key(node_after_n1, r2[n2])]
        new_r2_cost = new_r2_cost + self.distances_dict[self.key(node_before_n2, r1[n1])]
        new_r2_cost = new_r2_cost + self.distances_dict[self.key(node_after_n2, r1[n1])]

        # update cost dict:
        route_costs[t1.id] = new_r1_cost
        route_costs[t2.id] = new_r2_cost
        longest = max(route_costs)
        new_total_dist = self.total_distance_cost - r1_cost - r2_cost + new_r1_cost + new_r2_cost
        new_total_cost = (new_total_dist * self.w_dist) + (longest * self.w_time)
        result = self.should_accept(new_total_cost, current_total_cost, current_temp)
        if (result == NEW_SOLN) or (result == SAME_COST):    
            r1[n1], r2[n2] = r2[n2], r1[n1]

            '''# test
            test_cost_r1 = self.get_route_cost(r1)
            if not math.isclose(new_r1_cost, test_cost_r1):
                print("give subroute (r1) wrong cost")
                new_r1_cost = test_cost_r1
            test_cost_r2 = self.get_route_cost(r2)
            if not math.isclose(new_r2_cost, test_cost_r2):
                print("give subroute (r2) wrong cost")
                new_r2_cost = test_cost_r2
            route_costs[t1.id] = new_r1_cost
            route_costs[t2.id] = new_r2_cost'''

            self.longest_route_cost = longest
            self.total_distance_cost = new_total_dist
            return result
        
        # not accepted
        route_costs[t1.id] = r1_cost
        route_costs[t2.id] = r2_cost
        
        return result
    
    def swap_subroutes(self, trucks, current_total_cost, current_temp, inverted):
        routes = self.routes
        route_costs = self.route_costs
        if len([t1 for t1 in trucks if len(routes[t1.key]) >= 3]) < 2:
            return None

        # choose first route:
        t1 = random.choice(trucks)
        while len(routes[t1.key]) < 3: # needs at least 3 nodes to get a subroute
            t1 = random.choice(trucks)
        r1 = routes[t1.key]
        r1_cost = route_costs[t1.id]
        # choose second route
        t2 = random.choice(trucks)
        while (t1 == t2) or (len(routes[t2.key]) < 3): # while less than 3 nodes or same as t1
            t2 = random.choice(trucks)
        r2 = routes[t2.key]
        r2_cost = route_costs[t2.id]

        # get subroute from r1
        # select 2 random indices to get the subroute to move
        n1 = random.randrange(0, len(r1))
        n2 = random.randrange(0, len(r1))
        while (n1 == n2) or (abs(n2 - n1) == (len(r1) - 1)):
            n2 = random.randrange(0, len(r1))
        if n1 > n2:
            temp = n1
            n1 = n2
            n2 = temp

        # get the first subroute and its cost and decouple it
        subroute1 = r1[n1:(n2+1)]
        subroute1_cost = 0
        for i in range(0, len(subroute1) - 1):
            subroute1_cost = subroute1_cost + self.distances_dict[self.key(subroute1[i], subroute1[i+1])]
        
        new_r1_cost = r1_cost
        # adjust cost when decoupling the subroute:
        node_before_n1 = self.get_node_before(n1, r1)
        node_after_n2 = self.get_node_after(n2, r1)
        new_r1_cost = new_r1_cost - self.distances_dict[self.key(node_before_n1, subroute1[0])]
        new_r1_cost = new_r1_cost - self.distances_dict[self.key(node_after_n2, subroute1[-1])]
        new_r1_cost = new_r1_cost + self.distances_dict[self.key(node_before_n1, node_after_n2)]
        new_r1_cost = new_r1_cost - subroute1_cost

        # get subroute from r2
        # select 2 random indices to get the subroute to move
        n3 = random.randrange(0, len(r2))
        n4 = random.randrange(0, len(r2))
        while (n3 == n4) or (abs(n4 - n3) == (len(r2) - 1)):
            n4 = random.randrange(0, len(r2))
        if n3 > n4:
            temp = n3
            n3 = n4
            n4 = temp

        # get the second subroute and its cost and decouple it
        subroute2 = r2[n3:(n4+1)]
        subroute2_cost = 0
        for i in range(0, len(subroute2) - 1):
            subroute2_cost = subroute2_cost + self.distances_dict[self.key(subroute2[i], subroute2[i+1])]
        
        new_r2_cost = r2_cost
        # adjust cost when decoupling the subroute:
        node_before_n3 = self.get_node_before(n3, r2)
        node_after_n4 = self.get_node_after(n4, r2)
        new_r2_cost = new_r2_cost - self.distances_dict[self.key(node_before_n3, subroute2[0])]
        new_r2_cost = new_r2_cost - self.distances_dict[self.key(node_after_n4, subroute2[-1])]
        new_r2_cost = new_r2_cost + self.distances_dict[self.key(node_before_n3, node_after_n4)]
        new_r2_cost = new_r2_cost - subroute2_cost

        if inverted:
            subroute1.reverse()
            subroute2.reverse()
        
        # decouple subroutes:
        r1[n1:(n2+1)] = []
        r2[n3:(n4+1)] = []

        # find locations in the lists to insert
        n5 = random.randrange(0, len(r1)+1)
        n6 = random.randrange(0, len(r2)+1)

        # adjust cost of adding subroute2 to r1
        node_before_n5 = self.get_node_before(n5, r1)
        n5_node = self.get_node_after(n5-1, r1)
        new_r1_cost = new_r1_cost - self.distances_dict[self.key(node_before_n5, n5_node)]
        new_r1_cost = new_r1_cost + self.distances_dict[self.key(node_before_n5, subroute2[0])]
        new_r1_cost = new_r1_cost + self.distances_dict[self.key(subroute2[-1], n5_node)]
        new_r1_cost = new_r1_cost + subroute2_cost

        # adjust cost of adding subroute1 to r2
        node_before_n6 = self.get_node_before(n6, r2)
        n6_node = self.get_node_after(n6-1, r2)
        new_r2_cost = new_r2_cost - self.distances_dict[self.key(node_before_n6, n6_node)]
        new_r2_cost = new_r2_cost + self.distances_dict[self.key(node_before_n6, subroute1[0])]
        new_r2_cost = new_r2_cost + self.distances_dict[self.key(subroute1[-1], n6_node)]
        new_r2_cost = new_r2_cost + subroute1_cost

        # update cost dict:
        route_costs[t1.id] = new_r1_cost
        route_costs[t2.id] = new_r2_cost

        longest = max(route_costs)
        new_total_dist = self.total_distance_cost - r1_cost - r2_cost + new_r1_cost + new_r2_cost
        new_total_cost = (new_total_dist * self.w_dist) + (longest * self.w_time)
        result = self.should_accept(new_total_cost, current_total_cost, current_temp)
        if (result == NEW_SOLN) or (result == SAME_COST):
            # insert subroutes
            r1[n5:n5] = subroute2
            r2[n6:n6] = subroute1

            '''# test
            test_cost_r1 = self.get_route_cost(r1)
            if not math.isclose(new_r1_cost, test_cost_r1):
                print("swap subroute (r1) wrong cost")
                new_r1_cost = test_cost_r1
            test_cost_r2 = self.get_route_cost(r2)
            if not math.isclose(new_r2_cost, test_cost_r2):
                print("swap subroute (r2) wrong cost")
                new_r2_cost = test_cost_r2
            route_costs[t1.id] = new_r1_cost
            route_costs[t2.id] = new_r2_cost'''

            self.longest_route_cost = longest
            self.total_distance_cost = new_total_dist
            return result
        
        # not accepted, revert costs, add subroutes back where they came from (after also uninverting)
        if inverted:
            subroute1.reverse()
            subroute2.reverse()
        r1[n1:n1] = subroute1
        r2[n3:n3] = subroute2
        route_costs[t1.id] = r1_cost
        route_costs[t2.id] = r2_cost

        return result
    
    def split_route(self, trucks, routes, route_costs):
        pass

        

        