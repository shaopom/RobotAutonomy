#from collections import deque
import numpy as np
from . import Planner
import time

class AStarPlanner(Planner):
    def DoPlan(self, start_config, goal_config):
        plan = []
        goal= self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
        start = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)

        closed_set = set()
        open_set   = set()
        open_set.add(start)
        node_map = dict() # {node: parent node}

        g_cost = {start:0}                                                        #path cost to a given node, type(dict())
    	f_cost = {start:g_cost[start]+self.planning_env.ComputeHeuristicCost(start,goal)}
        while len(open_set) != 0:
            min_node   = 0
            min_f_cost = float("inf")
            for node in open_set:
                #print f_cost[node]
                if f_cost[node] < min_f_cost:
                    min_node = node
                    min_f_cost = f_cost[node]
            #print "min_node: %f" %(min_node)
            #print "min_f_cost: %f" %(min_f_cost)
            current = min_node
            open_set.remove(current)
            closed_set.add(current)

            #solution found, backtrack the path, and return it...
            if(current == goal):
                n = current
                while(n != start):
                    plan.insert(0, self.planning_env.discrete_env.NodeIdToConfiguration(n))
                    n = node_map[n]
                break

            #find the nearest neighbors
            for neighbor in self.planning_env.GetSuccessors(current):
                if neighbor in closed_set:
                    continue
                temp_g_cost = g_cost[current] + self.planning_env.ComputeDistance(current,neighbor)

                if (not (neighbor in open_set) or temp_g_cost < g_cost[neighbor]):
                    node_map[neighbor] = current
                    if self.visualize:
                        self.planning_env.PlotEdge(self.planning_env.discrete_env.NodeIdToConfiguration(current),self.planning_env.discrete_env.NodeIdToConfiguration(neighbor))
                    g_cost[neighbor] = temp_g_cost
                    f_cost[neighbor] = g_cost[neighbor] + self.planning_env.ComputeHeuristicCost(neighbor,goal)
                    if not (neighbor in open_set):
                        open_set.add(neighbor)

        #save metrics and return
        self.node_count = len(closed_set)
        return plan
