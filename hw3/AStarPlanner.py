#from collections import deque
import numpy as np
from copy import deepcopy
from DiscreteEnvironment import DiscreteEnvironment
from Planner import Planner
from operator import itemgetter
import time

class AStarPlanner(Planner):
    def Plan(self, start_config, goal_config):
        plan = []
        goal= self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
        start = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)

        if self.visualize:
            self.planning_env.InitializePlot(goal_config)

        closed_nodes = []
        open_nodes = [start]
        node_map = dict()

        path_cost = {start:0}                                   #path cost to a given node
        h_cost = {start:self.planning_env.ComputeDistance(start,goal)}       #heuristic cost
        while open_nodes:
            open_nodes.sort(key=lambda node: h_cost[node], reverse=True) #sort list by heuristic cost
            current = open_nodes.pop()
            closed_nodes.append(current)

            #solution found, backtrack the path, and return it...
            if(current == goal):
                n = current
                while(n != start):
                    plan.insert(0, self.planning_env.discrete_env.NodeIdToConfiguration(n))
                    n = node_map[n]
                return plan

            #find the nearest neighbors
            for neighbor in self.planning_env.GetSuccessors(current):
                if neighbor in closed_nodes:
                    continue
                p_cost = path_cost[current] + self.planning_env.ComputeDistance(current,neighbor)

                if (not (neighbor in path_cost) or p_cost < path_cost[neighbor]):
                    node_map[neighbor] = current
                    if self.visualize:
                        self.planning_env.PlotEdge(self.planning_env.discrete_env.NodeIdToConfiguration(current),self.planning_env.discrete_env.NodeIdToConfiguration(neighbor))
                    path_cost[neighbor] = p_cost
                    h_cost[neighbor] = p_cost + self.planning_env.ComputeDistance(neighbor,goal)
                    if not (neighbor in open_nodes):
                        open_nodes.append(neighbor)
        return None