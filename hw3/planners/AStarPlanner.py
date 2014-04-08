#from collections import deque
import numpy as np
from . import Planner
import time

class AStarPlanner(Planner):
    def Plan(self, start_config, goal_config):
        t0 = time.clock()
        num_vertex = 0
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
                path_length = 0
                for i in xrange(len(plan)-1):
                    path_length = path_length + self.planning_env.ComputeDistanceConfig(plan[i], plan[i+1])
                print "number of vertices: %d" %(num_vertex)
                print "path length: %f" %(path_length)
                print "plan time: %f" %(time.clock() - t0)
                return plan

            #find the nearest neighbors
            for neighbor in self.planning_env.GetSuccessors(current):
                num_vertex = num_vertex + 1
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
        print "number of vertices: %d" %(num_vertex)
        print "path length: 0, cannot find a path"
        print "plan time: %f" %(time.clock() - t0)
        return None