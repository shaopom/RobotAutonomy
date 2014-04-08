import time
class Planner(object):
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize and hasattr(self.planning_env, 'InitializePlot')
        self.path_length = 0.0
        self.node_count = 0
        self.plan_time = 0
        
    def Plan(self, start_config, goal_config):
        """planner wrapper function to abstract statistics away from individual planners"""
        if self.visualize:
            self.planning_env.InitializePlot(goal_config)

        start_time = time.clock()
        plan = self.DoPlan(start_config, goal_config)
        self.plan_time = time.clock() - start_time

        if plan: #non-empty path
            for i in range(len(plan)-1):
                self.path_length += self.planning_env.ComputeDistanceConfig(plan[i], plan[i+1])
            return plan
        else:
            return None #obvious planning failure

    def DoPlan(self, start_config, goal_config):
        return None

from AStarPlanner import AStarPlanner
from BreadthFirstPlanner import BreadthFirstPlanner
from DepthFirstPlanner import DepthFirstPlanner
from HeuristicRRTPlanner import HeuristicRRTPlanner