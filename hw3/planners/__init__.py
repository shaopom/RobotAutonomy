import time
class Planner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize and hasattr(self.planning_env, 'InitializePlot')
        self.path_length = 0
        self.node_count = 0
        self.plan_time = 0
        
    def Plan(self, start_config, goal_config):
    	"""planner wrapper function to abstract statistics away from individual planners"""
    	start_time = time.clock()
    	plan = self.DoPlan(start_config, goal_config)
    	self.plan_time = time.clock() - start_time

    	self.path_length = len(plan)				#is this supposed to be environment length?
        return plan

    def DoPlan(self, start_config, goal_config):
        return None

from AStarPlanner import AStarPlanner
from BreadthFirstPlanner import BreadthFirstPlanner
from DepthFirstPlanner import DepthFirstPlanner
from HeuristicRRTPlanner import HeuristicRRTPlanner