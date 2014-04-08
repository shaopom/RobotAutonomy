class Planner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize and hasattr(self.planning_env, 'InitializePlot')
        
    def Plan(self, start_config, goal_config):
        return None