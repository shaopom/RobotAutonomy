#from sets import Set    
from collections import deque #FIFO queue of BFS
from . import Planner
class BreadthFirstPlanner(Planner):
    def DoPlan(self, start_config, goal_config):
        num_vertex = 0
        plan = []

        start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_id  = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)

        Q = deque()
        V = set()              # import Set to check whether a node is visited or not
        Route = dict()         # use dict to record each node's parent node,
                               # in order to trace back to get the planned route.
                               # Route[end_id] = start_id

        Q.put(start_id)
        V.add(start_id)

        find_route = False

        while not Q.empty():
            node_id = Q.get()
            if node_id == goal_id:
                find_route = True
                break
            for successor_id in self.planning_env.GetSuccessors(node_id):
                if not successor_id in V:
                    num_vertex = num_vertex + 1
                    parent_config = self.planning_env.discrete_env.NodeIdToConfiguration(node_id)
                    child_config  = self.planning_env.discrete_env.NodeIdToConfiguration(successor_id)
                    self.planning_env.PlotEdge(parent_config, child_config)
                    V.add(successor_id)
                    Q.put(successor_id)
                    Route[successor_id] = node_id

        if not find_route:
            return []

        current_id = goal_id
        while current_id != start_id:
            current_config = self.planning_env.discrete_env.NodeIdToConfiguration(current_id)
            plan.append(current_config)
            current_id = Route[current_id]
        plan.append(start_config)
        plan = plan[::-1]

        return plan