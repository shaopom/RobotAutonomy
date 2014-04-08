from . import Planner
class DepthFirstPlanner(Planner):
    def DoPlan(self, start_config, goal_config):
        num_vertex = 0
        
        plan = []
        
        start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_id  = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)

        S = []                 # use List as Stack to get the LIFO character of DFS
        V = set()              # import Set to check whether a node is visited or not
        Route = dict()         # use dict to record each node's parent node,
                               # in order to trace back to get the planned route.
                               # Route[end_id] = start_id

        S.append(start_id)
        V.add(start_id)

        find_route = False

        while len(S) != 0:
            node_id = S.pop()
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
                    S.append(successor_id)
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