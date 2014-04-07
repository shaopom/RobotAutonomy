class BreadthFirstPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        
    def Plan(self, start_config, goal_config):
        
        plan = []

        # TODO: Here you will implement the breadth first planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space 
       
        from sets import Set    
        import Queue           # import Queue to get the FIFO character of BFS

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        
        start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_id  = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)

        Q = Queue.Queue()
        V = Set()              # import Set to check whether a node is visited or not
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

#Pseudo Code for BFS
#procedure BFS(G,v) is
#    create a queue Q
#    create a vector set V
#    enqueue v onto Q
#    add v to V
#    while Q is not empty loop
#       t := Q.dequeue()
#       if t is what we are looking for then
#          return t
#       end if
#       for all edges e in G.adjacentEdges(t) loop
#          u := G.adjacentVertex(t,e)
#          if u is not in V then
#              add u to V
#              enqueue u onto Q
#          end if
#       end loop
#    end loop
#    return none
#end BFS