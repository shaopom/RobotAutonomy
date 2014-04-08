import time
from . import Planner
class BreadthFirstPlanner(Planner):
    def Plan(self, start_config, goal_config):
        
        t0 = time.clock()
        num_vertex = 0

        plan = []
       
        from sets import Set    
        import Queue           # import Queue to get the FIFO character of BFS

        if self.visualize:
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
                    num_vertex = num_vertex + 1
		    if self.visualize:
                    	parent_config = self.planning_env.discrete_env.NodeIdToConfiguration(node_id)
                    	child_config  = self.planning_env.discrete_env.NodeIdToConfiguration(successor_id)
                    	self.planning_env.PlotEdge(parent_config, child_config)
                    V.add(successor_id)
                    Q.put(successor_id)
                    Route[successor_id] = node_id

        print "number of vertices: %d" %(num_vertex)
        if not find_route:
            print "path length: 0"
            print "plan time: %f" %(time.clock() - t0)
            return []

        current_id = goal_id
        while current_id != start_id:
            current_config = self.planning_env.discrete_env.NodeIdToConfiguration(current_id)
            plan.append(current_config)
            current_id = Route[current_id]
        plan.append(start_config)
        plan = plan[::-1]
   
        path_length = 0
        for i in xrange(len(plan)-1):
            path_length = path_length + self.planning_env.ComputeDistanceConfig(plan[i], plan[i+1])
        print "path length: %f" %(path_length)
        print "plan time: %f" %(time.clock() - t0)
   
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
