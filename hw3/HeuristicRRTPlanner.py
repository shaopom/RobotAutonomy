import numpy
from RRTTree import RRTTree

# Pseudocode for hRRT
# Data: q_start, q_goal
# Result: Path from start to goal
# initialize Tree(q_start)
# while q_goal is not in Tree do
#   q_rand = RandomSample()
#   q = NearestNeighbor(q_rand)
#   Compute mq
#   p = max(mq,p_min)
#   if RandomValue() < p then
#       Extend(q, q_rand)
#   end
# end
#
# mq = 1 - (c(q)-C_opt)/(C_max - C_opt)
# c(q) = C(q_start, q) + H(q,q_goal)
# C_opt = H(q_start,q_goal)
# C_max = max c(q), for q belong to Tree()

class HeuristicRRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize


    def Plan(self, start_config, goal_config, epsilon = 0.001):
        
        self.start_config = start_config
        self.goal_config  = goal_config

        self.start_id = self.planning_env.discrete_env.ConfigurationToNodeId(self.start_config)
        self.goal_id = self.planning_env.discrete_env.ConfigurationToNodeId(self.goal_config)

        # count for executing timing
        import time
        t0 = time.clock()
        
        tree = RRTTree(self.planning_env, start_config)
        plan = []
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        q_start = start_config
        q_goal  = goal_config

        # start the Heuristic RRT algorithm
        # set up the number of iterations we want to try before we give up
        num_iter = 2000
        # set up the probability to generate goal config
        prob_goal = 0.5

        # p_min is the threshold between exploration and exploitation
        # if p_min = 1, this will be just the simple Forward RRT
        p_min = 0.3
        p = 0
        self.distance = dict()
        self.distance[self.start_id] = 0

        # if it suceeeds to connect the goal, make isFail = False
        isFail = True
        import random
        num_vertex = 0

        for i in xrange(num_iter):
            # generate random configuration, q_rand
            if random.random() < prob_goal:
                q_rand = numpy.copy(q_goal)
            else:
                q_rand = self.planning_env.GenerateRandomConfiguration()
            
            # get the nearest neighbor from the Tree, based on q_rand
            mid, mdist = tree.GetNearestVertex(q_rand)

            # get the nearest neighbor, q
            q = tree.vertices[mid]

            # compute mq
            q_id = self.planning_env.discrete_env.ConfigurationToNodeId(q)
            mq = self.GetNodeQuality(q_id)

            if mq >= p_min:
                p = mq
            else:
                p = p_min

            if random.random() < p:
                # append config_n to the tree if config_n is addable
                # where config_n is the extended node between q and q_rand
                config_n = self.planning_env.Extend(q, q_rand)
                if numpy.array_equal(config_n, q):
                    continue
                else:
                    n_id = self.planning_env.discrete_env.ConfigurationToNodeId(config_n)
                    self.distance[n_id] = self.distance[q_id] + self.planning_env.ComputeDistanceConfig(q, config_n)
                    num_vertex = num_vertex + 1
                    tree.AddVertex(config_n)
                    tree.AddEdge(mid,len(tree.vertices)-1) # id of config_n is the last one in vertices
                    # draw the expended edge
                    if len(q) == 2 :
                        self.planning_env.PlotEdge(q, config_n)      

                # check if config_n equals to goal_config, if yes, break
                if numpy.array_equal(config_n, q_goal):
                    isFail = False
                    goal_id = len(tree.vertices)-1
                    break

        # recursive append the waypoints to the tree based on the tree.edges information
        # stop if we trace back to root, id = 0
        # start here
        
        print "number of vertices: %d" %(num_vertex)
        
        if isFail:
            print "path length: cannot make it in the amounted iterations!"
            print "plan time: %f" %(time.clock() - t0)
            plan.append(start_config)
            plan.append(goal_config)
            return plan
        else:
            current_id = goal_id
            while current_id != 0:
                plan.append(tree.vertices[current_id])
                current_id = tree.edges[current_id]
            plan.append(tree.vertices[current_id]) # add start config to the plan
        
        # reverse the order of plan
            plan = plan[::-1]
        
        path_length = 0
        for i in xrange(len(plan)-1):
            path_length = path_length + self.planning_env.ComputeDistanceConfig(plan[i], plan[i+1])
        print "path length: %f" %(path_length)
        print "plan time: %f" %(time.clock() - t0)
            
        return plan

    def GetNodeQuality(self, q_id):

        c_cost = self.distance[q_id]
        h_cost = self.planning_env.ComputeHeuristicCost(q_id, self.goal_id)
        c_opt  = self.planning_env.ComputeHeuristicCost(self.start_id, self.goal_id)
        c_max = 0
        for i in xrange(len(self.distance)):
            temp_cost = self.distance.values()[i] + self.planning_env.ComputeHeuristicCost(self.distance.keys()[i], self.goal_id)
            if temp_cost > c_max:
                c_max = temp_cost
        c = c_cost + h_cost
        if c_max == c_opt:
            mq = 1
        else:
            mq = 1 - (c-c_opt)/(c_max-c_opt)
        print "c: %f" %(c)
        print "c_cost: %f" %(c_cost)
        print "h_cost: %f" %(h_cost)
        print "c_opt: %f" %(c_opt)
        print "c_max: %f" %(c_max)
        print "mq: %f" %(mq)
        print "========================"
        return mq
