from collections import deque
import numpy as np
from copy import deepcopy
from DiscreteEnvironment import DiscreteEnvironment
from operator import itemgetter

class AStarPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()


    def Plan(self, start_config, goal_config):
        
        plan = []
        print "astar"

        # TODO: Here you will implement the breadth first planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
    
        #call the planner
        final_queue_count = self.Astar(start_config,goal_config)
        
        #trace the path
        final_plan = self.finalplan(final_queue_count[0])
        

        print "hello"
   
        return final_plan


    def Astar(self,start_config,goal_config):
        goal= self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
        statenew = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
       

        count = 1

        last_visited = deque()
            
        parent = []
        child = deepcopy(statenew)

        #cost to come
        d_cost = self.planning_env.ComputeDistance(statenew,statenew)
        
        #cost to go (heuristic cost)
        h_cost = self.planning_env.ComputeDistance(statenew,goal)
        
        # f = g + h 
        # g = d_cost
        # h = h_cost
        cost = d_cost+h_cost

        main_queue = deque()
        main_queue.append([parent,child,cost,d_cost,h_cost])

        final_queue = []
        final_queue.append([parent,child,cost])

    
        while 1:
            # convert the queue in list
            main_queue_list = (list(main_queue))

            #sort the list on the basis of nearest node to goal and then total cost
            main_queue_list.sort(key=itemgetter(4,2))

            #clear the queue and extend the new list in this queue            
            main_queue.clear()
            main_queue.extend(main_queue_list)

            #pop the first element(node) of queue which will have least distance from goal and 
            #least total cost
            state_to_test = main_queue.popleft()

            #find the successors of the node
            successors = self.planning_env.GetSuccessors(state_to_test[1])

            if successors ==[]:
                last_visited.append(state_to_test[1])
                final_queue.append(state_to_test)
            
            else:
                last_visited.append(state_to_test[1])


                for x in xrange(0,len(successors)):
                    i = 0
                    #for every successor check if the node was already visited
                    for y in xrange(0,len(list(last_visited))):
                        if (successors[x] == last_visited[y]):
                            i = i+1
                            break

                    if i==0:
                        #if node not visited then check if it has reached goal
                        if (successors[x] == goal):
                            final_queue.append([state_to_test[1],successors[x]])
                            final_queue_submit = final_queue
                            count = count + 1
                            
                            #remove the comment from following line to get PlotEdge working for simple robot
                            # ONLY FOR SIMPLE ROBOT: comment the following line to make the code execution faster if not visualizing
                            # self.planning_env.PlotEdge(self.planning_env.discrete_env.NodeIdToConfiguration(successors[x]),self.planning_env.discrete_env.NodeIdToConfiguration(state_to_test[1]))
                            
                            #return the final path and number of nodes visited
                            return [final_queue_submit,count]
                            
                        #if node not visited and not goal, then compute the new cost
                        else:
                            d_cost = self.planning_env.ComputeDistance(successors[x],state_to_test[1])+state_to_test[3]
                            h_cost = self.planning_env.ComputeDistance(successors[x],goal)
                            cost = d_cost+h_cost
                            
                            #donot append the node in main queue which has same heuristic cost
                            #this avoids the number of nodes to check
                            if h_cost == state_to_test[4]:
                                last_visited.append(successors[x])
                                break
                            else:
                                #remove the comment from following line to get PlotEdge working for simple robot
                                # ONLY FOR SIMPLE ROBOT: comment the following line to make the code execution faster if not visualizing
                                # self.planning_env.PlotEdge(self.planning_env.discrete_env.NodeIdToConfiguration(successors[x]),self.planning_env.discrete_env.NodeIdToConfiguration(state_to_test[1]))
                                
                                main_queue.append([state_to_test[1],successors[x],round(cost,3),d_cost,h_cost])
                                final_queue.append([state_to_test[1],successors[x]])
                                last_visited.append(successors[x])
                                count = count + 1
                    else:
                        i==0
                        


    def finalplan(self,final_queue_count):
        # this function traces the path from end to start

        start = final_queue_count[0][1]
        end = final_queue_count[-1][1]

        final_queue_count.reverse()
        inverse_final_queue = deepcopy(final_queue_count)
        
        start_inverse_final_queue = inverse_final_queue[0]
        
        count = 0
        final=[]
        final.append(self.planning_env.discrete_env.NodeIdToConfiguration(end))

        while not (end == start):
            if start_inverse_final_queue[0]==inverse_final_queue[count][1]:
                final.append(self.planning_env.discrete_env.NodeIdToConfiguration(inverse_final_queue[count][1]))
                start_inverse_final_queue = inverse_final_queue[count]
                end = inverse_final_queue[count][1]
                count = count+1
                                
            else:
                count=count+1
        # reverse the new queue so as to get first element as start node and last as end node
        final.reverse()
        return final