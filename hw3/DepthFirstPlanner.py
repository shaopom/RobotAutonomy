from collections import deque
import numpy as np
from copy import deepcopy
from DiscreteEnvironment import DiscreteEnvironment
import time

class DepthFirstPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()
        self.running_time = 0


    def Plan(self, start_config, goal_config):
        
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
       

        plan = []

        # TODO: Here you will implement the breadth first planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
    
        final_queue_count = self.DFS(start_config,goal_config)
        
        final_plan = self.finalplan(final_queue_count[0])
        
        return final_plan


    def DFS(self,start_config,goal_config):
        goal= self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
        statenew = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)

        count = 1

        last_visited = deque()
            
        parent = []
        child = deepcopy(statenew)

        main_queue = deque()
        main_queue.append([parent,child])

        final_queue = []
        final_queue.append([parent,child])

        original = time.time()


        while 1:

            state_to_test = main_queue.popleft()

            successors = self.planning_env.GetSuccessors(state_to_test[1])

            

            if successors ==[]:
                last_visited.append(state_to_test[1])
                final_queue.append(state_to_test)
            
            else:
                last_visited.append(state_to_test[1])

                for x in xrange(0,len(successors)):
                    i = 0
                    for y in xrange(0,len(list(last_visited))):
                        if (successors[x] == last_visited[y]):
                            i = i+1
                            break

                    if i==0:
                        if not self.IsCollision(self.planning_env.discrete_env.NodeIdToGridCoord(successors[x])) and self.IsInBoundary(self.planning_env.discrete_env.NodeIdToGridCoord(successors[x])):
                            if (successors[x] == goal):
                                main_queue.append([state_to_test[1],successors[x]])
                                final_queue.append([state_to_test[1],successors[x]])
                                final_queue_submit = final_queue
                                count = count + 1


                                # comment the following line to make the code execution faster if not visualizing
                                if self.visualize:
                                    self.planning_env.PlotEdge(self.planning_env.discrete_env.NodeIdToConfiguration(successors[x]),self.planning_env.discrete_env.NodeIdToConfiguration(state_to_test[1]))
                                
                                end = time.time()
                                self.running_time = end-original
                                #print "time", end-original
                                return [final_queue_submit,count]
                                
                            else:
                                main_queue_encore = deepcopy(main_queue)
                                main_queue.clear()
                                main_queue.append([state_to_test[1],successors[x]])
                                main_queue.extend(main_queue_encore)
                                final_queue.append([state_to_test[1],successors[x]])
                                count = count + 1
                            

                                
                                # main_queue.append([state_to_test[1],successors[x]])
                                # final_queue.append([state_to_test[1],successors[x]])
                                # count = count + 1
                                last_visited.append(successors[x])

                                # comment the following line to make the code execution faster if not visualizing
                                if self.visualize:
                                    self.planning_env.PlotEdge(self.planning_env.discrete_env.NodeIdToConfiguration(successors[x]),self.planning_env.discrete_env.NodeIdToConfiguration(state_to_test[1]))
                    else:
                        i==0


        # while 1:

        #     state_to_test = main_queue.popleft()

        #     successors = self.planning_env.GetSuccessors(state_to_test[1])

        #     if successors ==[]:
        #         last_visited.append(state_to_test[1])
        #         final_queue.append(state_to_test)
            
        #     else:
        #         last_visited.append(state_to_test[1])

        #         for x in xrange(0,len(successors)):
        #             i = 0
        #             for y in xrange(0,len(list(last_visited))):
        #                 if (successors[x] == last_visited[y]):
        #                     i = i+1
        #                     break

        #             if i==0:
        #                 if (successors[x] == goal):
        #                     final_queue.append([state_to_test[1],successors[x]])
        #                     final_queue_submit = final_queue
        #                     count = count + 1

        #                     # comment the following line to make the code execution faster if not visualizing
        #                     self.planning_env.PlotEdge(self.planning_env.discrete_env.NodeIdToConfiguration(successors[x]),self.planning_env.discrete_env.NodeIdToConfiguration(state_to_test[1]))
                            
        #                     end = time.time()
        #                     print "time", end-original
        #                     return [final_queue_submit,count]
                            
        #                 else:
        #                     main_queue_encore = deepcopy(main_queue)
        #                     main_queue.clear()
        #                     main_queue.append([state_to_test[1],successors[x]])
        #                     main_queue.extend(main_queue_encore)
        #                     final_queue.append([state_to_test[1],successors[x]])
        #                     count = count + 1
                            
        #                     # comment the following line to make the code execution faster if  not visualizing
        #                     self.planning_env.PlotEdge(self.planning_env.discrete_env.NodeIdToConfiguration(successors[x]),self.planning_env.discrete_env.NodeIdToConfiguration(state_to_test[1]))
        #             else:
        #                 i==0



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

        distance =0

        while not (end == start):
            if start_inverse_final_queue[0]==inverse_final_queue[count][1]:
                
                distance = distance + self.planning_env.ComputeDistance(inverse_final_queue[count][1],self.planning_env.discrete_env.ConfigurationToNodeId(final[-1]))
                
                final.append(self.planning_env.discrete_env.NodeIdToConfiguration(inverse_final_queue[count][1]))
                start_inverse_final_queue = inverse_final_queue[count]
                end = inverse_final_queue[count][1]
                count = count+1
                                
            else:
                count=count+1
        # reverse the new queue so as to get first element as start node and last as end node
        final.reverse()
        print "time", self.running_time
        print "distance", distance, "number of nodes ", count
        return final


    def IsInBoundary(self, coord):
        config = self.planning_env.discrete_env.GridCoordToConfiguration(coord)
        if config[0] < self.planning_env.lower_limits[0] or config[1] < self.planning_env.lower_limits[0]:
            return False
        if config[0] > self.planning_env.upper_limits[0] or config[1] > self.planning_env.upper_limits[1]:
            return False
        return True

    def IsCollision(self, coord):
        config = self.planning_env.discrete_env.GridCoordToConfiguration(coord)

        init_T = self.planning_env.robot.GetTransform()
        env = self.planning_env.robot.GetEnv()
        # get the translation matrix based on the config
        #print init_T
        #print config
        init_T[0][3] = config[0]
        init_T[1][3] = config[1]
        T = init_T
        #T = numpy.array([[1,0,0,config[0]],[0,1,0,config[1]],[0,0,1,0],[0,0,0,1]])

        with env:
            self.planning_env.robot.SetTransform(T)
        isCollision =  env.CheckCollision(self.planning_env.robot)
        with env:
            self.planning_env.robot.SetTransform(init_T)
        return isCollision
