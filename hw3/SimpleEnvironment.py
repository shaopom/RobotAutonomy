import numpy
import pylab as pl
import time
from DiscreteEnvironment import DiscreteEnvironment

class SimpleEnvironment(object):
    
    def __init__(self, herb, resolution):
        self.robot = herb.robot
        self.env = self.robot.GetEnv()
        self.lower_limits = [-5., -5.]
        self.upper_limits = [5., 5.]
        self.boundary_limits = [[-5., -5.], [5., 5.]]
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        # add an obstacle
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 1.5], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

        # goal sampling probability
        self.p = 0.0

    # This function returns the successors that are collision free, if no successors, return []
    def GetSuccessors(self, node_id):
	
    	successors = []

            # TODO: Here you will implement a function that looks
            #  up the configuration associated with the particular node_id
            #  and return a list of node_ids that represent the neighboring
            #  nodes

    	# if node_id is not collision free or is not in the boundary, directly return []
    	coord = self.discrete_env.NodeIdToGridCoord(node_id)
    	if not self.IsInBoundary(coord) or self.IsCollision(coord):
    		return successors

    	# get the successors that are obstacle free, and in the boundary
    	successors = self.GetNeighbor(coord)
        return successors


    def IsInBoundary(self, coord):
    	config = self.discrete_env.GridCoordToConfiguration(coord)
        if config[0] < self.lower_limits[0] or config[1] < self.lower_limits[0]:
    		return False
    	if config[0] > self.upper_limits[0] or config[1] > self.upper_limits[1]:
    		return False
    	return True
    	

    def IsCollision(self, coord):
    	config = self.discrete_env.GridCoordToConfiguration(coord)

    	init_T = self.robot.GetTransform()
    	env = self.robot.GetEnv()
    	# get the translation matrix based on the config
        #print init_T
        #print config
        #init_T[0][3] = config[0]
        #init_T[1][3] = config[1]
        #T = init_T
        T = numpy.array([[1,0,0,config[0]],[0,1,0,config[1]],[0,0,1,0],[0,0,0,1]])

        with env:
    		self.robot.SetTransform(T)
    	isCollision =  env.CheckCollision(self.robot)
    	with env:
    		self.robot.SetTransform(init_T)
    	return isCollision


    def GetNeighbor(self, coord):
    	successors_candidate = []
    	successors_candidate.append([coord[0]-1, coord[1]])
    	successors_candidate.append([coord[0]+1, coord[1]])
    	successors_candidate.append([coord[0], coord[1]-1])
    	successors_candidate.append([coord[0], coord[1]+1])

    	successors = []

    	for i in xrange(len(successors_candidate)):
    		if not self.IsCollision(successors_candidate[i]) and self.IsInBoundary(successors_candidate[i]):
    			successors.append(self.discrete_env.GridCoordToNodeId(successors_candidate[i]))
        return successors 


    def ComputeDistance(self, start_id, end_id):

        dist = 0

            # TODO: Here you will implement a function that 
            # computes the distance between the configurations given
            # by the two node ids
    	start_config = numpy.array(self.discrete_env.NodeIdToConfiguration(start_id))
    	end_config   = numpy.array(self.discrete_env.NodeIdToConfiguration(end_id))
        dist = numpy.linalg.norm(end_config - start_config)
        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids

	# here we use Manhattan distance. You can also use Euclidian distance. 
	# however, for a square grid map, the manhattan usually give a better result
    	start_config = self.discrete_env.NodeIdToConfiguration(start_id)
        goal_config   = self.discrete_env.NodeIdToConfiguration(goal_id)	
    	D = 1.5
    	dx = abs(start_config[0]-goal_config[0])
    	dy = abs(start_config[1]-goal_config[1])
    	cost = D*(dx+dy)
        return cost

    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        pl.xlim([self.lower_limits[0], self.upper_limits[0]])
        pl.ylim([self.lower_limits[1], self.upper_limits[1]])
        pl.plot(goal_config[0], goal_config[1], 'gx')

        # Show all obstacles in environment
        for b in self.robot.GetEnv().GetBodies():
            if b.GetName() == self.robot.GetName():
                continue
            bb = b.ComputeAABB()
            pl.plot([bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0]],
                    [bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1]], 'r')
                    
                     
        pl.ion()
        pl.show()
        
    def PlotEdge(self, sconfig, econfig, color = 'k.-'):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                color, linewidth=2.5)
        pl.draw()

    #########################################################################
    ################  Codes about hRRT ######################################
    #########################################################################

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p

    def GenerateRandomConfiguration(self):
        config = [0] * 2;
        lower_limits, upper_limits = self.boundary_limits

        # Save robot current transform
        init_transform = self.robot.GetTransform()
        res = init_transform[:2, 3]

        # try at most 100 times to find one random collision free config
        xs = numpy.random.uniform(lower_limits[0], upper_limits[0], 100)
        ys = numpy.random.uniform(lower_limits[1], upper_limits[1], 100)
        for n in xrange(100):
            # get uniform sampling in 2D configuration space
            config = [xs[n], ys[n]]

            # transform robot to that config in 6D (x,y,z,r,p,y) space
            # only set the x and y in that 4 by 4 transform matrix
            robot_pose = numpy.eye(4)
            robot_pose[:2, 3] = config
            with self.env:
                self.robot.SetTransform(robot_pose)

            # if this 2D random config sampling is collision free, then return this 2D config
            if not self.env.CheckCollision(self.robot):
                res = config
                break

        # Restore robot transform
        with self.env:
            self.robot.SetTransform(init_transform)

        # Return found random configuration
        # If no collision-free configuration found, then return robots position
        return numpy.array(res)

    def ComputeDistanceConfig(self, start_config, end_config):
        #
        # TODO: Implement a function which computes the distance between
        # two configurations
        #
        sconfig = numpy.array(start_config)
        econfig = numpy.array(end_config)
        return numpy.linalg.norm(econfig - sconfig)

    def Extend(self, start_config, end_config, num_points=500):

        #
        # TODO: Implement a function which attempts to extend from
        #   a start configuration to a goal configuration
        #

        #original transform
        orig_T = self.robot.GetTransform()

        # get the boundaries limits
        lower_limits, upper_limits = self.boundary_limits

        # unit distance between checking points
        # unit_dist = 0.02 #meters

        # number of checking points between start point and end point
        # num_points = numpy.floor(self.ComputeDistance(start_config, end_config)/unit_dist)

        config_increment = (end_config - start_config)/num_points

        check_config = start_config

        # check all the checking points
        for i in xrange(num_points):
            # check_config is the interpolation point of the start_config and end_config
            # based on the current check point
            check_config = start_config + config_increment * (i + 1)

            # check if the check_config is outside the boundaries
            if check_config[0] < lower_limits[0] or check_config[0] > upper_limits[0]:
                return check_config - config_increment
            if check_config[1] < lower_limits[1] or check_config[1] > upper_limits[1]:
                return check_config - config_increment

            # get the translation matrix based on the check_config
            T = numpy.array([[1, 0, 0, check_config[0]],
                            [0, 1, 0, check_config[1]],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])

            # always lock the environment first if you want to change the robot configuration
            with self.env:
                # set the robot to the new transfomation check_config to see whether it collides with
                # all the obstacles
                self.robot.SetTransform(T)

            # check whether the robot at checking points collides with the obstacles
            # run through all the obstacles
            if self.env.CheckCollision(self.robot):
                with self.env:
                    self.robot.SetTransform(orig_T)
                return check_config - config_increment

        with self.env:
            self.robot.SetTransform(orig_T)

        return check_config

    def PathLength(self, path):
        length = 0
        for i in range(len(path)-1):
            length += numpy.linalg.norm(path[i+1]-path[i])
        return length

    def ShortenPath(self, path, timeout=5.0, bisection=False):

        #
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the
        #  given timout (in seconds).
        #
        if (len(path) <= 2):
            return path

        print "Running path shortening..."
        print "Initial length = ", self.PathLength(path)

        num_check_points = 100
        start = time.clock()

        cont = True
        change = False
        i = 0
        while(cont):
            # Try to connect first and third points
            # skipping the second
            config_a = path[i]
            config_b = path[i+2]
            config_c = self.Extend(
                    config_a,
                    config_b,
                    num_check_points
                    )

            # If no collisions - remove second
            if (numpy.array_equal(config_b, config_c)):
                path.pop(i+1)
                change = True
            else:
                i += 1

            # Repeat until changes exist
            if (i >= (len(path)-2)):
                if change:
                    change = False
                    i = 0
                else:
                    cont = False

            cont = cont and ((time.clock() - start) < timeout)

        if (bisection):
            # If we are not done yet try to shorten more
            print "Running bisection shortening"
            cont = (time.clock() - start) < timeout
            change = False
            i = 0
            while(cont):
                config_a = path[i]
                config_b = path[i+1]
                config_c = path[i+2]
                config_p = numpy.median(numpy.array([config_a,config_b]),axis=0)
                config_q = numpy.median(numpy.array([config_b,config_c]),axis=0)
                config_r = self.Extend(
                        config_p,
                        config_q,
                        num_check_points
                        )

                if (numpy.array_equal(config_r,config_q)):
                    path[i+1] = config_p
                    path.insert(i+2,config_q)
                    change = True
                    i += 3
                else:
                    i += 1

                if (i >= (len(path)-2)):
                    if change:
                        change = False
                        i = 0
                    else:
                        cont = False

                cont = cont and ((time.clock() - start) < timeout)

        print "Shortened length = ", self.PathLength(path)

        return path

    def PlotPath(self,path):
        self.InitializePlot(path[-1])
        for i in range(len(path)-1):
            self.PlotEdge(path[i],path[i+1])
