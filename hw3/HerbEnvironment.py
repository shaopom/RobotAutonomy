import numpy
import time
from DiscreteEnvironment import DiscreteEnvironment

class HerbEnvironment(object):
    
    def __init__(self, herb, resolution):
        
        self.robot = herb.robot
        self.lower_limits, self.upper_limits = self.robot.GetActiveDOFLimits()
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        self.env = self.robot.GetEnv()

        # account for the fact that snapping to the middle of the grid cell may put us over our
        #  upper limit
        upper_coord = [x - 1 for x in self.discrete_env.num_cells]
        upper_config = self.discrete_env.GridCoordToConfiguration(upper_coord)
        for idx in range(len(upper_config)):
            self.discrete_env.num_cells[idx] -= 1

        # add a table and move the robot into place
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 0.7], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)
        
        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)

        # goal sampling probability
        self.p = 0.0
    
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

        for i in xrange(len(coord)):
            if coord[i] == 0 or coord[i] == self.discrete_env.num_cells[i]:
                return False
        return True

        #config = self.discrete_env.GridCoordToConfiguration(coord)
	#for i in xrange(len(self.lower_limits)):
		#if config[i] < self.lower_limits[i] or config[i] > self.upper_limits[i]:
			#return False
        #return True


    def IsCollision(self, coord):

        config = self.discrete_env.GridCoordToConfiguration(coord)
	activeDOFIndices = self.robot.GetActiveDOFIndices()
        init_config = self.robot.GetActiveDOFValues()
        env = self.robot.GetEnv()

        with env:
                self.robot.SetDOFValues(config, activeDOFIndices)
        isCollision =  env.CheckCollision(self.robot)
        with env:
                self.robot.SetDOFValues(init_config, activeDOFIndices)
        return isCollision


    def GetNeighbor(self, coord):

        successors_candidate = []
	# generate a list of neighbors by change only one joint value at a time
	for i in xrange(self.robot.GetActiveDOF()):
		temp_coord = list(coord)
		temp_coord[i] = temp_coord[i]+1
		successors_candidate.append(temp_coord)
		temp_coord = list(coord)
		temp_coord[i] = temp_coord[i]-1
		successors_candidate.append(temp_coord)

        successors = []

        for i in xrange(len(successors_candidate)):
                if self.IsInBoundary(successors_candidate[i]) and not self.IsCollision(successors_candidate[i]):
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
        goal_config  = self.discrete_env.NodeIdToConfiguration(goal_id)
        D = 1
	for i in xrange(len(start_config)):
        	delta = abs(start_config[i]-goal_config[i])
        	cost = cost + D*delta
        return cost

    #########################################################################
    ################  Codes about hRRT ######################################
    #########################################################################

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p

    def GenerateRandomConfiguration(self):
        activeDOFIndices = self.robot.GetActiveDOFIndices()
        config = [0] * len(activeDOFIndices)

        #
        # TODO: Generate and return a random configuration
        #

        lower_limits, upper_limits = self.robot.GetActiveDOFLimits()

        # Save robot current configuration
        init_config = self.robot.GetActiveDOFValues()
        res = init_config
        samples = []

        # try at most 100 times to find one random collision free config
        # Samples is a DOF by 100 matrix. Each row is one joint value with 100 samples.
        for i in xrange(len(activeDOFIndices)):
            samples.append(numpy.random.uniform(lower_limits[i], upper_limits[i], 100))

        for n in xrange(100):
            # get the nth column in the samples matrix
            config = [sample[n] for sample in samples]

            with self.env:
                self.robot.SetDOFValues(config, activeDOFIndices)

            if not self.env.CheckCollision(self.robot):
                res = config
                break
        # Restore robot transform
        with self.env:
            self.robot.SetDOFValues(init_config, activeDOFIndices)

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
        activeDOFIndices = self.robot.GetActiveDOFIndices()

        config_increment = (end_config - start_config)/num_points

        check_config = start_config

        lower_limits, upper_limits = self.robot.GetActiveDOFLimits()

        # check all the checking points
        for i in xrange(num_points):
            # check_config is the interpolation point of the start_config and end_config
            # based on the current check point
            check_config = start_config + config_increment * (i + 1)

            # if check_config is out of DOF limits, then return the last successful check_config
            for n in xrange(len(check_config)):
                if check_config[n] < lower_limits[n] or check_config[n] > upper_limits[n]:
                    with self.env:
                        self.robot.SetDOFValues(start_config, activeDOFIndices)
                    return check_config - config_increment

            # always lock the environment first if you want to change the robot configuration
            with self.env:
                # set the robot to the new transfomation check_config to see whether it collides with
                # all the obstacles
                self.robot.SetDOFValues(check_config, activeDOFIndices)

            if self.env.CheckCollision(self.robot):
                with self.env:
                    self.robot.SetDOFValues(start_config, activeDOFIndices)
                return check_config - config_increment

        with self.env:
            self.robot.SetDOFValues(start_config, activeDOFIndices)

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

        num_check_points = 500
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

