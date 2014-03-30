import numpy
from DiscreteEnvironment import DiscreteEnvironment

class HerbEnvironment(object):
    
    def __init__(self, herb, resolution):
        
        self.robot = herb.robot
        self.lower_limits, self.upper_limits = self.robot.GetActiveDOFLimits()
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

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
    
    def GetSuccessors(self, node_id):

        successors = []

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring
        #  nodes
        
        return successors

    def IsInBoundary(self, coord):
        config = self.discrete_env.GridCoordToConfiguration(coord)
	for i in xrange(len(self.lower_limits)):
		if config[i] < self.lower_limits[i] or config[i] > self.upper_limits[i]:
			return False
        return True


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
        goal_config  = self.discrete_env.NodeIdToConfiguration(goal_id)
        D = 1
	for i in xrange(len(start_config)):
        	delta = abs(start_config[i]-goal_config[i])
        	cost = cost + D*delta
        return cost

