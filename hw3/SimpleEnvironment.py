import numpy
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment

class SimpleEnvironment(object):
    
    def __init__(self, herb, resolution):
        self.robot = herb.robot
        self.lower_limits = [-5., -5.]
        self.upper_limits = [5., 5.]
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        # add an obstacle
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 1.5], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

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
	T = numpy.array([[1, 0, 0, config[0]],
			 [0, 1, 0, config[1]],
			 [0, 0, 1, 0],
			 [0, 0, 0, 1]])
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
	D = 1
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
        
    def PlotEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k.-', linewidth=2.5)
        pl.draw()

        
