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

    def GetSuccessors(self, node_id):

        successors = []

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring
        #  nodes
	coord = self.discrete_env.NodeIdToGridCoord(node_id)
	coord_successors = self.GetNeighbor(coord)
	for i in xrange(len(coord_successors)):
		successors.append(self.discrete_env.GridCoordToNodeId(coord_successors[i]))
        return successors

    def GetNeighbor(self, coord):
	x_limits = self.discrete_env.num_cells[0]
        y_limits = self.discrete_env.num_cells[1]
        coord_successors = []
        # if the coord is at the four corner, we only return two neighbor       
        if coord[0] == 0 and coord[1] == 0:
                coord_successors.append([coord[0]+1, coord[1]])
                coord_successors.append([coord[0], coord[1]+1])
        elif coord[0] == x_limits-1 and coord[1] == 0:
                coord_successors.append([coord[0]-1, coord[1]])
                coord_successors.append([coord[0], coord[1]+1])
        elif coord[0] == 0 and coord[1] == y_limits-1:
                coord_successors.append([coord[0]+1, coord[1]])
                coord_successors.append([coord[0], coord[1]-1])
        elif coord[0] == x_limits-1 and coord[1] == y_limits-1:
                coord_successors.append([coord[0]-1, coord[1]])
                coord_successors.append([coord[0], coord[1]-1])
        # if the coord is at the boundary and not the corner, we only return three neighbor
        elif coord[0] == 0:
                coord_successors.append([coord[0], coord[1]-1])
                coord_successors.append([coord[0]+1, coord[1]])
                coord_successors.append([coord[0], coord[1]+1])
        elif coord[0] == x_limits-1:
                coord_successors.append([coord[0], coord[1]-1])
                coord_successors.append([coord[0]-1, coord[1]])
                coord_successors.append([coord[0], coord[1]+1])
        elif coord[1] == 0:
                coord_successors.append([coord[0]-1, coord[1]])
                coord_successors.append([coord[0], coord[1]+1])
                coord_successors.append([coord[0]+1, coord[1]])
        elif coord[1] == y_limits-1:
                coord_successors.append([coord[0]-1, coord[1]])
                coord_successors.append([coord[0], coord[1]-1])
                coord_successors.append([coord[0]+1, coord[1]])
        # if the coord is inside the boundary, we can return its four neighbor
        else:
		coord_successors.append([coord[0], coord[1]-1])
                coord_successors.append([coord[0]-1, coord[1]])
                coord_successors.append([coord[0], coord[1]+1])
                coord_successors.append([coord[0]+1, coord[1]])

        return coord_successors


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

        
