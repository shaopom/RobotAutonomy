import numpy

class DiscreteEnvironment(object):

    def __init__(self, resolution, lower_limits, upper_limits):

        # Store the resolution
        self.resolution = resolution

        # Store the bounds
        self.lower_limits = lower_limits
        self.upper_limits = upper_limits

        # Calculate the dimension
        self.dimension = len(self.lower_limits)

        # Figure out the number of grid cells that are in each dimension
        self.num_cells = self.dimension*[0]
        for i in range(self.dimension):
            self.num_cells[i] = numpy.ceil((upper_limits[i] - lower_limits[i])/resolution)


    def ConfigurationToNodeId(self, config):
        """This function maps a node configuration in full configuration space to a node in discrete space"""
        coord = self.ConfigurationToGridCoord(config)
        node_id = self.GridCoordToNodeId(coord)
        return node_id

    def NodeIdToConfiguration(self, nid):
        """This function maps a node in discrete space to a configuration in the full configuration space"""
        coord = self.NodeIdToGridCoord(nid)
        config = self.GridCoordToConfiguration(coord)
        return config
        
    def ConfigurationToGridCoord(self, config):
        """This function maps a configuration in the full configuration space to a grid coordinate in discrete space"""
        coord = [0] * self.dimension
    	for i in xrange(self.dimension):
    		coord[i] = numpy.floor((config[i]-self.lower_limits[i])/self.resolution) 
        return coord
    
    def GridCoordToConfiguration(self, coord):
        """This function maps a grid coordinate in discrete space to a configuration in the full configuration space"""
        config = [0] * self.dimension
        for i in xrange(self.dimension):
		    config[i] = self.lower_limits[i] + self.resolution*coord[i] + self.resolution/2
        return config

    def GridCoordToNodeId(self,coord):
        """This function maps a grid coordinate to the associated node id"""
        node_id = 0
    	for i in xrange(self.dimension-1,-1,-1): # from 'self.dimension-1' to '0'
    		cells = 1
    		for j in xrange(i):
    			cells = cells*self.num_cells[j]
    		node_id = node_id + coord[i]*cells
        return node_id

    def NodeIdToGridCoord(self, node_id):
        """This function maps a node id to the associated grid coordinate"""
        coord = [0] * self.dimension
    	for i in xrange(self.dimension-1,-1,-1):
    		cells = 1
    		for j in xrange(i):
    			cells = cells*self.num_cells[j]
    		coord[i] = numpy.floor(node_id/cells)
    		node_id = node_id - cells*coord[i]
        return coord


