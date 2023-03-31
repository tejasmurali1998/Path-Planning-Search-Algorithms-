#A* Algorithm

#Create a class called node
class Node():

    def __init__(self, par=None, pos=None):
        self.par = par
        self.pos = pos
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.pos == other.pos

#List of positions returned as a path from the start to the end node in grid
def myastar(grid, start, end):
	
    # Set up both the open and closed lists
    OpenList = []
    ClosedList = []
	
    # Create the start and end node
    StartNode = Node(None, start)
    EndNode = Node(None, end)

    StartNode.g = 0
    StartNode.h = 0
    StartNode.f = 0

    EndNode.g = 0
    EndNode.h = 0 
    EndNode.f = 0

    # Add the start node to the open list 
    OpenList.append(StartNode)

    # while end is not reached
    while len(OpenList) > 0:

        # Get the node with least f value
        CurrentNode = OpenList[0];

        for item in OpenList:
            if (item.f < CurrentNode.f):
                CurrentNode = item;

	# Remove current off open list, add to closed list
        OpenList.remove(CurrentNode)        
        ClosedList.append(CurrentNode)

        # check if goal reached
        if CurrentNode == EndNode:
            path = []
            current = CurrentNode
            while current is not None:
                path.append(current.pos)
                current = current.par
            return path[::-1] # Return reversed pat
        
        # Create children
        children = []
        for NbrPos in getNeighborPositions(CurrentNode): # Positions of the adjacent squares 
            # Get the node position
            # nbr_position is an offset from the current node
            NodePos = (CurrentNode.pos[0] + NbrPos[0], CurrentNode.pos[1] + NbrPos[1])

            # Make sure position is within grid
            if NodePos[0] > (len(grid) - 1) or NodePos[0] < 0 or NodePos[1] > (len(grid[len(grid)-1]) -1) or NodePos[1] < 0:
                continue

            # Check for obstacles
            if grid[NodePos[0]][NodePos[1]] != 0:
                continue

            # Create neighbor
            NewNode = Node(CurrentNode, NodePos)

            # Add it to children list
            children.append(NewNode)

        # Loop through every child
        for child in children:

            # Child already on the closed list
            for ClosedNode in ClosedList:
                if child == ClosedNode:
                    continue

            # Create the f, g, and h values 
            child.g = CurrentNode.g + 1
            child.h = ((child.pos[0] - EndNode.pos[0]) ** 2) + ((child.pos[1] - EndNode.pos[1]) ** 2)
            child.f = child.g + child.h

            
            # If child exists in the open list and the g value is higher than the open list
            # then, do not add the child to open list and dont update openlist node. 
            # If child exists in the openList but its g value is less than the openlist node's g
            # then update the openlost node's g,f & parent  with the child's g,f and parent
                
            ChildExistsInOpenList = False;
            for OpenNode in OpenList:
                if (child == OpenNode):
                    ChildExistsInOpenList = True;
                    if (child.g > OpenNode.g):
                        break        
                    else:
                        OpenNode.par = child.par;
                        OpenNode.g = child.g
                        OpenNode.f = child.f
                        break
            
            # if child is not in the open list, then append to the openlist            
            if (ChildExistsInOpenList == False):
                OpenList.append(child)

def getNeighborPositions(node):
	NbrPos = []
	for pos in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares
		NbrPos.append(pos)
	return NbrPos;

def main():
    
# Maze is an array of arrays
# First argument = list/row, 
# Second argument = element in list/row
# 0 = empty space, 1 = obstacle

    grid = [[0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

    start = (0, 0)
    end = (9, 9)

    path = myastar(grid, start, end)
    print(path)


if __name__ == '__main__':
    main()