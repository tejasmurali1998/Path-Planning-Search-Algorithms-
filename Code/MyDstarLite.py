#D* Lite Algorithm

import numpy as np
import heapq
from queue import PriorityQueue

class DStarLite:
    def __init__(self,grid, start, end):
        
        self.rhs = np.full((len(grid), len(grid[0])),np.inf)
        self.g = np.full((len(grid), len(grid[0])),np.inf)
        self.start = start 
        self.end = end 
        self.grid = grid    
        self.sensedGrid = np.zeros((len(grid), len(grid[0])))
        self.k = 0
        self.openQueue = PriorityQueue(10000)
        
        self.rhs[self.end[0],self.end[1]]=0
        #Start from the target node-- 
        end = Node(self.end,self.CalculateKey(self.end))
        self.openQueue.put(end)
                   
    def heuristic(self, n1, n2):
        return np.linalg.norm(n1 - n2)
    
    def scanGrid(self, range_s):
        RealList = []
        Row = len(self.grid)
        Column = len(self.grid[0])
        for i in range(-range_s, range_s+1):
            for j in range(-range_s, range_s+1):
                if self.start[0] + i >= 0 and self.start[0] + i < Row and self.start[1] + j >= 0 and self.start[1] + j < Column:
                    if not (i == 0 and j == 0):
                        RealList.append(np.array([self.start[0]+i,self.start[1]+j]))
        return RealList

    def getSuccessor(self, u):
        SuccList = [np.array([u[0]-1,u[1]-1]), np.array([u[0]-1,u[1]]), np.array([u[0]-1,u[1]+1]), np.array([u[0],u[1]-1]),np.array([u[0],u[1]+1]), np.array([u[0]+1,u[1]-1]), np.array([u[0]+1,u[1]]), np.array([u[0]+1,u[1]+1])];
        Row = len(self.grid)
        Column = len(self.grid[0])
        RealList = []
        for s in SuccList:
            if s[0] >= 0 and s[0] < Row and s[1] >= 0 and s[1] < Column:
                RealList.append(s)
        return RealList
    
    def cost(self, u1, u2):
        if self.sensedGrid[u1[0],u1[1]] == 1 or self.sensedGrid[u2[0],u2[1]] == 1:
            return np.inf
        else:
            return round(self.heuristic(u1, u2),1)
 
    def CalculateKey(self, n):
        key = [0,0]
        key[0] = min(self.g[n[0], n[1]],self.rhs[n[0], n[1]]) + self.heuristic(self.start,n) + self.k 
        key[1] = min(self.g[n[0], n[1]],self.rhs[n[0], n[1]])
        return key
                                 
    def isConsistent(self,position):
        return self.g[position[0], position[1]] != self.rhs[position[0], position[1]]

    def SenseIfAnyObstacles(self,DS, current):
        SuccList = DS.scanGrid(5)
        misMatch = False

        for s in SuccList:
            if DS.sensedGrid[s[0],s[1]] != DS.grid[s[0],s[1]]:
                misMatch = True
                break
        if misMatch == True:
            DS.k += DS.heuristic(current, DS.start)
            current = DS.start.copy()
            for s in SuccList:
                if DS.sensedGrid[s[0],s[1]] != DS.grid[s[0],s[1]]:
                    #updated_points.append(s[0])
                    #updated_points.append(s[1])
                    DS.sensedGrid[s[0],s[1]] = DS.grid[s[0],s[1]]
                    #print("In scan calling update", s)
                    DS.UpdateVertex(s)
            print("Planning the path..")
            DS.ComputeShortestPath()
        return current #, np.asarray(updated_points)

    def UpdateVertex(self, u):
        SuccList = []
        if np.sum(np.abs(u - self.end)) != 0:
            SuccList = self.getSuccessor(u)
            MinSucc = np.inf
            for s in SuccList:
                         if self.cost(u, s) + self.g[s[0],s[1]] < MinSucc:
                            MinSucc = self.cost(u, s) + self.g[s[0],s[1]]
            self.rhs[u[0],u[1]] = MinSucc
            if Node(u, [0, 0]) in self.openQueue.queue:
                self.openQueue.queue.remove(Node(u, [0, 0]))
            if self.g[u[0],u[1]] != self.rhs[u[0],u[1]]:
                self.openQueue.put(Node(u, self.CalculateKey(u)))
    
    def lowestCostNode(self):
        return heapq.nsmallest(1, self.openQueue.queue)[0]
                         
    def ComputeShortestPath(self):
     while len(self.openQueue.queue) > 0 and (self.lowestCostNode() < Node(self.start, self.CalculateKey(self.start)) or self.rhs[self.start[0], self.start[1]] != self.g[self.start[0], self.start[1]]):
        st = Node(self.start, self.CalculateKey(self.start))
        u = self.openQueue.get().position 
        if self.g[u[0],u[1]] > self.rhs[u[0],u[1]]:
             self.g[u[0],u[1]] = self.rhs[u[0],u[1]]
             SuccList = self.getSuccessor(u)
             for s in SuccList:
                  self.UpdateVertex(s)
        else:
             self.g[u[0],u[1]] = np.inf
             SuccList = self.getSuccessor(u)
             SuccList.append(u)
             for s in SuccList:
                  self.UpdateVertex(s)

class Node:
    def __init__(self, pos, key):
        self.key = key
        self.position = pos
    
    def __eq__(self, other):
        return (np.array(self.position) == np.array(other.position)).all()
    
    def __ne__(self, other):
        return self.key != other.key

    def __lt__(self, other):
        return (np.array(self.key) < np.array(other.key)).all()

    def __le__(self, other):
        return (np.array(self.key) <= np.array(other.key)).all()
    def __gt__(self, other):
        return (np.array(self.key) > np.array(other.key)).all()

    def __ge__(self, other):
        return np.array(self.key) >= np.array(other.key).all()
 
def CreateGrid(width, height):
    
    Shape = (width,height)
    y = np.zeros(Shape,dtype=int)

    for i in range(0,len(y)-1):
         if (i != 3 and i != 4 and i !=5):         
             y[i,5]=1
    #border       
    y[0,:] = y[-1,:] = 1
    y[:,0] = y[:,-1] = 1
    y[3,4] = 1
    map = y
    return map;
    
def AddObstacles(locations,grid):
    for pos in locations:
        grid[pos[0],pos[1]] = 1
    

def getNextNodetoMove(DS):
  SuccList = DS.getSuccessor(DS.start)      
  MinSucc = np.inf
  for s in SuccList:
      if DS.cost(DS.start, s) + DS.g[s[0],s[1]] < MinSucc:
          MinSucc = DS.cost(DS.start, s) + DS.g[s[0],s[1]]
          temp = s
  return temp

def inputStartAndEnd():
    print("Please enter start and end.. Example: 1 2,9 9")
    val = input()
    valarr = val.split(',')
    if (len(valarr) != 2):
        print("Failed to read start and end... exiting")
        return
    positions=[]
    for numarr in valarr:
            pos = [int(num) for num in numarr.split(' ')]
            positions.append(pos)
            
    start = np.array(positions[0])
    end = np.array(positions[1])
    return start, end              

def main():
    
    map = CreateGrid(10,10)
    start, end = inputStartAndEnd()
    print(start,end)
    #DS = DStarLite(map,np.array([2,3]),np.array([7,8]))
    DS = DStarLite(map,start,end)
    Last = DS.start
    print("Hey I am a robot, at this location:",DS.start)
    print("\n\n")
    #print("Start:", DS.start, "End:", DS.end)
    print("I need directions to go to this location:", DS.end)
    print("\n\n")
    
    print("-----------------------------------------------------------")
    Last = DS.SenseIfAnyObstacles(DS,Last)
    path = [DS.start.tolist()]
    DS.ComputeShortestPath()
    counter = 0
    print("\n\n")
    print("Ok, Found a path, robot started moving:")
    print("\n\n")
    
    while (np.array(DS.start) !=  np.array(DS.end)).any():
      counter += 1
      temp = getNextNodetoMove(DS)            
      DS.start = temp.copy()
      path.append(DS.start.tolist())
      
      print("Path moved so far...:",path)
      print("\n\n")
      print("-----------------------------------------------------------")
      print("Are there any obstacle in my path? : (a b,c d etc), or c for continue:")     
      obstacles = []
      val = input()
      if (val != 'c'):
        print("\n")
        print("ok, i am finding a new path..")
        valarr = val.split(',')
        for numarr in valarr:
            valarr = [int(num) for num in numarr.split(' ')]
            obstacles.append(valarr)      
        #print(obstacles)
 
        AddObstacles(obstacles,DS.grid)
          
        DS.SenseIfAnyObstacles(DS, Last)
        # print("---------------------RHS values--------------------------------------")
        # print(DS.rhs)
        # print("-----------------------G values------------------------------------")
        # print(DS.g)

      
   
    print("---------------------RHS values--------------------------------------")
    print(DS.rhs)
    print("-----------------------G values------------------------------------")
    print(DS.g)
    print("Wheee.. i reached my target..") 
    print("--------------------------- PATH -------------------------------")
    print(path)    
       #return path, sensed_map, updated_points
       
## HERE we invoke the main() which will statup the whole thing.##
## Along the way the grid can be dynamically changed to flag obstacles, 
## Just enter the coordinate of the obstacle like for example: 4 5,5 6 etc
## which will result in replanning the path.        
main() 