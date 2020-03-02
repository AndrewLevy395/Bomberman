# This is necessary to find the main code
import sys
import random 
sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from colorama import Fore, Back



class TestCharacter(CharacterEntity):

    closedList = []
    openList = []
    pathList = []
    surrounding = []
    previous = []
    distance = 0
    exitX = -1
    exitY = -1
    exitFound = False
    pathFound = False

    def do(self, wrld):

        for i in range(wrld.width()):
            for j in range(wrld.height()):
                if wrld.exit_at(i,j):
                    self.exitX = i
                    self.exitY = j

        if self.closedList == []:
            self.previous = (self.x,self.y)
            self.closedList.append(((self.x, self.y), "start"))

        while self.exitFound == False:
            self.findExit(wrld)

        if self.pathFound == False:
            self.findPath(wrld)

        self.moveMan(wrld)

    def findExit(self, wrld):

        self.surrounding = []

        hx = self.exitX - self.previous[0]
        hy = self.exitY - self.previous[1]
        ht = abs(hx) + abs(hy) + self.distance

        for x in range(-1, 2):
            for y in range(-1, 2):
                if(wrld.height() > self.previous[1] + y and wrld.width() > self.previous[0] + x):
                    if(wrld.wall_at(self.previous[0] + x, self.previous[1]) == False and wrld.wall_at(self.previous[0], self.previous[1] + y) == False and wrld.wall_at(self.previous[0] + x, self.previous[1] + y) == False):
                        ignore = False
                        for i in self.closedList:
                            if i == (self.previous[0] + x, self.previous[1] + y):
                                ignore = True
                        if ignore == False:
                            self.surrounding.append((self.previous[0] + x, self.previous[1] + y))

        closest = ((-1,-1), 10000)

        for node in self.surrounding:
            exists = False
            hx = self.exitX - node[0]
            hy = self.exitY - node[1]
            ht = abs(hx) + abs(hy)
            if self.openList == []:
                self.openList.append((node, ht, (self.previous[0], self.previous[1])))
            else:
                for i in self.openList:
                    if i[0] == node:
                        exists = True
                for j in self.closedList:
                    if j[0] == node:
                        exists = True
                if exists == False: 
                    self.openList.append((node, ht, (self.previous[0], self.previous[1])))

        for node in self.openList:
            for n in range(len(self.surrounding)):
                if node[1] < closest[1]:
                    closest = (node[0], node[1], node[2])

        for i in range(len(self.openList)):
            if self.openList[i][0] == closest[0]:
                remover = closest
        
        self.openList.remove(remover)
        
        newx = closest[0][0]
        newy = closest[0][1]

        self.closedList.append(((newx,newy), closest[2]))

        self.previous = closest[0]

        self.distance += 1

        if wrld.exit_at(newx, newy):
            self.exitFound = True

    def findPath(self, wrld):
        child = (self.exitX, self.exitY)
        for i in self.closedList[::-1]:
            if i[0] == child:
                self.pathList.append(i)
                child = i[1]
        print(self.pathList)
        self.pathFound = True

    def moveMan(self, wrld):
        for i in self.pathList[::-1]:
            if i[1] == (self.x, self.y):
                movement = i[0]
        dx = int(movement[0]) - self.x
        dy = int(movement[1]) - self.y
        self.move(dx, dy)
