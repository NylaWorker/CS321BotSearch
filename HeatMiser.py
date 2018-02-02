# @Anonymous
# HeatMiser.py
# AI HW 2
# Winter 2018

import random
import math
from collections import deque
import re
import csv
import heapq

heuristic = []  # Global heuristic list for each run of simulation

class PriorityQueue:
    def __init__(self):
        self.elements = []
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self):
        return heapq.heappop(self.elements)[1]

class Node:
    def __init__(self, office):
        self.office = office
        self.edges = []
        self.weights = []

    def __eq__(self, other):
        return self.office == other.office

    def __lt__(self, other):
        return self.office < other.office

    def __hash__(self):
        return self.office

    def __repr__(self):
        return str(self.office)

class Graph:
    def __init__(self, nodes=[]):
        self.nodes = nodes

    def addNode(self, office):
        newNode = Node(office)
        self.nodes.append(newNode)

    def addEdge(self, node1, node2, weight):
        node1.edges.append(node2)
        node1.weights.append(weight)

        node2.edges.append(node1)
        node2.weights.append(weight)


    def bfs(self, startRoom, goalRoom):
        if not self.nodes[startRoom]:
            return [0, 0]   # Error

        start = self.nodes[startRoom]
        visited = set()
        queue = deque([start])
        result = []
        path = []
        pathWeights = []

        for i in range(13):
            path.append(0)
            pathWeights.append(0)
        path[startRoom] = -1


        while queue:
            node = queue.popleft()
            if node not in visited:
                result.append(node)
                visited.add(node)

                for i in range(len(node.edges)):
                    curNode = node.edges[i]
                    if curNode not in visited:
                        queue.append(curNode)
                        if path[curNode.office] == 0:
                            path[curNode.office] = node.office
                            pathWeights[curNode.office] = node.weights[i]

        # Calculate jumps and path cost
        numJumps = 0
        pathWeight = 0
        pathFound = False
        curRoom = goalRoom

        while not pathFound:
            if curRoom == startRoom:
                pathFound = True
            else:
                pathWeight += pathWeights[curRoom]
                numJumps += 1
                curRoom = path[curRoom]

        return [numJumps, pathWeight]

    def aStarSearch(self, startRoom, goalRoom):
        if not self.nodes[startRoom]:
            return [0, 0]   # Error

        frontier = PriorityQueue()
        start = self.nodes[startRoom]
        frontier.put(start, 0)

        cameFrom = {}
        costSoFar = {}

        cameFrom[start.office] = 0
        costSoFar[start.office] = 0
        
        while not frontier.empty():
            current = frontier.get()
            
            if current.office == goalRoom:
                break

            for i in range(len(current.edges)):
                next = current.edges[i]
                newCost = costSoFar[current.office] + current.weights[i]

                if next.office not in costSoFar or newCost < costSoFar[next.office]:
                    costSoFar[next.office] = newCost
                    priority = newCost + findHeuristicWeight(next.office, goalRoom)
                    frontier.put(next, priority)
                    cameFrom[next.office] = current.office

        # Calculate jumps and path cost
        numJumps = 0
        pathFound = False
        curRoom = goalRoom        

        while not pathFound:
            if curRoom == startRoom:
                pathFound = True
            else:
                numJumps += 1
                curRoom = cameFrom[curRoom]

        return [numJumps, costSoFar[goalRoom]]


class Floor(object):

    def __init__(self):
        self.Rooms = 12
        self.RoomTemps = []
        self.RoomHumid = []
        self.minTemp  = 65; self.maxTemp  = 75
        self.minHumid = 45; self.maxHumid = 55
        self.graph = Graph()
        self.initRooms()
        self.initGraph()

    def initRooms(self):
        for i in range(self.Rooms):
            self.RoomTemps.append(random.randint(self.minTemp, self.maxTemp))
            self.RoomHumid.append(random.randint(self.minHumid, self.maxHumid))

    def initGraph(self):
        for i in range(self.Rooms+1):
            self.graph.addNode(i)

        self.graph.addEdge(self.graph.nodes[1],  self.graph.nodes[2],  13)
        self.graph.addEdge(self.graph.nodes[1],  self.graph.nodes[3],  15)
        self.graph.addEdge(self.graph.nodes[2],  self.graph.nodes[4],  7)
        self.graph.addEdge(self.graph.nodes[3],  self.graph.nodes[7],  23)
        self.graph.addEdge(self.graph.nodes[4],  self.graph.nodes[5],  6)
        self.graph.addEdge(self.graph.nodes[4],  self.graph.nodes[6],  10)
        self.graph.addEdge(self.graph.nodes[4],  self.graph.nodes[9],  16)
        self.graph.addEdge(self.graph.nodes[5],  self.graph.nodes[8],  4)
        self.graph.addEdge(self.graph.nodes[6],  self.graph.nodes[7],  9)
        self.graph.addEdge(self.graph.nodes[7],  self.graph.nodes[10], 17)
        self.graph.addEdge(self.graph.nodes[8],  self.graph.nodes[9],  5)
        self.graph.addEdge(self.graph.nodes[9],  self.graph.nodes[10], 8)
        self.graph.addEdge(self.graph.nodes[10], self.graph.nodes[11], 2)
        self.graph.addEdge(self.graph.nodes[11], self.graph.nodes[12], 19)

    def printRooms(self):
        for i in range(self.Rooms):
            print('Office {}: {} degrees, {}% humidity'
                    .format(i+1, self.RoomTemps[i], self.RoomHumid[i]))
        print()

    def getAvgTemperature(self):
        return sum(self.RoomTemps) / len(self.RoomTemps)

    def getAvgHumidity(self):
        return sum(self.RoomHumid) / len(self.RoomHumid)

    def getTemperature(self, room):
        return self.RoomTemps[room-1]

    def getHumidity(self, room):
        return self.RoomHumid[room-1]

    def getStdDevTemperature(self):
        return calculateStdDev(self.RoomTemps)

    def getStdDevHumidity(self):
        return calculateStdDev(self.RoomHumid)

    def increaseTemp(self, room):
        if (self.RoomTemps[room-1] < self.maxTemp):
            self.RoomTemps[room-1] = self.RoomTemps[room-1] + 1

    def decreaseTemp(self, room):
        if (self.RoomTemps[room-1] > self.minTemp):
            self.RoomTemps[room-1] = self.RoomTemps[room-1] - 1

    def increaseHumid(self, room):
        if (self.RoomHumid[room-1] < self.maxHumid):
            self.RoomHumid[room-1] = self.RoomHumid[room-1] + 1

    def decreaseHumid(self, room):
        if (self.RoomHumid[room-1] > self.minHumid):
            self.RoomHumid[room-1] = self.RoomHumid[room-1] - 1

    def setTemp(self, room, val):
        self.RoomTemps[room - 1] = val

    def setHumid(self, room,val):
        self.RoomHumid[room - 1] = val


class Robot(object):

    def __init__(self):
        self.floor = Floor()
        self.floor.printRooms()

        self.curRoom = random.randint(1, self.floor.Rooms)
        self.GoalTemp  = 72; self.TempDev  = 1.5
        self.GoalHumid = 47; self.HumidDev = 1.75

    def nextRoom(self):
        self.curRoom = (self.curRoom % 12) + 1

    def setRoom(self, room):
        self.curRoom = room

    def isTempGood(self):
        avg = self.floor.getAvgTemperature()
        # Round to tenths place
        dev = math.floor(self.floor.getStdDevTemperature()*10)/10
        if ((self.GoalTemp <= avg < self.GoalTemp + 1) and dev <= self.TempDev):
            return True
        else:
            return False

    def isHumidGood(self):
        avg = self.floor.getAvgHumidity()
        # Round to hundredths place
        dev = math.floor(self.floor.getStdDevHumidity()*100)/100
        if ((self.GoalHumid <= avg < self.GoalHumid + 1) and dev <= self.HumidDev):
            return True
        else:
            return False

    def changeTemp(self, room, tem):
        avg = self.floor.getAvgTemperature()
        if(self.floor.getStdDevTemperature()<1.5 - 4/12):
            if(avg > 72):
                self.floor.setTemp(room, 71)
            elif(avg < 72):
                self.floor.setTemp(room, 73)
        elif (self.floor.getStdDevTemperature()<1.5 - 9/12):
            if (avg > 72):
                self.floor.setTemp(room, 70)
            elif (avg < 72):
                self.floor.setTemp(room, 74)
        else:
            self.floor.setTemp(room, 72)

    def changeHumid(self, room, hum):
        avg = self.floor.getAvgHumidity()
        self.floor.setHumid(room, 47)
        if (self.floor.getStdDevHumidity() < 1.7 - 4 / 12):
            if (avg > 47):
                self.floor.setHumid(room, 46)
            elif (avg < 47):
                self.floor.setHumid(room, 48)
        elif (self.floor.getStdDevHumidity() < 1.7 - 9/12):
            if (avg > 47):
                self.floor.setHumid(room, 45)
            elif (avg < 47):
                self.floor.setHumid(room, 49)
        else:
            self.floor.setHumid(room, 47)


def calculateStdDev(array):
    numItems = len(array)
    mean = sum(array)/numItems
    differences = [x - mean for x in array]
    sqDifferences = [d ** 2 for d in differences]
    ssd = sum(sqDifferences)

    variance = ssd / numItems
    stdDev = math.sqrt(variance)

    return stdDev

def findHeuristicWeight(curRoom, goalRoom):
    if goalRoom == curRoom:
        return 0
    else:
        return heuristic[(curRoom-1)*11 + goalRoom - (2 if goalRoom > curRoom else 1)][2]


def runSimulation(type):
    robot = Robot()

    temp  = 0; stdTemp  = 0
    humid = 0; stdHumid = 0
    visits = 0
    totalJumps = 0
    totalWeights = 0

    while ((not robot.isTempGood()) or (not robot.isHumidGood())):

        # Search for room to change:
        maxDelta = 0
        maxDeltaRoom = 0
        if (not robot.isTempGood()):
            for i in range(len(robot.floor.RoomTemps)):
                if abs(robot.GoalTemp+0.5 - robot.floor.RoomTemps[i]) > maxDelta:
                    maxDelta = abs(robot.GoalTemp - robot.floor.RoomTemps[i])
                    maxDeltaRoom = i+1
        if (not robot.isHumidGood()):
            for i in range(len(robot.floor.RoomHumid)):
                if abs(robot.GoalHumid+0.5 - robot.floor.RoomHumid[i]) > maxDelta:
                    maxDelta = abs(robot.GoalHumid - robot.floor.RoomHumid[i])
                    maxDeltaRoom = i+1


        if (type):  # A*
            [numJumps, pathWeight] = robot.floor.graph.aStarSearch(robot.curRoom, maxDeltaRoom)
        else:       # BFS
            [numJumps, pathWeight] = robot.floor.graph.bfs(robot.curRoom, maxDeltaRoom)

        totalJumps += numJumps
        totalWeights += pathWeight

        print('Starting in room {}, going to room {} in {} jumps, using {} power'
                .format(robot.curRoom, maxDeltaRoom, numJumps, pathWeight))

        robot.setRoom(maxDeltaRoom)
        room = maxDeltaRoom
        visits += 1

        
        # Figure out which factor to change
        tempDelta = abs(robot.GoalTemp - robot.floor.getTemperature(room))
        humidDelta = abs(robot.GoalHumid - robot.floor.getHumidity(room))

        print('Office {}: {} degrees, {}% humidity'
                .format(room, robot.floor.getTemperature(room), 
                              robot.floor.getHumidity(room)))

        if (robot.isTempGood()):
            # print('Changing humidity.')
            robot.changeHumid(room,robot.floor.RoomTemps[room-1])
        elif (robot.isHumidGood()):
            # print('Changing temperature.')
            robot.changeTemp(room,robot.floor.RoomHumid[room-1])
        elif (humidDelta > tempDelta):
            # print('Changing humidity.')
            robot.changeHumid(room,robot.floor.RoomHumid[room-1])
        else:
            # print('Changing temperature.')
            robot.changeTemp(room,robot.floor.RoomTemps[room-1])

        print('Office {}: {} degrees, {}% humidity'
                .format(room, robot.floor.getTemperature(room), 
                              robot.floor.getHumidity(room)))

        temp = robot.floor.getAvgTemperature()
        humid = robot.floor.getAvgHumidity()
        stdTemp = robot.floor.getStdDevTemperature()
        stdHumid = robot.floor.getStdDevHumidity()
        
        print('The average is {:.2f} degrees ({:.2f} deviation)'
                ' and {:.2f}% humidity ({:.2f} deviation).\n'
                .format(temp, stdTemp, humid, stdHumid))


    # Simulation over, print results
    print('Finished simulation:')
    robot.floor.printRooms()

    print('The average is {:.2f} degrees ({:.2f} deviation)'
            ' and {:.2f}% humidity ({:.2f} deviation).'
            .format(temp, stdTemp, humid, stdHumid))

    return [visits, totalJumps, totalWeights]


def mainSimulation(runs, type):
    numVisits = []
    numJumps = []
    numWeights = []
    for i in range(runs):
        print('Simulation {}:'.format(i+1))
        [visits, totalJumps, totalWeights] = runSimulation(type)
        numVisits.append(visits)
        numJumps.append(totalJumps)
        numWeights.append(totalWeights)

        print('It took {} total office visits and used {} total power.'.format(totalJumps, totalWeights))
        print()

    stdDevVisits = calculateStdDev(numJumps)
    meanVisits = sum(numJumps)/len(numJumps)

    stdDevPower = calculateStdDev(numWeights)
    meanPower = sum(numWeights)/len(numWeights)

    print('Overall, it took an average of {} office visits ({:.2f} deviation)\n'
            'and an average of {} power ({:.2f} deviation).\n'
            .format(meanVisits, stdDevVisits, meanPower, stdDevPower))


def main():

    # Load heuristic
    with open('HeatMiserHeuristic.txt', 'r') as tsv:
        heuristicFile = [line.strip().split('\t') for line in tsv]

        for i in range(1, len(heuristicFile)):
            heuristic.append(list(map(int,list(filter(None, heuristicFile[i])))))


    numberRuns = 100
    type = 0    # 0 = BFS, 1 = A*

    # BFS
    print('\nRunning part A using Bredth First Search\n')
    mainSimulation(numberRuns, type)

    # # A*
    print('\nRunning part B using A*\n')
    type = 1
    mainSimulation(numberRuns, type)


main()
