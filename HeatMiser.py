# @Martin Green, @Nyla Worker
# HeatMiser.py
# AI HW 2
# Winter 2018

import random
import math




class Node:
    def __init__(self, office):
        self.office = office
        self.edges = []

    def __eq__(self, other):
        return self.office == other.office

        # Used for finding the collision chain for this node.

    def __hash__(self):
        return self.office


class Graph:
    def __init__(self, nodes=[]):
        self.nodes = nodes

    def addNode(self, office):
        newNode = Node(office)
        self.nodes.append(newNode)

    def addEdge(self, node1, node2, weight):
        node1.edges.append([node2, weight])
        node2.edges.append([node1, weight])



    def bfs(self):
        if not self.nodes:
            return []
        start = self.nodes[0]
        visited= set([start])
        queue = deque([start])
        result = []

        while queue:
            node = queue.popleft()
            result.append(node)
            for nd in node.edges:
                if nd not in visited:
                    queue.append(nd)
                    visited.add(nd)
        return result

    def dfs(self):
        if not self.nodes:
            return []
        start = self.nodes[0]
        visited = set([start])
        stack = [start]
        result = []
        
        while stack:
            node = stack.pop()
            result.append(node)
            for nd in node.edges:
                if nd not in visited:
                    stack.append(nd)
                    visited.add(nd)
        return result


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




class Robot(object):

    def __init__(self, curRoom = 1):
        self.floor = Floor()
        self.floor.printRooms()

        self.curRoom = curRoom
        self.GoalTemp  = 72; self.TempDev  = 1.5
        self.GoalHumid = 47; self.HumidDev = 1.75

    # def curRoom(self):
    #     return self.curRoom

    def nextRoom(self):
        self.curRoom = (self.curRoom % 12) + 1

    def isTempGood(self):
        avg = self.floor.getAvgTemperature()
        dev = math.floor(self.floor.getStdDevTemperature()*10)/10    # Round to tenths place
        if ((self.GoalTemp <= avg < self.GoalTemp + 1) and dev <= self.TempDev):
            return True
        else:
            return False

    def isHumidGood(self):
        avg = self.floor.getAvgHumidity()
        dev = math.floor(self.floor.getStdDevHumidity()*100)/100     # Round to hundredths place
        if ((self.GoalHumid <= avg < self.GoalHumid + 1) and dev <= self.HumidDev):
            return True
        else:
            return False

    def changeTemp(self, room):
        if (self.floor.getStdDevTemperature() > self.TempDev):
            if (self.floor.getTemperature(room) <= self.GoalTemp):
                self.floor.increaseTemp(room)
                print('Increasing temperature.')
            else:
                self.floor.decreaseTemp(room)
                print('Decreasing temperature.')
        else:
            if (self.floor.getAvgTemperature() <= self.GoalTemp):
                self.floor.increaseTemp(room)
                print('Increasing temperature.')
            else:
                self.floor.decreaseTemp(room)
                print('Decreasing temperature.')


    def changeHumid(self, room):
        if (self.floor.getStdDevHumidity() > self.HumidDev):
            if (self.floor.getHumidity(room) < self.GoalHumid):
                self.floor.increaseHumid(room)
                print('Increasing humidity.')
            else:
                self.floor.decreaseHumid(room)
                print('Decreasing humidity.')
        else:
            if (self.floor.getAvgHumidity() < self.GoalHumid):
                self.floor.increaseHumid(room)
                print('Increasing humidity.')
            else:
                self.floor.decreaseHumid(room)
                print('Decreasing humidity.')


def calculateStdDev(array):
    num_items = len(array)
    mean = sum(array)/num_items
    differences = [x - mean for x in array]
    sq_differences = [d ** 2 for d in differences]
    ssd = sum(sq_differences)

    variance = ssd / num_items
    stdDev = math.sqrt(variance)

    return stdDev

def runSimulation():

    robot = Robot()

    temp  = 0; stdTemp  = 0;
    humid = 0; stdHumid = 0;
    visits = 0

    while ((not robot.isTempGood()) or (not robot.isHumidGood())):
        visits += 1
        room = robot.curRoom
        print('Office {}: {} degrees, {}% humidity'
                .format(room, robot.floor.getTemperature(room), robot.floor.getHumidity(room)))

        
        # Figure out which factor to change
        tempDelta = abs(robot.GoalTemp - robot.floor.getTemperature(room))
        humidDelta = abs(robot.GoalHumid - robot.floor.getHumidity(room))

        if (robot.isTempGood()):
            robot.changeHumid(room)
        elif (robot.isHumidGood()):
            robot.changeTemp(room)
        elif (humidDelta > tempDelta):
            robot.changeHumid(room)
        else:
            robot.changeTemp(room)


        temp = robot.floor.getAvgTemperature()
        humid = robot.floor.getAvgHumidity()
        stdTemp = robot.floor.getStdDevTemperature()
        stdHumid = robot.floor.getStdDevHumidity()
        
        print('The average is {:.2f} degrees ({:.2f} deviation)'
                ' and {:.2f}% humidity ({:.2f} deviation).\n'
                .format(temp, stdTemp, humid, stdHumid))

        robot.nextRoom()


    # Simulation over, print results
    print('Finished simulation:')
    robot.floor.printRooms()

    print('The average is {:.2f} degrees ({:.2f} deviation)'
            ' and {:.2f}% humidity ({:.2f} deviation).'
            .format(temp, stdTemp, humid, stdHumid))

    return visits


def main():
    numVisits = []
    for i in range(100):
        print('Simulation {}:'.format(i+1))
        visits = runSimulation()
        numVisits.append(visits)
        print('It took {} total visits.'.format(visits))
        print()

    stdDevVisits = calculateStdDev(numVisits)
    meanVisits = sum(numVisits)/len(numVisits)
    print('Overall, it took an average of {} visits ({:.2f} deviation).\n'
            .format(meanVisits, stdDevVisits))

main()
