# @Martin Green, @Nyla Worker
# HeatMiser.py
# AI HW 2
# Winter 2018

import random
import math




class Node:
    def __init__(self):
        self.val = val
        self.edges = []

    def __eq__(self, other):
        return self.val == other.val

        # Used for finding the collision chain for this node.

    def __hash__(self):
        return self.val


class Graph:
    def __init__(self, nodes=[]):
        self.nodes = nodes

    def add_node(self, val):
        new_node = Node(val)
        self.nodes.append(new_node)

    def add_edge(self, node1, node2):
        node1.edges.append(node2)
        node2.edges.append(node1)

    def bfs(self):
        if not self.nodes:
            return []
        start = self.nodes[0]
        visited, queue, result = set([start]), deque([start]), []
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
        visited, stack, result = set([start]), [start], []
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
        self.initRooms()

    def initRooms(self):
        for i in range(self.Rooms):
            self.RoomTemps.append(random.randint(self.minTemp, self.maxTemp))
            self.RoomHumid.append(random.randint(self.minHumid, self.maxHumid))



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
        self.Floor = Floor()
        self.Floor.printRooms()

        self.CurRoom = curRoom
        self.GoalTemp  = 72; self.TempDev  = 1.5
        self.GoalHumid = 47; self.HumidDev = 1.75

    def curRoom(self):
        return self.CurRoom

    def nextRoom(self):
        self.CurRoom = (self.curRoom() % 12) + 1

    def isTempGood(self):
        avg = self.Floor.getAvgTemperature()
        dev = math.floor(self.Floor.getStdDevTemperature()*10)/10    # Round to tenths place
        if ((self.GoalTemp <= avg < self.GoalTemp + 1) and dev <= self.TempDev):
            return True
        else:
            return False

    def isHumidGood(self):
        avg = self.Floor.getAvgHumidity()
        dev = math.floor(self.Floor.getStdDevHumidity()*100)/100     # Round to hundredths place
        if ((self.GoalHumid <= avg < self.GoalHumid + 1) and dev <= self.HumidDev):
            return True
        else:
            return False

    def changeTemp(self, room):
        if (self.Floor.getStdDevTemperature() > self.TempDev):
            if (self.Floor.getTemperature(room) <= self.GoalTemp):
                self.Floor.increaseTemp(room)
                print('Increasing temperature.')
            else:
                self.Floor.decreaseTemp(room)
                print('Decreasing temperature.')
        else:
            if (self.Floor.getAvgTemperature() <= self.GoalTemp):
                self.Floor.increaseTemp(room)
                print('Increasing temperature.')
            else:
                self.Floor.decreaseTemp(room)
                print('Decreasing temperature.')


    def changeHumid(self, room):
        if (self.Floor.getStdDevHumidity() > self.HumidDev):
            if (self.Floor.getHumidity(room) < self.GoalHumid):
                self.Floor.increaseHumid(room)
                print('Increasing humidity.')
            else:
                self.Floor.decreaseHumid(room)
                print('Decreasing humidity.')
        else:
            if (self.Floor.getAvgHumidity() < self.GoalHumid):
                self.Floor.increaseHumid(room)
                print('Increasing humidity.')
            else:
                self.Floor.decreaseHumid(room)
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
        room = robot.curRoom()
        print('Office {}: {} degrees, {}% humidity'
                .format(room, robot.Floor.getTemperature(room), robot.Floor.getHumidity(room)))

        
        # Figure out which factor to change
        tempDelta = abs(robot.GoalTemp - robot.Floor.getTemperature(room))
        humidDelta = abs(robot.GoalHumid - robot.Floor.getHumidity(room))

        if (robot.isTempGood()):
            robot.changeHumid(room)
        elif (robot.isHumidGood()):
            robot.changeTemp(room)
        elif (humidDelta > tempDelta):
            robot.changeHumid(room)
        else:
            robot.changeTemp(room)


        temp = robot.Floor.getAvgTemperature()
        humid = robot.Floor.getAvgHumidity()
        stdTemp = robot.Floor.getStdDevTemperature()
        stdHumid = robot.Floor.getStdDevHumidity()
        
        print('The average is {:.2f} degrees ({:.2f} deviation)'
                ' and {:.2f}% humidity ({:.2f} deviation).\n'
                .format(temp, stdTemp, humid, stdHumid))

        robot.nextRoom()


    # Simulation over, print results
    print('Finished simulation:')
    robot.Floor.printRooms()

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
