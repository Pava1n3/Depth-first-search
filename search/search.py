# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    #   _______________
    #  /               \
    # /     R.I.P.      \
    # |                 |
    # | Marijn's ideeën |
    # |                 |
    # |   Moge zij in   |
    # |  vrede rusten   |
    # |                 |
    # |   15-11-2015    |
    # |   21-11-2015    |
    # |                 |
    # |_________________|

    # state: (currentLocation, directionFromPrevtoCurr, cost)
    # node:  (currentState, previousState)
    #       ((currentLocation, directionFromPrevToCurr, cost), (previousLocation, _, cost))

    from game import Directions
    s = Directions.STOP
    
    # the stack, our 'fringe'
    # an item in this stack is a node
    fringe = util.Stack()

    # 'closed', the list of visited nodes
    # an item in this list is a node
    closed = []

    # list of directions which pacman can use to move
    solution = []
    
    # put the start node in the fringe
    startState = problem.getStartState()
    node = (startState, s, 1), (startState, s, 1)
    fringe.push(node)

    while not fringe.isEmpty():
        # get a node from the fringe
        node = fringe.pop()

        # check if currentLocation is not a currentLocation in the closed list
        if node[0][0] not in [x[0][0] for x in closed]:
            # if it isn't, put it in the closed list
            closed.append(node)

            # if this node is the goal, stop
            if problem.isGoalState(node[0][0]):
                goal = node[0]
                break

            # otherwise, put all successors in the fringe
            successors = problem.getSuccessors(node[0][0])
            for successor in successors:
                fringe.push((successor, node[0]))

    # converting the list to a dictionary
    # to allow searching a key (the location)
    # for its value (the previous location and the direction)
    closed = dict(closed)

    # while you can go to the goal and while the direction is not STOP
    while goal in closed and goal[1] != s:
        # put the direction in the front of the solution
        solution.insert(0, goal[1])
        # set the previous state as the new goal
        goal = closed[goal]

    return solution
    

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    from game import Directions
    s = Directions.STOP
    
    # the stack, our 'fringe'
    # an item in this stack is a node
    fringe = util.Queue()

    # 'closed', the list of visited nodes
    # an item in this list is a node
    closed = []

    # list of directions which pacman can use to move
    solution = []
    
    # put the start node in the fringe
    startState = problem.getStartState()
    node = (startState, s, 1), (startState, s, 1)
    fringe.push(node)

    while not fringe.isEmpty():
        # get a node from the fringe
        node = fringe.pop()

        # check if currentLocation is not a currentLocation in the closed list
        if node[0][0] not in [x[0][0] for x in closed]:
            # if it isn't, put it in the closed list
            closed.append(node)

            # if this node is the goal, stop
            if problem.isGoalState(node[0][0]):
                goal = node[0]
                break

            # otherwise, put all successors in the fringe
            successors = problem.getSuccessors(node[0][0])
            for successor in successors:
                fringe.push((successor, node[0]))

    # converting the list to a dictionary
    # to allow searching a key (the location)
    # for its value (the previous location and the direction)
    closed = dict(closed)

    # while you can go to the goal and while the direction is not STOP
    while goal in closed and goal[1] != s:
        # put the direction in the front of the solution
        solution.insert(0, goal[1])
        # set the previous state as the new goal
        goal = closed[goal]

    print solution
    return solution

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    """
    procedure UniformCostSearch(Graph, start, goal)
    node ← start
    cost ← 0
    frontier ← priority queue containing node only
    explored ← empty set
    do
        if frontier is empty
          return failure
        node ← frontier.pop()
        if node is goal
          return solution
        explored.add(node)
        for each of node's neighbors n
          if n is not in explored
            if n is not in frontier
              frontier.add(n)
            else if n is in frontier with higher cost
              replace existing node with n
    """
    
    "We will need to use getCostOfActions probably, as this returns the correct cost of moving somewhere"
    
    from game import Directions
    
    # state: (currentLocation, directionFromPrevtoCurr, 1)
    # node:  (currentLocation, (previousLocation, directionFromPrevToCurr, costForThisStep))
    
    #For when you literally can't even
    #STOP has a step cost of 1
    s = Directions.STOP
    
    #Priority queue, data in the queue should be pushed as push(item, cost)
    fringe = util.PriorityQueue()
    
    #List of explored nodes
    closed = []
    
    # list of directions which pacman can use to move
    solution = []
    
    # put the start node in the fringe
    startLocation = problem.getStartState()
    node = (startLocation, (startLocation, s, 0))
    fringe.push(node, 0)
    goal = ()
    
    while not fringe.isEmpty():
        node = fringe.pop()
        
        # check if the is not in the closed list
        if node[0] not in dict(closed):
            # if it isn't, put it in the closed list
            closed.append(node)
            
            # if this node is the goal, stop
            if problem.isGoalState(node[0]):
                goal = node
                break
                
            # otherwise, put all successors in the fringe
            successors = problem.getSuccessors(node[0])
            for successor in successors:
                #checking if something is already in the fringe is discouraged, just add it again with a new priority. This is what I understood from the comments in util.py, in the priorityqueue class
                cost = successor[2] + node[1][2]
                newNode = (successor[0], (node[0], successor[1], cost))
                fringe.push(newNode, cost)
    
    # converting the list to a dictionary
    # to allow searching a key (the location)
    # for its value (the previous location and the direction)
    closed = dict(closed)

    # put the (proper) directions in the solution list
    while goal[0] in closed and goal[1] != s:
        solution.append(closed[goal[0]][1])
        goal = closed[goal[0]]
        # print "Goal node:", goal

    # since the solution went from the goal to the starting point
    # the list has to be reversed
    solution.reverse()

    # remove the very first element
    solution = solution[1:]

    
    return solution

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    from game import Directions
    
    # state: (currentLocation, directionFromPrevtoCurr, cost)
    # node:  (currentLocation, (previousLocation, directionFromPrevToCurr, cost))
    
    #For when you literally can't even
    #STOP has a step cost of 1
    s = Directions.STOP
    
    #Priority queue, data in the queue should be pushed as push(item, cost)
    fringe = util.PriorityQueue()
    
    #List of explored nodes
    closed = []
    
    # list of directions which pacman can use to move
    solution = []
    
    # put the start node in the fringe
    startLocation = problem.getStartState()
    node = (startLocation, (startLocation, s, 0))
    fringe.push(node, 0)
    goal = ()
    
    while not fringe.isEmpty():
        node = fringe.pop()
        
        # check if the is not in the closed list
        if node[0] not in dict(closed):
            # if it isn't, put it in the closed list
            closed.append(node)
            
            # if this node is the goal, stop
            if problem.isGoalState(node[0]):
                goal = node
                break
                
            # otherwise, put all successors in the fringe
            successors = problem.getSuccessors(node[0])
            for successor in successors:
                cost = successor[2] + node[1][2]
                newNode = (successor[0], (node[0], successor[1], cost))
                fringe.push(newNode, cost + heuristic(successor[0], problem))
    
    # converting the list to a dictionary
    # to allow searching a key (the location)
    # for its value (the previous location and the direction)
    closed = dict(closed)

    # put the (proper) directions in the solution list
    while goal[0] in closed and goal[1] != s:
        solution.append(closed[goal[0]][1])
        goal = closed[goal[0]]
        # print "Goal node:", goal

    # since the solution went from the goal to the starting point
    # the list has to be reversed
    solution.reverse()

    # remove the very first element
    solution = solution[1:]

    
    return solution


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
