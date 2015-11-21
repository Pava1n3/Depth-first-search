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
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
<<<<<<< HEAD
    from game import Directions
    """
    - a 'node', which contains its x and y, and information on how to get to this node (the step that was taken to get here, so just its predecessor)
    - a fringe (queue) containing unexplored nodes, add the new nodes to the front of the queue
    - a closed list containing explored nodes
    
    - given an x and a y, test if it is the goal node (isGoalState)
    -- construct a solution if it is 
    -- if not, expand the node (getSuccessors), these new nodes are the front of the queue
    -- the current node is added to the closed list
    - do a recursive call to these steps with the front of the queue
    
    - WHEN the queue is popped, it might be a good idea to test if this node is not in 'closed'
    - I think it is a good idea to test for that here
    """
    
    "node: ([x,y], 'Direction', 1)"
=======

    # node: ((x,y), 'Direction', 1)
>>>>>>> origin/master
    
    # the stack, our 'fringe'
    fringe = util.Stack()
    # 'closed', list of visited nodes
    closed = []
    # list of directions which pacman can use to move
    solution = []
<<<<<<< HEAD
    
    def dfsearch(loc):
        util.raiseNotDefined()
        """if(problem.isGoalState(loc))
            getSolution(loc)
        
        expand(loc)
        
        dfsearch(q.Pop())"""
    
    "I think we might have to use the direction we came from to calculate the [x,y] of the predecessor, find the data of this node in the closed list and iterate on that"
    "find the loc in the closed list, store the action of how to get there in solution, calculate the predecessor, repeat until the predecessor matches the problem.getStartState"
    def getSolution(loc):
        "consider it a help function, so I can locate the full pair in the closed list based on the coordinates [x,y] of a pair"
        cc = map(getFirst(), closed)
        "predecessor"
        a,b,c = closed[cc.index(loc)]
        "add the direction to the front of the solution list"
        solution.insert(0, b)
        
        x,y = a
        
        util.raiseNotDefined()
    
    "expand the fringe"
    def expand(loc):
        util.raiseNotDefined()
    
    "returns the first element of a pair that has 3 elements"
    def getFirst((x, y, z)):
        return x
        
    def getPredCoor((x, y), dir):
        if dir == Directions.NORTH:
            return (x, y - 1)
        else :
            return (x, y)
=======

    location = problem.getStartState()
    
    while not problem.isGoalState(location):
        # put location in closed list
        closed.append(location)
        # get the successors of location
        successors = problem.getSuccessors(location)

        # put all successors not in the closed list in the fringe
        # IF ALL SUCCESSORS ARE IN THIS CLOSED LIST, IT BREAKS!! FIX THIS
        for successor in successors:
            if successor[0] not in closed:
                fringe.push(successor)

        # get the first node in the fringe that is not in the closed list
        node = fringe.pop()

        location = node[0]
        solution.append(node[1])
>>>>>>> origin/master
    
    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    print "Fringe:", fringe.list
    print "Location:", location
    print "Solution:", solution

    return solution
    # util.raiseNotDefined()
    

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
