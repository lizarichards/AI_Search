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

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"

    # print("Start:", problem.getStartState())
    # print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    # print("Start's successors:", problem.getSuccessors(problem.getStartState()))

    # fringe = util.Stack()  # make the fringe a stack to store all the nodes
    # nodes_visited = []  #keep track of all the visited nodes
    #
    # # have to include the state and the other info needed to reconstruct the path/plan to get to the state
    # fringe.push((problem.getStartState(), [], ))  # the second argument is the path
    #
    #
    # while not fringe.isEmpty():
    #     popped = fringe.pop()
    #     state = popped[0]  # gets the state of the popped node
    #     path = popped[1]  # gets the direction of the current popped node
    #     if problem.isGoalState(state):
    #         break
    #     else:
    #         if state not in nodes_visited:
    #             nodes_visited.append(state)  # add the node to nodes visited list
    #             succ = problem.getSuccessors(state)  # look into the children of the unvisited nodes
    #             for child in succ:  # look at each of the children of the parent node
    #                 child_state = child[0]  # get the child state
    #                 child_path = child[1]  # get the child direction
    #                 if child_state not in nodes_visited:  # if child has not been visited
    #                     nodes_visited.append(child_state)  # add the child state to nodes visited
    #                     child_path = path + child_path  # have to add to path in this order
    #                     # child_path += path  # add the path it took to get to the child node
    #                     fringe.push((child_state, child_path))  # add child to stack since it has not been visited

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))

    fringe = util.Stack()
    nodes_visited = []
    # start_state = list(problem.getStartState)
    fringe.push((problem.getStartState(), [], 1))

    while not fringe.isEmpty():
        popped = fringe.pop()
        state = popped[0]
        direction = popped[1]

        if problem.isGoalState(state):
            return direction
            # return popped?

        if state not in nodes_visited:
            nodes_visited.append(state)

            succ = problem.getSuccessors(state)
            if succ not in nodes_visited:
                nodes_visited.append(succ)
            for child in succ:
                child_state = child[0]
                child_direction = child[1]
                if child_state not in nodes_visited:
                    child_direction = direction + [child_direction]
                    fringe.push((child_state, child_direction, 1))
                    # fringe.push(child)



    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"


    fringe = util.Queue() # takes the start
    nodes_visited = []

    start_state = list(problem.getStartState())
    fringe.push((problem.getStartState(), [], 1))

    print()
    while not fringe.isEmpty():

        dequeued = fringe.pop()
        node = dequeued[0]
        direction = dequeued[1]

        if problem.isGoalState(node):
            return direction

        if node not in nodes_visited:
            nodes_visited.append(node)

        neighbor = problem.getSuccessors(node)
        if neighbor not in nodes_visited:
            nodes_visited.append(neighbor)
        for child in neighbor:
            child_state = child[0]
            child_direction = child[1]
            if child_state not in nodes_visited:
                child_direction = direction + [child_direction]
                fringe.push((child_state, child_direction, 1))


    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"

    fringe = util.PriorityQueue()
    nodes_visited = []
    fringe.push((problem.getStartState(), [], 0))

    while not fringe.isEmpty():
        node = fringe.pop()
        state = node[0]
        direction = node[1]

        if problem.isGoalState(state):
            return direction
            # return popped?

        if state not in nodes_visited:
            nodes_visited.append(state)

            succ = problem.getSuccessors(state)
            if succ not in nodes_visited:
                nodes_visited.append(succ)  # are these two lines needed?
            for child in succ:
                child_state = child[0]
                child_direction = child[1]
                if child_state not in nodes_visited:
                    child_direction = direction + [child_direction]
                    cost = problem.getCostOfActions(child_direction)
                    fringe.push((child_state, child_direction, 1), cost)
                    # fringe.push(child)


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
    fringe = util.PriorityQueue()
    nodes_visited = []
    fringe.push((problem.getStartState(), [], 0), heuristic(problem.getStartState(), problem))

    while not fringe.isEmpty():
        node = fringe.pop()
        state = node[0]
        direction = node[1]

        if problem.isGoalState(state):
            return direction
            # return popped?

        if state not in nodes_visited:
            nodes_visited.append(state)

            succ = problem.getSuccessors(state)
            if succ not in nodes_visited:
                nodes_visited.append(succ)  # are these two lines needed?
            for child in succ:
                child_state = child[0]
                child_direction = child[1]
                if child_state not in nodes_visited:
                    child_direction = direction + [child_direction]
                    cost = problem.getCostOfActions(child_direction)
                    fringe.push((child_state, child_direction, 1), cost + heuristic(child_state, problem))
                    # fringe.push(child)

    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
