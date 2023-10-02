# search.py
# ---------
# Licensing Information:  You are free to use or extend this codebase for educational purposes.
# Credits must be included in all assignments, projects, and labs when using this code.
# For example, include the following in your report:
#
# Code provided by AI for Everyone (ai4eonline.org)
# Extended and modified for use in CSE140 by [Your Name]
# (include your name in the comments if you extend or modify the code)
#
# This code is adapted from the UC Berkeley Pacman Project:
# http://ai.berkeley.edu/search.html

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
        raise NotImplementedError()

    def isGoalState(self, state):
        """
        Returns True if and only if the state is a valid goal state.
        """
        raise NotImplementedError()

    def getSuccessors(self, state):
        """
        Returns a list of triples, (successor, action, stepCost), where 'successor'
        is a successor to the current state, 'action' is the action
        required to get there, and 'stepCost' is the incremental cost of expanding
        to that successor.
        """
        raise NotImplementedError()

    def getCostOfActions(self, actions):
        """
        Returns the cost of a particular sequence of actions. If those actions
        include an illegal move, return 999999.  This is implemented for you.
        """
        if actions is None:
            return 999999
        x, y = self.getStartState()
        cost = 0
        for action in actions:
            # figure out the next state and see whether it's valid
            x, y = x + util.dx[action], y + util.dy[action]
            if not self.isStateValid((x, y)):
                return 999999
            cost += 1
        return cost

    def isStateValid(self, state):
        """
        Returns True if the state is valid (not hitting walls or out of bounds).
        """
        raise NotImplementedError()


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
    # Create a stack for DFS
    stack = util.Stack()
    # Push the start state and an empty path onto the stack
    stack.push((problem.getStartState(), []))
    # Create a set to keep track of visited states
    visited = set()

    while not stack.isEmpty():
        state, path = stack.pop()

        # Check if the current state is the goal state
        if problem.isGoalState(state):
            return path

        # If the current state has not been visited, mark it as visited
        if state not in visited:
            visited.add(state)

            # Get successors of the current state
            successors = problem.getSuccessors(state)

            for successor, action, _ in successors:
                if successor not in visited:
                    stack.push((successor, path + [action]))

    # If no path to the goal is found, return an empty list
    return []

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
    goal in the provided SearchProblem. This heuristic is trivially 0 for all states.
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
