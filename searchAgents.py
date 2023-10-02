# searchAgents.py
# ---------------
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
In searchAgents.py, you will implement generic search algorithms which are called by
Pacman agents (in game.py).
"""

import random
import util
from game import Agent
from game import Directions

class SearchAgent(Agent):
    """
    This very general search agent finds a path using a supplied search algorithm for a
    supplied search problem, then returns actions to follow that path.
    As a default, this agent runs DFS on a PositionSearchProblem to find the location
    of the nearest food on each move, but it can be configured with any search problem
    and any search algorithm.
    """

    def __init__(self, fn='depthFirstSearch', prob='PositionSearchProblem', heuristic='nullHeuristic'):
        # Warning: some advanced Python magic is employed below to find the right functions and problems
        # as well as their default arguments.
        # fn = search function, prob = search problem, heuristic = heuristic function

        # Read and store the search function.
        if fn not in dir(self):
            raise AttributeError(fn + ' is not a valid search function in SearchAgent')
        self.searchFunction = getattr(self, fn)

        # Read and store the search problem and heuristic.
        if prob not in dir(searchAgents):
            raise AttributeError(prob + ' is not a valid search problem in SearchAgent')
        self.searchType = getattr(searchAgents, prob)
        self.heuristic = getattr(searchAgents, heuristic)

    def registerInitialState(self, state):
        """
        This is called before the first move is made. You can use this
        method to precompute things if needed.
        """
        if self.searchFunction == self.astar or self.searchFunction == self.ucs:
            # Note: A* uses a heuristic, so we must pass a heuristic function to the search function.
            problem = self.searchType(state, searchAgents=state, heuristic=self.heuristic)
        else:
            problem = self.searchType(state, searchAgents=state)
        self.searchProblem = problem
        time_1 = time.time()
        self.actions = self.searchFunction(problem)
        time_2 = time.time()
        with open("q2_time.txt", "w") as text_file:
            text_file.write(str(time_2 - time_1))

        if not self.actions:
            print('Failed to find a path')
            self.actions = []

    def getAction(self, state):
        """
        Returns the next action in the path chosen earlier (in
        registerInitialState).
        """
        if 'actionIndex' not in dir(self):
            self.actionIndex = 0
        i = self.actionIndex
        self.actionIndex += 1
        if i < len(self.actions):
            return self.actions[i]
        else:
            return Directions.STOP

def manhattanHeuristic(position, problem, info={}):
    """
    The Manhattan distance heuristic for a PositionSearchProblem.
    This heuristic is used for A* and uniform cost search.
    """
    xy1 = position
    xy2 = problem.goal
    return abs(xy1[0] - xy2[0]) + abs(xy1[1] - xy2[1])

class CornersProblem(searchAgents.PositionSearchProblem):
    """
    This search problem finds paths through all four corners of a layout.
    You must select a suitable state space and successor function
    """

    def __init__(self, gameState):
        """
        Stores the walls, pacman's starting position, and corners.
        """
        # Store the initial state information
        self.startState = gameState.getAgentPosition(0)
        self.walls = gameState.getWalls()
        self.corners = self.getCorners(gameState)

        # Check if corners exist, else raise an exception
        if len(self.corners) != 4:
            raise ValueError("Exactly four corners should exist in the layout")

        # Initialize the visited corners to False
        self.visitedCorners = [False, False, False, False]

    def getCorners(self, gameState):
        """
        Returns a list of the four corners in the layout.
        """
        corners = []
        corners.append((1, 1))
        corners.append((1, gameState.data.layout.height - 2))
        corners.append((gameState.data.layout.width - 2, 1))
        corners.append((gameState.data.layout.width - 2, gameState.data.layout.height - 2))
        return corners

    def isGoalState(self, state):
        """
        Returns True if the given state is a goal state, which means all four
        corners have been visited.
        """
        return all(self.visitedCorners)

    def getSuccessors(self, state):
        """
        Returns successor states, the actions they require, and a cost of 1.
        """
        successors = []
        for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            x, y = state
            dx, dy = Actions.directionToVector(action)
            next_x, next_y = int(x + dx), int(y + dy)
            if not self.walls[next_x][next_y]:
                next_state = (next_x, next_y)
                # Check if the next state is one of the corners and mark it as visited
                if next_state in self.corners:
                    corner_index = self.corners.index(next_state)
                    self.visitedCorners[corner_index] = True
                successors.append(((next_state, self.visitedCorners), action, 1))
        return successors

    def isStateValid(self, state):
        """
        Returns True if the state is valid (not hitting walls or out of bounds).
        """
        x, y = state
        return not self.walls[x][y]  # Valid if the cell is not a wall

class AStarCornersAgent(SearchAgent):
    """
    A SearchAgent for CornersProblem using A* search and the manhattanHeuristic.
    """
    def __init__(self):
        self.searchFunction = searchAgents.aStarSearch
        self.searchType = CornersProblem

class FoodSearchProblem(searchAgents.SearchProblem):
    """
    A search problem associated with finding food.
    """

    def __init__(self, gameState):
        """
        Stores the start and goal states.
        """
        self.startState = gameState.getPacmanPosition()
        self.foodGrid = gameState.getFood()
        self.walls = gameState.getWalls()
        self.goal = self.findClosestFood(gameState)

    def isGoalState(self, state):
        """
        Returns True if the given state is a goal state, which means the current
        position contains food.
        """
        x, y = state
        return self.foodGrid[x][y]

    def getSuccessors(self, state):
        """
        Returns successor states, the actions they require, and a cost of 1.
        """
        successors = []
        for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            x, y = state
            dx, dy = Actions.directionToVector(action)
            next_x, next_y = int(x + dx), int(y + dy)
            if not self.walls[next_x][next_y]:
                next_state = (next_x, next_y)
                successors.append((next_state, action, 1))
        return successors

    def isStateValid(self, state):
        """
        Returns True if the state is valid (not hitting walls or out of bounds).
        """
        x, y = state
        return not self.walls[x][y]  # Valid if the cell is not a wall

    def findClosestFood(self, gameState):
        """
        Returns the position of the closest food using BFS.
        """
        from util import Queue

        # Create a queue for BFS
        queue = Queue()
        # Create a set to keep track of visited states
        visited = set()
        # Push the start state onto the queue
        queue.push((self.startState, []))

        while not queue.isEmpty():
            state, path = queue.pop()

            # Check if the current state contains food
            if self.isGoalState(state):
                return state

            # If the current state has not been visited, mark it as visited
            if state not in visited:
                visited.add(state)

                # Get successors of the current state
                successors = self.getSuccessors(state)

                for successor, action, _ in successors:
                    if successor not in visited:
                        queue.push((successor, path + [action]))

        # If no food is found, return None
        return None

def closestFoodHeuristic(position, problem, info={}):
    """
    The closest food heuristic for a FoodSearchProblem.
    This heuristic is used for A* search.
    """
    closest_food = problem.findClosestFood(problem.goal)
    if closest_food is not None:
        return util.manhattanDistance(position, closest_food)
    else:
        return 0

def furthestFoodHeuristic(position, problem, info={}):
    """
    The furthest food heuristic for a FoodSearchProblem.
    This heuristic is used for A* search.
    """
    furthest_food = problem.findFurthestFood(position)
    if furthest_food is not None:
        return util.manhattanDistance(position, furthest_food)
    else:
        return 0

class ClosestFoodSearchAgent(SearchAgent):
    """
    A SearchAgent for FoodSearchProblem using A* search and the closestFoodHeuristic.
    """
    def __init__(self):
        self.searchFunction = searchAgents.aStarSearch
        self.searchType = FoodSearchProblem
        self.heuristic = closestFoodHeuristic

class FurthestFoodSearchAgent(SearchAgent):
    """
    A SearchAgent for FoodSearchProblem using A* search and the furthestFoodHeuristic.
    """
    def __init__(self):
        self.searchFunction = searchAgents.aStarSearch
        self.searchType = FoodSearchProblem
        self.heuristic = furthestFoodHeuristic

def studentClosestFoodHeuristic(position, problem, info={}):
    """
    The student's closest food heuristic for a FoodSearchProblem.
    This heuristic is used for A* search.
    """
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def studentFurthestFoodHeuristic(position, problem, info={}):
    """
    The student's furthest food heuristic for a FoodSearchProblem.
    This heuristic is used for A* search.
    """
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

class StudentClosestFoodSearchAgent(SearchAgent):
    """
    A SearchAgent for FoodSearchProblem using A* search and the student's closestFoodHeuristic.
    """
    def __init__(self):
        self.searchFunction = searchAgents.aStarSearch
        self.searchType = FoodSearchProblem
        self.heuristic = studentClosestFoodHeuristic

class StudentFurthestFoodSearchAgent(SearchAgent):
    """
    A SearchAgent for FoodSearchProblem using A* search and the student's furthestFoodHeuristic.
    """
    def __init__(self):
        self.searchFunction = searchAgents.aStarSearch
        self.searchType = FoodSearchProblem
        self.heuristic = studentFurthestFoodHeuristic

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch

# Whether to use student's heuristic functions
USE_STUDENT_HEURISTICS = False

if USE_STUDENT_HEURISTICS:
    closestHeuristic = studentClosestFoodHeuristic
    furthestHeuristic = studentFurthestFoodHeuristic
else:
    closestHeuristic = closestFoodHeuristic
    furthestHeuristic = furthestFoodHeuristic
