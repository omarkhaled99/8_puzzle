import sys
import time
GoalState = [0, 1, 2, 3, 4, 5, 6, 7, 8]
# start_state= list()
GoalNode = None

NodesExpanded = 0
MaxSearchDeep = 0
MaxFrontier = 0

## Node class used for the traversal
class Node:
    def __init__(self, state, parent, operator, depth, cost):
        # Contains the state of the node, [list of the state of the board at this node]
        self.state = state
        # Contains the node that generated this node
        self.parent = parent
        # Contains the operation that generated this node from the parent
        self.operator = operator
        # Contains the depth of this node (parent.depth +1)
        self.depth = depth
        # Contains the path cost of this node from depth 0. Not used for depth/breadth first.
        self.cost = cost
        if self.state:
            self.map = ''.join(str(e) for e in self.state)

## function used to move the blank tile
def move(state, idx):
    new_state = state[:]
    index = new_state.index(0)

    # Moves the blank tile up on the board.
    if idx == 0:
        if index not in range(0, board_side):
            new_state[index - board_side], new_state[index] = new_state[index], new_state[index - board_side]
            return new_state
        else:
            return None

    # Moves the blank tile Down on the board.
    elif idx == 1:
        if index not in range(board_len - board_side, board_len):
            new_state[index + board_side], new_state[index] = new_state[index], new_state[index + board_side]
            return new_state
        else:
            return None

    # Moves the blank tile left on the board.
    elif idx == 2:
        if index not in range(0, board_len, board_side):
            new_state[index - 1], new_state[index] = new_state[index], new_state[index - 1]
            return new_state
        else:
            return None

    # Moves the blank tile right on the board
    elif idx == 3:
        if index not in range(board_side - 1, board_len, board_side):
            new_state[index + 1], new_state[index] = new_state[index], new_state[index + 1]
            return new_state
        else:
            return None

## function used to create node
def create_node(state, parent, operator, depth, cost):
    return Node(state, parent, operator, depth, cost)

## function used to expand graph node
def expand_node(node):
    """Returns a list of expanded nodes"""

    expanded_nodes = []

    expanded_nodes.append(create_node(move(node.state, 0), node, "u", node.depth + 1, node.cost + 1 ))
    expanded_nodes.append(create_node(move(node.state, 1), node, "d", node.depth + 1, node.cost + 1 ))
    expanded_nodes.append(create_node(move(node.state, 2), node, "l", node.depth + 1, node.cost + 1 ))
    expanded_nodes.append(create_node(move(node.state, 3), node, "r", node.depth + 1, node.cost + 1 ))

    # Filter the list and remove the nodes that are impossible (move function returned None)
    filteredNodes = [node for node in expanded_nodes if node.state != None]
    return filteredNodes

## main function for the dfs traversal
def dfs(startState):
    global MaxFrontier, GoalNode, MaxSearchDeep
    global max_depth
    max_depth = 0
    boardVisited = set()
    stack = list([create_node(startState, None, None, 0, 0)])

    while stack:
        node = stack.pop()
        if node.depth > max_depth:
            max_depth = node.depth
        boardVisited.add(node.map)

        if node.state == GoalState:
            GoalNode = node
            return stack, boardVisited

        posiblePaths = reversed(expand_node(node))
        for path in posiblePaths:
            if path.map not in boardVisited:
                stack.append(path)
                boardVisited.add(path.map)
    # 		if path.depth > MaxSearchDeep:
    # 			MaxSearchDeep = 1 + MaxSearchDeep

    # if len(stack) > MaxFrontier:
    # 	MaxFrontier = len(stack)
    return None,None

def setStartState(start):
    global start_state
    start_state = start

# stuff to run always here such as class/def
def main():
    global board_len
    board_len = len(start_state)
    global board_side
    board_side = int(board_len ** 0.5)
    global moves

    t0 = time.time()
    result, explored = dfs(start_state)
    t1 = time.time()  -t0
    if result == None:
        print("No solution found")
        moves = None

    elif result == [None]:
        print("Start node was the goal!")
        moves = [None]
    else:
        print("Solution Found")
        moves = []
        path = []
        p = GoalNode
        cost = p.cost
        while p.operator != None:
            moves.append(p.operator)
            path.append(p.state)
            p = p.parent
        path.reverse()
        print("cost to the goal= \n", cost)
       # print("path to the goal:\n", path)
        print("The search depth:\n", max_depth)
       # print("expanded nodes:\n",explored)
        print("number of expanded nodes:\n", len(explored))
        print("Running time = ", t1, " seconds")




