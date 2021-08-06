import heapq
import time
GoalState = [0, 1, 2, 3, 4, 5, 6, 7, 8]
#start_state = [4,3,2,6,5,0,7,8,1]

##class for the graph node
class Node:
    ## functino used to calculate the heuristic and it takes a boolean to check whether to work
    ## with manhatten or ecludian distance
    def calculate_heuristics(self, state, manhatten_or_ecludian):
        heuristic = 0
        for i in range(len(state)):
            if state[i] != i and state[i] != 0:
                if manhatten_or_ecludian:
                    # h = abs(currentcell:x - goal:x) + abs(currentcell:y - goal:y)
                    heuristic += abs(state[i] % 3 - i % 3) + abs(state[i] // 3 - i // 3)
                else:
                    # h = sqrt((currentcell:x - goal:x)2+ (currentcell.y - goal:y)2)
                    heuristic += ((state[i] % 3 - i % 3) ** 2 + (state[i] // 3 - i // 3) ** 2) ** 0.5
        return heuristic
    #costructor for the Node class
    def __init__(self, state, parent,movement, manhatten_or_ecludian):
        # Contains the state of the node, [list of the state of the board at this node]
        self.state = state
        # Contains the node that generated this node
        self.parent = parent
        self.movement = movement
        if self.parent is not None:
            self.g = parent.g + 1
        else:
            self.g = 0

        self.h = self.calculate_heuristics(state, manhatten_or_ecludian)
        self.f = self.g + self.h
        if self.state:
            self.map = ''.join(str(e) for e in self.state)

    def __lt__(self, other):
        return self.f < other.f

## function to check if the goal is reached
def goal_test(check_state):
    return check_state.state == GoalState

##functino used to compute the neighbours of a given state
def get_neighbours(parent_state, manhatten_or_ecludian):
    state = parent_state.state
    zero_position = state.index(0)

    neighbours = []
    motions = []

    if zero_position % 3 != 0:
        motions.append('l')
    if zero_position % 3 != 2:
        motions.append('r')
    if zero_position // 3 != 0:
        motions.append('u')
    if zero_position // 3 != 2:
        motions.append('d')

    for motion in motions:

        new_state = state[:]
        if motion == 'l':
            new_state[zero_position], new_state[zero_position - 1] = new_state[zero_position - 1], new_state[
                zero_position]
        elif motion == 'r':
            new_state[zero_position], new_state[zero_position + 1] = new_state[zero_position + 1], new_state[
                zero_position]
        elif motion == 'u':
            new_state[zero_position], new_state[zero_position - 3] = new_state[zero_position - 3], new_state[
                zero_position]
        elif motion == 'd':
            new_state[zero_position], new_state[zero_position + 3] = new_state[zero_position + 3], new_state[
                zero_position]
        neighbours.append(Node(new_state, parent_state,motion, manhatten_or_ecludian))

    return neighbours

## function used to check the solvability of the puzzle
## by counting the number of inversions if it is even then it
## is solvable else it is not
def is_solvable(state):
    inv_count = 0
    state_arr = state[:]
    state_arr.remove(0)
    n = len(state_arr)
    for i in range(n):
        for j in range(i + 1, n):
            if state_arr[i] > state_arr[j]:
                inv_count += 1

    if inv_count % 2 == 0:
        return True
    else:
        return False

### main function for the A star algorithm
def a_star(start_state, manhatten_or_ecludian):
    global max_depth
    max_depth = 0
    if not is_solvable(start_state):
        return False, Node([], None," ", manhatten_or_ecludian),[]
    frontier = []
    start_node = Node(start_state, None," ",manhatten_or_ecludian)
    if not is_solvable(start_state):
        return False, start_node,[]
    heapq.heappush(frontier, start_node)
    explored = set()
    in_frontier = set()
    in_frontier.add(start_node.map)
    if not is_solvable(start_state):
        return False, start_node, explored
    while len(frontier) != 0:

        popped_state = heapq.heappop(frontier)
        if popped_state.g > max_depth:
            max_depth = popped_state.g
        in_frontier.remove(popped_state.map)
        explored.add(popped_state.map)
        if goal_test(popped_state):
            return True, popped_state, explored
        neighbours = get_neighbours(popped_state, manhatten_or_ecludian)
        for neighbour in neighbours:
            if not (neighbour.map in explored.union(in_frontier)):
                heapq.heappush(frontier, neighbour)
                in_frontier.add(neighbour.map)
            elif neighbour.map in in_frontier:

                index = 0
                j = 0
                for i in frontier:
                    if i.map == neighbour.map:
                        index = j
                        break
                    j += 1
                frontier[index], frontier[0] = frontier[0], frontier[index]
                test = heapq.heappop(frontier)
                heapq.heapify(frontier)
                heapq.heappush(frontier, neighbour)

## function used to set the start state
def setStartState(start):
    global start_state
    start_state = start


# stuff to run always here such as class/def
def main():
    manhatten_or_ecludian = True
    t0 = time.time()
    result, returned_goal_state ,explored = a_star(start_state, manhatten_or_ecludian)
    t1 = time.time()- t0

    cost = returned_goal_state.f
    global movements
    movements = []
    heuristic_used = "manhatten" if manhatten_or_ecludian else "ecludian"
    print("when using ",heuristic_used)
    print("cost to the goal =\n",cost)
    if result == False:
        print("The puzzle has no solution")
        movements= None
    elif result == True and returned_goal_state.parent is None:
        print("the start state is the goal")
        movements = [None]
    else:

        path = []
        state = returned_goal_state
        while state.parent is not None:
            path.append(state.state)
            state = state.parent
        path.reverse()
        #print("the path to the goal is:\n",path)
        movements= []
        state = returned_goal_state
        while state.parent is not None:
            movements.append(state.movement)
            state = state.parent

    #print("expanded nodes:\n", explored)
    print("The search depth:\n", max_depth)
    print("number of expanded nodes:\n", len(explored))
    print("Running time = ",t1," seconds")

