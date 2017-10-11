# This was made for one of the projects in this course:
# https://www.edx.org/course/artificial-intelligence-ai-columbiax-csmm-101x-1

import sys
import queue
import time
from math import sqrt

start = time.time()

# copies a two-dimensional array
def copy_board(board):
    n = len(board)
    new_board = []

    for i in range(n):
        new_board.append([])
        for j in range(n):
            new_board[i].append(board[i][j])
            
    return new_board

def goalTest(state):
    board = state.get_info()["board"]
    n = len(board)
    tile = 0
    
    for i in range(n):
        for j in range(n):
            if board[i][j] != tile:
                return False
            tile += 1
    return True

# this makes a queue turn around
def queue_to_list(queue):
    queue_list = []
    
    while not queue.empty():
        queue_list.append(queue.get())
        
    for item in queue_list:
        queue.put(item)
            
    return queue_list

class State(object):
    def __init__(self, board, parent_node, children_nodes, turn, move):
        self.board = board
        self.parent_node = parent_node
        self.children_nodes = children_nodes
        self.turn = turn
        self.move = move
        
    def get_info(self):
        return {"board":self.board, "parent_node":self.parent_node, "children_nodes":self.children_nodes, "turn":self.turn, "move":self.move}
    
    def neighbors(self):
        n = len(self.board)
        
        # a list of stateo objects
        neighbors = []
        
        for i in range(n):
            for j in range(n):
                if self.board[i][j] == 0:
                    if i-1 >= 0:
                        #up
                        neighbor_board = copy_board(self.board)

                        neighbor_board[i][j] = neighbor_board[i-1][j]
                        neighbor_board[i-1][j] = 0
                        
                        neighbor = State(neighbor_board, self, [], self.turn+1, "Up")

                        neighbors.append(neighbor)
                        self.children_nodes.append(neighbor)
                        
                    if i+1 <= n-1:
                        #down
                        neighbor_board = copy_board(self.board)

                        neighbor_board[i][j] = neighbor_board[i+1][j]
                        neighbor_board[i+1][j] = 0

                        neighbor = State(neighbor_board, self, [], self.turn+1, "Down")

                        neighbors.append(neighbor)
                        self.children_nodes.append(neighbor)

                    if j-1 >= 0:
                        #left
                        neighbor_board = copy_board(self.board)

                        neighbor_board[i][j] = neighbor_board[i][j-1]
                        neighbor_board[i][j-1] = 0

                        neighbor = State(neighbor_board, self, [], self.turn+1, "Left")

                        neighbors.append(neighbor)
                        self.children_nodes.append(neighbor)

                    if j+1 <= n-1:
                        #right
                        neighbor_board = copy_board(self.board)

                        neighbor_board[i][j] = neighbor_board[i][j+1]
                        neighbor_board[i][j+1] = 0

                        neighbor = State(neighbor_board, self, [], self.turn+1, "Right")

                        neighbors.append(neighbor)
                        self.children_nodes.append(neighbor)
                        
        return neighbors

#two-dim list
def list_to_tuple(l):
    n = len(l)

    tup = []

    for i in range(n):
        for j in range(n):
            tup.append(l[i][j])

    return tuple(tup)

def breadth_first_search(state):
    frontier = queue.Queue()
    frontier.put(state)
    
    explored = set()
    frontier_set = set()
    frontier_set.add(list_to_tuple(state.get_info()["board"]))
    
    nodes_expanded = 0
    max_fringe_size = 0
    max_search_depth = 0

    while not frontier.empty():

        if frontier.qsize() > max_fringe_size:
            max_fringe_size = frontier.qsize()
        
        state = frontier.get()
        frontier_set.remove(list_to_tuple(state.get_info()["board"]))
        explored.add(list_to_tuple(state.get_info()["board"]))

        if goalTest(state):
            path_to_goal = []
            path_to_goal.append(state.get_info()["move"])
            
            parent_node = state.get_info()["parent_node"]
            while parent_node.get_info()["move"] != "S":
                path_to_goal.append(parent_node.get_info()["move"])
                parent_node = parent_node.get_info()["parent_node"]

            out = open("output.txt", "w")
            out.write(
                "path_to_goal: " + str(path_to_goal[::-1]) + "\n"
                "cost_of_path: " + str(len(path_to_goal)) + "\n"
                "nodes_expanded: " + str(nodes_expanded) + "\n"
                #"fringe_size: " + str(len(frontier_set)) + "\n"
                #"max_fringe_size: " + str(max_fringe_size) + "\n"
                "search_depth: " + str(state.get_info()["turn"]) + "\n"
                "max_search_depth: " + str(max_search_depth) + "\n"
                "running_time: " + str(round((time.time()-start), 8)) + "\n")
            out.close()
            
            return True
        
        nodes_expanded += 1
        for neighbor in state.neighbors():
            if list_to_tuple(neighbor.get_info()["board"]) not in frontier_set:
                if list_to_tuple(neighbor.get_info()["board"]) not in explored:
                    frontier.put(neighbor)
                    frontier_set.add(list_to_tuple(neighbor.get_info()["board"]))
                    
                    if neighbor.get_info()["turn"] > max_search_depth:
                        max_search_depth = neighbor.get_info()["turn"]
                
    return False


def depth_first_search(state):
    stack = []
    stack.append(state)
    stack_set = set()
    stack_set.add(list_to_tuple(state.get_info()["board"])) 
    explored = set()
    nodes_expanded = 0
    max_fringe_size = 0
    max_search_depth = 0
    

    while len(stack) > 0:
        
        if len(stack) > max_fringe_size:
            max_fringe_size = len(stack)
        
        state = stack.pop()
        stack_set.remove(list_to_tuple(state.get_info()["board"]))
        explored.add(list_to_tuple(state.get_info()["board"]))

        if goalTest(state):
            path_to_goal = []
            path_to_goal.append(state.get_info()["move"])
            
            parent_node = state.get_info()["parent_node"]
            while parent_node.get_info()["move"] != "S":
                path_to_goal.append(parent_node.get_info()["move"])
                parent_node = parent_node.get_info()["parent_node"]
            
            out = open("output.txt", "w")
            out.write(
                "path_to_goal: " + str(path_to_goal[::-1]) + "\n"
                "cost_of_path: " + str(len(path_to_goal)) + "\n"
                "nodes_expanded: " + str(nodes_expanded) + "\n"
                #"fringe_size: " + str(len(stack)) + "\n"
                #"max_fringe_size: " + str(max_fringe_size) + "\n"
                "search_depth: " + str(state.get_info()["turn"]) + "\n"
                "max_search_depth: " + str(max_search_depth) + "\n"
                "running_time: " + str(round((time.time()-start), 8)))
            out.close()
            
            return True

        nodes_expanded += 1

        for neighbor in state.neighbors()[::-1]:
            neighbor_tuple = list_to_tuple(neighbor.get_info()["board"])
            
            if neighbor_tuple not in explored:
                if neighbor_tuple not in stack_set:
                    stack.append(neighbor)
                    stack_set.add(neighbor_tuple)
        
                    if neighbor.get_info()["turn"] > max_search_depth:
                        max_search_depth = neighbor.get_info()["turn"]

    return False

# manhattan distance
def heuristic(board):
    n = len(board)

    board_correct = [[0,1,2],[3,4,5],[6,7,8]]
    distance = 0
    
    for i in range(n):
        for j in range(n):
            if board[i][j] != board_correct[i][j] and board[i][j] != 0:
                for i_c in range(n):
                    for j_c in range(n):
                        if board[i][j] == board_correct[i_c][j_c]:
                            distance += abs(i-i_c) + abs(j-j_c)
    return distance
        

def a_star_search(initialState):
    frontier = queue.PriorityQueue()
    frontier.put([heuristic(initialState.get_info()["board"]), 1, initialState])
    
    frontier_set = set()
    frontier_set.add((heuristic(initialState.get_info()["board"]), list_to_tuple(initialState.get_info()["board"])))
    
    explored = set()

    nodes_expanded = 0
    max_fringe_size = 0
    max_search_depth = 0

    while not frontier.empty():
        if frontier.qsize() > max_fringe_size:
            max_fringe_size = frontier.qsize()
        
        state = frontier.get()[2]
        
        frontier_set.remove((heuristic(state.get_info()["board"]), list_to_tuple(state.get_info()["board"])))
        explored.add(list_to_tuple(state.get_info()["board"]))

        if goalTest(state):
            path_to_goal = []
            path_to_goal.append(state.get_info()["move"])
            
            parent_node = state.get_info()["parent_node"]
            while parent_node.get_info()["move"] != "S":
                path_to_goal.append(parent_node.get_info()["move"])
                parent_node = parent_node.get_info()["parent_node"]
            
            out = open("output.txt", "w")
            out.write(
                "path_to_goal: " + str(path_to_goal[::-1]) + "\n"
                "cost_of_path: " + str(len(path_to_goal)) + "\n"
                "nodes_expanded: " + str(nodes_expanded) + "\n"
                #"fringe_size: " + str(frontier.qsize()) + "\n"
                #"max_fringe_size: " + str(max_fringe_size) + "\n"
                "search_depth: " + str(state.get_info()["turn"]) + "\n"
                "max_search_depth: " + str(max_search_depth) + "\n"
                "running_time: " + str(round((time.time()-start), 8)))
            out.close()
            return True

        nodes_expanded += 1
        priority = 0
        for neighbor in state.neighbors():
            neighbor_board = neighbor.get_info()["board"]
            priority += 1
            
            if list_to_tuple(neighbor_board) not in frontier_set and list_to_tuple(neighbor_board) not in explored:
                if (heuristic(neighbor_board), list_to_tuple(neighbor_board)) not in frontier_set:
                    frontier.put([heuristic(neighbor_board), priority, neighbor])
                    frontier_set.add((heuristic(neighbor_board), list_to_tuple(neighbor_board)))

                    if neighbor.get_info()["turn"] > max_search_depth:
                        max_search_depth = neighbor.get_info()["turn"]
    return False

def IDA_star_search(initialState):
    threshold = heuristic(initialState.get_info()["board"])

    while True:
        frontier = []
        frontier_set = set()
        explored = set()

        frontier.append(initialState)
        frontier_set.add(list_to_tuple(initialState.get_info()["board"]))
        nodes_expanded = 0
        max_fringe_size = 0
        max_search_depth = 0

        threshold += 1
        
        while len(frontier) > 0:

            if len(frontier) > max_fringe_size:
                max_fringe_size = len(frontier)
            
            state = frontier.pop()
            frontier_set.remove(list_to_tuple(state.get_info()["board"]))
            
            explored.add(list_to_tuple(state.get_info()["board"]))
        
            if goalTest(state):
                path_to_goal = []
                path_to_goal.append(state.get_info()["move"])
                
                parent_node = state.get_info()["parent_node"]
                while parent_node.get_info()["move"] != "S":
                    path_to_goal.append(parent_node.get_info()["move"])
                    parent_node = parent_node.get_info()["parent_node"]
                
                out = open("output.txt", "w")
                out.write(
                    "path_to_goal: " + str(path_to_goal[::-1]) + "\n"
                    "cost_of_path: " + str(len(path_to_goal)) + "\n"
                    "nodes_expanded: " + str(nodes_expanded) + "\n"
                    #"fringe_size: " + str(len(frontier)) + "\n"
                    #"max_fringe_size: " + str(max_fringe_size) + "\n"
                    "search_depth: " + str(state.get_info()["turn"]) + "\n"
                    "max_search_depth: " + str(max_search_depth) + "\n"
                    "running_time: " + str(round((time.time()-start), 8)))
                out.close()
            
                return True

            nodes_expanded += 1
            for neighbor in state.neighbors()[::-1]:
                neighbor_tuple = list_to_tuple(neighbor.get_info()["board"])
                    
                if neighbor_tuple not in frontier_set:
                    if neighbor_tuple not in explored:

                        if heuristic(neighbor.get_info()["board"]) + neighbor.get_info()["turn"] < threshold:
                            frontier.append(neighbor)
                            frontier_set.add(neighbor_tuple)

                            if neighbor.get_info()["turn"] > max_search_depth:
                                max_search_depth = neighbor.get_info()["turn"]
    return False
    

method = sys.argv[1]
board_input = sys.argv[2].split(",")
b = []

index = 0
n = int(sqrt(len(board_input)))

for i in range(n):
    b.append([])
    for j in range(n):
        b[i].append(int(board_input[index]))
        index += 1

State.state = State(b, None, [], 0, "S")

if method == "bfs":
    breadth_first_search(State.state)
elif method == "dfs":
    depth_first_search(State.state)
elif method == "ast":
    a_star_search(State.state)
elif method == "ida":
    IDA_star_search(State.state)

