import heapq
import matplotlib.pyplot as plt
import matplotlib.patches as patches

class Node:

    def __init__(self, state, parent_node = None, action_from_parent = None, path_cost = 0):
        self.state = state
        self.parent_node = parent_node
        self.action_from_parent = action_from_parent
        self.path_cost = path_cost
        self.depth = 0 if parent_node is None else parent_node.depth + 1

    def __lt__(self, other):
        return self.state < other.state
    
class PriorityQueue:

    def __init__(self, items=(), priority_function=(lambda x: x)):
        self.priority_function = priority_function
        self.pqueue = []
        # add the items to the PQ
        for item in items:
            self.add(item)

    """
    Add item to PQ with priority-value given by call to priority_function
    """
    def add(self, item):
        pair = (self.priority_function(item), item)
        heapq.heappush(self.pqueue, pair)
    """
    pop and return item from PQ with min priority-value
    """
    def pop(self):
        return heapq.heappop(self.pqueue)[1]
    """
    gets number of items in PQ
    """
    def __len__(self):
        return len(self.pqueue)
    
def expand(problem, node):
    s1 = node.state
    for action in problem.actions(s1):
        s2 = problem.result(s1, action)
        cost = node.path_cost + problem.action_cost(s1, action, s2)
        yield Node(state=s2, parent_node=node, action_from_parent=action, path_cost=cost)

def get_path_actions(node):
    actions_list = []
    if node is None or node.parent_node is None:
        return actions_list
    
    while node.parent_node is not None:
        actions_list.append(node.action_from_parent)
        node = node.parent_node
    
    actions_list.reverse()
    
    return actions_list

def get_path_states(node):
    states_list = []
    if node is None:
        return states_list
    
    while node.parent_node is not None:
        states_list.append(node.state)
        node = node.parent_node

    states_list.append(node.state)
    states_list.reverse()
    
    return states_list

def best_first_search(problem, f):
    node = Node(state=problem.initial_state)
    frontier = PriorityQueue(priority_function=f, items=(node,))
    reached = {problem.initial_state: node}
    
    while frontier.__len__() != 0:
        node = frontier.pop()
        if problem.is_goal(node.state):
            return node
        
        for child in expand(problem, node):
            s = child.state
            if s not in reached or child.path_cost < reached[s].path_cost:
                reached[s] = child
                frontier.add(child)
        
    return None

def best_first_search_treelike(problem, f):
    node = Node(state=problem.initial_state)
    frontier = PriorityQueue(priority_function=f, items=(node,))
    
    while frontier.__len__() != 0:
        node = frontier.pop()
        if problem.is_goal(node.state):
            return node
        
        for child in expand(problem, node):
            s = child.state
            frontier.add(child)
        
    return None

def breadth_first_search(problem, treelike=False):
    f = (lambda n: n.depth)

    if treelike:
        return best_first_search_treelike(problem, f)
    else:
        return best_first_search(problem, f)

def depth_first_search(problem, treelike=False):
    f = (lambda n: -n.depth)

    if treelike:
        return best_first_search_treelike(problem, f)
    else:
        return best_first_search(problem, f)

def uniform_cost_search(problem, treelike=False):
    f = (lambda n: n.path_cost)

    if treelike:
        return best_first_search_treelike(problem, f)
    else:
        return best_first_search(problem, f)

def greedy_search(problem, h, treelike=False):
    f = (lambda n: h(n))

    if treelike:
        return best_first_search_treelike(problem, f)
    else:
        return best_first_search(problem, f)

def astar_search(problem, h, treelike=False):
    f = (lambda n: n.path_cost + h(n))

    if treelike:
        return best_first_search_treelike(problem, f)
    else:
        return best_first_search(problem, f)
    
def visualize_route_problem_solution(problem, goal_node, file_name):
    nodes = problem.map_coords.keys()
    edges = problem.map_edges.keys()
    path_of_states = [state[0] for state in get_path_states(goal_node)]
    
    for node in nodes:
        x = problem.map_coords[node][0]
        y = problem.map_coords[node][1]

        if node == problem.initial_agent_loc:
            c = "red"
        elif node == problem.goal_loc:
            c = "green"
        elif node in problem.must_visit:
            c = "blue"
        else:
            c = "black"

        plt.scatter(x, y, marker="s", color=c)

    for edge in edges:
        x1 = problem.map_coords[edge[0]][0]
        y1 = problem.map_coords[edge[0]][1]
        x2 = problem.map_coords[edge[1]][0]
        y2 = problem.map_coords[edge[1]][1]

        plt.arrow(x1, y1, x2-x1, y2-y1, head_width=0, color="black")

    for i in range(len(path_of_states)-1):
        x1 = problem.map_coords[path_of_states[i]][0]
        y1 = problem.map_coords[path_of_states[i]][1]
        x2 = problem.map_coords[path_of_states[i+1]][0]
        y2 = problem.map_coords[path_of_states[i+1]][1]

        plt.arrow(x1, y1, x2-x1, y2-y1, head_width=0.1, color="magenta")
    
    plt.savefig(file_name, format="png")
    plt.close()

def visualize_grid_problem_solution(problem, goal_node, file_name):
    monsters = problem.monster_coords
    foods = problem.food_coords
    path_of_states = [(state[1], state[0]) for state in get_path_states(goal_node)]

    for monster in monsters:
        plt.scatter(monster[1], monster[0], color="black", marker="*", s=2500)

    for food in foods:
        plt.scatter(food[1], food[0], color="green", marker="h", s=1000)

    plt.scatter(problem.initial_agent_loc[1], problem.initial_agent_loc[0], color="red", marker="^", s=1000)

    for i in range(len(path_of_states)-1):
        x1 = path_of_states[i][0]
        y1 = path_of_states[i][1]
        x2 = path_of_states[i+1][0]
        y2 = path_of_states[i+1][1]

        plt.arrow(x1, y1, x2-x1, y2-y1, head_width=0.1, color="magenta")

    plt.ylim([0.5, problem.N + 0.5])
    plt.xlim([0.5, problem.N + 0.5])
    
    plt.savefig(file_name, format="png")
    plt.close()