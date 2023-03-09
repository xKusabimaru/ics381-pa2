import heapq

class Node:

    def __init__(self, state, parent_node = None, action_from_parent = None, path_cost = 0):
        self.state = state
        self.parent_node = parent_node
        self.action_from_parent = action_from_parent
        self.path_cost = path_cost
        self.depth = 0 if parent_node is None else parent_node.depth + 1

    def __it__(self, other):
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
        return len(self. pqueue)
    
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
    node = Node(state=problem)
    frontier = PriorityQueue(priority_function=f, items=node)
    reached = {problem.initial_state: node}
    
    while frontier.__len__ != 0:
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
    return None

def breadth_first_search(problem, treelike=False):
    return None

def depth_first_search(problem, treelike=False):
    return None

def uniform_cost_search(problem, treelike=False):
    return None

def greedy_search(problem, h, treelike=False):
    return None

def astar_search(problem, h, treelike=False):
    return None