class ConstrainedRouteProblem:

    def __init__(self, initial_agent_loc, goal_loc, map_edges, map_coords, must_visit):
        self.initial_agent_loc = initial_agent_loc
        self.goal_loc = goal_loc
        self.map_edges = map_edges
        self.map_coords = map_coords
        self.must_visit = must_visit
        self.initial_state = (initial_agent_loc, False, False)
        for i in range(len(must_visit)):
            self.initial_state += (False,)

    def actions(self, state):
        actions_list = []
        
        for edge in list(self.map_edges.keys()):
            
            if state[0] in edge:
                actions_list.append(edge[0] if edge[0] != state[0] else edge[1])
        
        return actions_list
    
    def result(self, state, action):
        state_list = list(state)

        state_list[0] = action

        if action == self.goal_loc:
            if not state_list[1]:
                state_list[1] = True
            else:
                state_list[2] = True
        
        if action in self.must_visit:
            index = self.must_visit.index(action)
            state_list[index + 3] = True

        return tuple(state_list)
    
    def action_cost(self, state1, action, state2):
        map_edges_list = list(self.map_edges.keys())

        if (state1[0], action) in map_edges_list:
            return self.map_edges[state1[0], action]
        else:
            return self.map_edges[action, state1[0]]
        
    def is_goal(self, state):
        if state[0] == self.goal_loc and state[1] and not state[2]:
            for t in range(3, len(state)):
                if not state[t]:
                    return False
        else:
            return False
        
        return True
    
    def h(self, node):
        if self.is_goal(node.state):
            return 0
        
        goal_location = self.map_coords[self.goal_loc]
        current_location = self.map_coords[node.state[0]]

        return ((goal_location[0] - current_location[0])**2 + (goal_location[1] - current_location[1])**2)**0.5
    
class GridProblemWithMonsters:

    def __init__(self, initial_agent_loc, N, monster_coords, food_coords):
        self.initial_agent_loc = initial_agent_loc
        self.N = N
        self.monster_coords = monster_coords
        self.food_coords = food_coords
        self.initial_state = initial_agent_loc + (0,)
        for i in range(len(food_coords)):
            self.initial_state += (False,)

    def actions(self, state):
        actions_list = []
        mstep = (state[2] + 1) % 4
        futute_monster_coords = []

        for monster_coord in self.monster_coords:
            if mstep == 0:
                futute_monster_coords.append((monster_coord[0], monster_coord[1]))
            if mstep == 1:
                futute_monster_coords.append((monster_coord[0], monster_coord[1]-1))
            if mstep == 2:
                futute_monster_coords.append((monster_coord[0], monster_coord[1]))
            if mstep == 3:
                futute_monster_coords.append((monster_coord[0], monster_coord[1]+1))

        if (state[0]+1, state[1]) not in futute_monster_coords and state[0]+1 <= self.N:
            actions_list.append("up")

        if (state[0]-1, state[1]) not in futute_monster_coords and state[0]-1 >= 1:
            actions_list.append("down")

        if (state[0], state[1]+1) not in futute_monster_coords and state[1]+1 <= self.N:
            actions_list.append("right")

        if (state[0], state[1]-1) not in futute_monster_coords and state[1]-1 >= 1:
            actions_list.append("left")

        if (state[0], state[1]) not in futute_monster_coords:
            actions_list.append("stay")

        return actions_list
    
    def result(self, state, action):
        state_list = list(state)
        state_list[2] = (state_list[2] + 1) % 4

        if action == "up":
            state_list[0] += 1
        elif action == "down":
            state_list[0] -= 1
        elif action == "right":
            state_list[1] += 1
        elif action == "left":
            state_list[1] -= 1
        
        if (state_list[0], state_list[1]) in self.food_coords:
            index = self.food_coords.index((state_list[0], state_list[1]))
            state_list[index + 3] = True

        return tuple(state_list)
    
    def action_cost(self, state1, action, state2):
        return 1
    
    def is_goal(self, state):
        for t in range(3, len(state)):
            if not state[t]:
                return False
        return True
    
    def h(self, node):
        if self.is_goal(node.state):
            return 0
        
        nearest_uneaten_distance = float('inf')

        for food_coord in self.food_coords:
            index = self.food_coords.index(food_coord) + 3
            if not node.state[index]:
                manhattan_distance = abs(node.state[0] - food_coord[0]) + abs(node.state[1] - food_coord[1])
                if manhattan_distance < nearest_uneaten_distance:
                    nearest_uneaten_distance = manhattan_distance

        return nearest_uneaten_distance
        