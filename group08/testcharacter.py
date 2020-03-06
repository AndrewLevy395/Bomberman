# This is necessary to find the main code
import sys
import random
sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from colorama import Fore, Back
import heapq
import math


class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]


class TestCharacter(CharacterEntity):

    def __init__(self, name, avatar, x, y):
        CharacterEntity.__init__(self, name, avatar, x, y)
        self.monster_dist = 1
        self.best = 10000000
        self.worst = -self.best*1000
        self.max_depth = 2

    def do(self, wrld):
        # find path and return next best node
        next_node = self.astar((self.x, self.y), wrld.exitcell, wrld)

        # Set best move if there is a path to the exit
        self.set_best_node(next_node)

        # Place bomb to maximize available routes and kill monsters
        self.place_bomb()

        # move to the node chosen by expectimax
        best_node = self.expectimax_argmax(wrld, 0)[0]
        self.move(best_node[0], best_node[1])

        return

    def set_best_node(self, next_node):
        if next_node is not None:
            self.best_action = next_node[0]

    def get_empty_neighbors(self, node, wrld):
        neighbors = []
        for dx in [-1, 0, 1]:
            # check bounds
            if (node[0] + dx >= 0) and (node[0] + dx < wrld.width()):
                for dy in [-1, 0, 1]:
                    # check bounds
                    if (node[1] + dy >= 0) and (node[1] + dy < wrld.height()):
                        # see if empty
                        if wrld.exit_at(node[0] + dx, node[1] + dy) or wrld.empty_at(node[0] + dx, node[1] + dy):
                            if not (dx is 0 and dy is 0):
                                neighbors.append((node[0] + dx, node[1] + dy))
        return neighbors

    # manhattan distance from a to b
    def manhattan_dist(self, a, b):
        (x1, y1) = a
        (x2, y2) = b
        return abs(x1 - x2) + abs(y1 - y2)

    # returns next best node to exit
    def astar(self, start, goal, wrld):
        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0

        while not frontier.empty():
            current = frontier.get()

            if current == goal:
                break

            for next in self.get_empty_neighbors(current, wrld):
                new_cost = cost_so_far[current] + 1
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.manhattan_dist(goal, next)
                    frontier.put(next, priority)
                    came_from[next] = current

        min = math.inf
        for next in came_from:
            dis_to_goal = self.manhattan_dist(next, goal)
            if dis_to_goal < min:
                min = dis_to_goal
                min_to_exit = next
        if next == goal:
            while came_from[next] is not None:
                self.set_cell_color(next[0], next[1], Fore.RED + Back.RED)
                if came_from[next] == start:
                    return next, "EXIT FOUND!"
                next = came_from[next]

        while came_from[min_to_exit] is not None:
            self.set_cell_color(min_to_exit[0], min_to_exit[1], Fore.RED + Back.RED)
            if came_from[min_to_exit] == start:
                return min_to_exit, "NO EXIT FOUND"
            min_to_exit = came_from[min_to_exit]

        # goal can not be reached
        # return a move that get close to the goal
        return None

    # Return the coordinates of the next node to go through
    def expectimax_argmax(self, wrld, depth):
        action = (0, 0)
        maximum = -math.inf
        bomberman = next(iter(wrld.characters.values()))
        bomberman = bomberman[0]
        monster_list = wrld.monsters.values()

        no_monster = 0
        if len(monster_list) == 0:
            no_monster = 1
        elif len(monster_list) == 1:
            monster = next(iter(monster_list))[0]
        else:
            # check which monster is closer
            monster1 = next(iter(monster_list))[0]
            monster2 = next(iter(monster_list))[0]
            if max(monster1.x - bomberman.x, monster1.y - bomberman.y) > max(monster2.x - bomberman.x, monster2.y - bomberman.y):
                monster = monster2
            else:
                monster = monster1

        # look at all bomberman actions
        for bomberman_dx in [-1, 0, 1]:
            # check bounds
            if (bomberman.x + bomberman_dx >= 0) and (bomberman.x + bomberman_dx < wrld.width()):
                for bomberman_dy in [-1, 0, 1]:
                    # check bounds
                    if (bomberman.y + bomberman_dy >= 0) and (bomberman.y + bomberman_dy < wrld.height()):
                        # dont need to check moves that go into walls
                        if not wrld.wall_at(bomberman.x + bomberman_dx, bomberman.y + bomberman_dy):
                            # Set move in wrld
                            bomberman.move(bomberman_dx, bomberman_dy)
                            if no_monster:
                                (new_wrld, new_events) = wrld.next()
                                dist_to_best = self.manhattan_dist((bomberman.x + bomberman_dx, bomberman.y + bomberman_dy), self.best_action)
                                expected_value = self.expectimax_event_value(new_wrld, new_events, depth + 1)
                                expected_value -= dist_to_best
                                if expected_value > maximum:
                                    action = (bomberman_dx, bomberman_dy)
                                    maximum = expected_value
                            else:
                                num_monster_options = 0
                                monster_action_sum = 0

                                # check all monster moves
                                for monster_dx in [-1, 0, 1]:
                                    if (monster.x + monster_dx >= 0) and (monster.x + monster_dx < wrld.width()):
                                        for monster_dy in [-1, 0, 1]:
                                            if ((monster_dx != 0) or (monster_dy != 0)) and\
                                                    (monster.y + monster_dy >= 0) and\
                                                    (monster.y + monster_dy < wrld.height()):
                                                # Remove moves that would go into walls
                                                if not wrld.wall_at(monster.x + monster_dx, monster.y + monster_dy):
                                                    # Increase options and expectimax sum from state
                                                    num_monster_options += 1
                                                    monster_action_sum += self.simulate_monster_action(monster,
                                                                                                       monster_dx,
                                                                                                       monster_dy,
                                                                                                       wrld,
                                                                                                       depth)
                                dist_to_best = self.manhattan_dist((bomberman.x + bomberman_dx, bomberman.y + bomberman_dy), self.best_action)
                                expected_value = monster_action_sum / num_monster_options - dist_to_best
                                # Update expected value if new max found
                                if expected_value > maximum:
                                    action = (bomberman_dx, bomberman_dy)
                                    maximum = expected_value
        return action, maximum

    # Simulates a monster action, returning the expected value from that action
    def simulate_monster_action(self, monster, dx, dy, wrld, depth):
        # do move in world
        monster.move(dx, dy)
        # get next world state
        (new_wrld, new_events) = wrld.next()
        # do something with new world and events
        return self.expectimax_event_value(new_wrld, new_events, depth + 1)

    # Bomberman agent wants to MAXIMIZE score
    def expectimax_event_value(self, wrld, events, depth):
        # If win or death event occur, return
        for event in events:
            if event.tpe == event.BOMB_HIT_CHARACTER or event.tpe == event.CHARACTER_KILLED_BY_MONSTER:
                # bomberman died, return worst value
                return self.worst
            elif event.tpe == event.CHARACTER_FOUND_EXIT:
                # bomberman wins, return best value
                return self.best
        if depth >= self.max_depth:
            # If depth reached, return evaluation
            return self.evaluate(wrld)
        return self.expectimax_argmax(wrld, depth)[1]

    # Evaluation function for a given world state. Returns the value of the state
    def evaluate(self, wrld):
        bomberman = next(iter(wrld.characters.values()))
        bomberman = bomberman[0]
        score = 0
        if len(wrld.monsters.values()) == 0: return score
        monsters = next(iter(wrld.monsters.values()))
        for monster in monsters:
            distx = abs(bomberman.x - monster.x)
            disty = abs(bomberman.y - monster.y)
            if distx <= 2 and disty <= 2:
                if distx <= 1 and disty <= 1:
                    score -= 100000
                score -= 10000
            score -= self.monster_dist*100 / (distx + disty) ** 2
        return score

