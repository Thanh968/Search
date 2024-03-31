import pygame
from maze import SearchSpace, Node
from const import *
from heapq import *

# seperate coordinates of a node
def seperate_coordinates(node: Node)->tuple:
    x, y = node.id%COLS, node.id//COLS
    result = (x*(A+A1) + BOUND+15,y*(A+A1) + BOUND + 15)
    return result

# seperate coordinates of all nodes from start node to node
def seperate_points(father: dict, node: Node, start_node: Node)->list:
    result=[]
    cur = node
    result.append(seperate_coordinates(cur))

    while cur != start_node:
        cur = father[cur]
        result.append(seperate_coordinates(cur))

    return result


def draw_multiple_lines(array_of_points: list, sc: pygame.Surface):
    pygame.draw.lines(sc,WHITE,False,array_of_points,2)


def DFS(g: SearchSpace, sc: pygame.Surface)->None:
    # contains frontier nodes
    open_set = [g.start]

    # contains explored nodes
    closed_set = []

    # contains father of node
    father = {}
    clock = pygame.time.Clock()

    while len(open_set) != 0:
        top = open_set[len(open_set) - 1]

        if top != g.start:
            top.set_color(YELLOW,sc)
        pygame.display.flip()
        clock.tick(10)

        if g.is_goal(top):
            top.set_color(PURPLE,sc)
            array_of_points = seperate_points(father,top,g.start)
            draw_multiple_lines(array_of_points,sc)
            pygame.display.flip()
            return
        
        if closed_set.__contains__(top) == False:
            closed_set.append(top)
            if top != g.start:
                top.set_color(BLUE,sc)

            neighbors = g.get_neighbors(top)

            for neighbor in neighbors:
                if closed_set.__contains__(neighbor) == False:
                    father[neighbor] = top
                    if neighbor != g.start:
                        neighbor.set_color(RED,sc)
                    open_set.append(neighbor)
                    pygame.display.flip()
        else:
            top.set_color(BLUE,sc)
            
        
def BFS(g: SearchSpace, sc: pygame.Surface):
    open_set = [g.start]
    closed_set = []
    father = {}
    clock = pygame.time.Clock()

    while len(open_set) != 0:
        top = open_set[0]
        open_set.pop(0)
        if top != g.start:
            top.set_color(YELLOW,sc)
        pygame.display.flip()
        clock.tick(20)
        
        if g.is_goal(top):
            top.set_color(PURPLE,sc)
            array_of_points = seperate_points(father,top,g.start)
            draw_multiple_lines(array_of_points,sc)
            pygame.display.flip()
            return
        
        if closed_set.__contains__(top) == False:
            closed_set.append(top)
            if top != g.start:
                top.set_color(BLUE,sc)

            neighbors = g.get_neighbors(top)

            for neighbor in neighbors:
                if closed_set.__contains__(neighbor) == False and open_set.__contains__(neighbor) == False:
                    open_set.append(neighbor)
                    if neighbor != g.start:
                        neighbor.set_color(RED,sc)
                    father[neighbor] = top
                    pygame.display.flip()
        else:
            top.set_color(BLUE,sc)


def find_index_of_value(open_set, node)->int:
    result = -1
    leng = len(open_set)

    for index in range(leng):
        i, j = open_set[index]
        if j == node:
            result = index
            break

    return result

def UCS(g: SearchSpace, sc: pygame.Surface):
    open_set = [(0,g.start)]
    closed_set = {}
    father = {}
    clock = pygame.time.Clock()

    for node in g.grid_cells:
        closed_set[node] = 0

    while len(open_set) > 0:
        current_cost, current_node = heappop(open_set)
        if current_node != g.start:
            current_node.set_color(YELLOW,sc)
        pygame.display.flip()
        clock.tick(20)

        if g.is_goal(current_node) == True:

            current_node.set_color(PURPLE,sc)
            array_of_points = seperate_points(father,current_node,g.start)
            draw_multiple_lines(array_of_points,sc)
            pygame.display.flip()

            return
        
        neighbors = g.get_neighbors(current_node)

        for node in neighbors:
            is_in_closed_set = (closed_set[node] == 1)
            index_of_node = find_index_of_value(open_set, node)
            is_in_open_set = (index_of_node != -1)

            if is_in_closed_set == False:
                new_cost = current_cost + 1

                if is_in_open_set == False:
                    heappush(open_set,(new_cost,node))
                    if node != g.goal:
                        node.set_color(RED,sc)
                    father[node] = current_node
                else:
                    temp_cost, temp_node = open_set[index_of_node]

                    if temp_cost > new_cost:
                        open_set.pop(index_of_node)
                        open_set.append((new_cost,node))
                        father[node] = current_node
                        heapify(open_set)


        closed_set[current_node] = 1
        if current_node != g.start:
            current_node.set_color(BLUE,sc)
            

def manhattan(node_a: Node, node_b: Node) -> int:
    x_a, y_a = node_a.id%COLS, node_a.id//COLS
    x_b, y_b = node_b.id%COLS, node_b.id//COLS

    result = abs(x_a - x_b) + abs(y_a - y_b)

    return result


def AStar(g: SearchSpace, sc: pygame.Surface):

    # +1 respect if you can implement AStar with a priority queue
    open_set = [(manhattan(g.start,g.goal), g.start)]
    closed_set = {}
    father = {}
    cost = {}
    clock = pygame.time.Clock()

    for i in g.grid_cells:
        closed_set[i] = 0
        cost[i] = 100000

    cost[g.start] = 0

    while len(open_set) > 0:
        current_fvalue, current_node = heappop(open_set)

        if current_node != g.start:
            current_node.set_color(YELLOW, sc)

        pygame.display.flip()
        clock.tick(10)

        if g.is_goal(current_node) == True:
            current_node.set_color(PURPLE,sc)
            array_of_points = seperate_points(father,g.goal,g.start)
            draw_multiple_lines(array_of_points,sc)
            pygame.display.flip()


            return
        
        neighbors = g.get_neighbors(current_node)

        for node in neighbors:
            is_in_closed_set = (closed_set[node] == 1)
            index_of_value = find_index_of_value(open_set,node)
            is_in_open_set = (index_of_value != -1)

            if is_in_closed_set == False:
                cost[node] = min(cost[node], cost[current_node]+1)
                new_fvalue = cost[node] + manhattan(node,g.goal)

                if is_in_open_set ==False:
                    heappush(open_set,(new_fvalue,node))
                    if node != g.goal:
                        node.set_color(RED,sc)
                    father[node] = current_node
                else:
                    temp_fvalue, temp_node = open_set[index_of_value]

                    if temp_fvalue > new_fvalue:
                        open_set.pop(index_of_value)
                        open_set.append((new_fvalue,node))
                        father[node] = current_node
                        heapify(open_set)


        closed_set[current_node] = 1
        if current_node != g.start:
            current_node.set_color(BLUE,sc)




        

    
