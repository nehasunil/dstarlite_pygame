import heapq
from utils import stateNameToCoords
from math import sqrt

def heuristic(nodeA, nodeB):
    dx = abs(int(nodeA.split('x')[1][0]) - int(nodeB.split('x')[1][0]))
    dy = abs(int(nodeA.split('y')[1][0]) - int(nodeB.split('y')[1][0]))
    return ((sqrt(2.0)-1.0)*min(dx, dy) + max(dx, dy))

def topKey(heap):
    heap.sort()
    if len(heap) > 0:
        return heap[0][:2]
    else:
        return (float('inf'), float('inf'))

def calculateKey(graph, s, s_start, k_m):
    comp = min(graph.graph[s].g, graph.graph[s].rhs)
    key1 = comp + heuristic(s_start, s) + k_m
    return (key1, comp)

def removeFromHeap(graph, heap, s):
    if (graph.graph[s].inHeap):
        id_in_heap = [item for item in heap if s in item]
        if id_in_heap != []:
            if len(id_in_heap) != 1:
                raise ValueError('more than one ' + id + ' in the queue!')
            heap.remove(id_in_heap[0])
    graph.graph[s].inHeap = False

def popHeap(graph, heap):
    poppedNode = heapq.heappop(heap)[2]
    graph.graph[poppedNode].inHeap = False
    return poppedNode

def insert(graph, heap, s, key):
    heapq.heappush(heap, key + (s,))
    graph.graph[s].inHeap = True

def updateVertex(graph, heap, u, s_start, k_m):
    s_goal = graph.goal
    if u != s_goal:
        min_rhs = float('inf')
        for n in graph.graph[u].successors:
            min_rhs = min(
                min_rhs, graph.graph[n].g + graph.graph[u].successors[n])
        graph.graph[u].rhs = min_rhs
        removeFromHeap(graph, heap, u)
        if graph.graph[u].rhs != graph.graph[u].g:
            insert(graph, heap, u, calculateKey(graph, u, s_start, k_m))

def computeShortestPath(graph, heap, s_start, k_m):
    while (graph.graph[s_start].rhs != graph.graph[s_start].g) or (topKey(heap) < calculateKey(graph, s_start, s_start, k_m)):
        k_old = topKey(heap)
        u = popHeap(graph, heap)
        if k_old < calculateKey(graph, u, s_start, k_m):
            insert(graph, heap, u, calculateKey(graph, u, s_start, k_m))
        elif graph.graph[u].g > graph.graph[u].rhs:
            graph.graph[u].g = graph.graph[u].rhs
            for n in graph.graph[u].neighbors:
                neighbor_coords = stateNameToCoords(n)
                # in order to be a predecessor, neighbor is not occupied
                if (graph.cells[neighbor_coords[1]][neighbor_coords[0]] != -1):
                    updateVertex(graph, heap, n, s_start, k_m)
        else:
            graph.graph[u].g = float('inf')
            updateVertex(graph, heap, u, s_start, k_m)
            for n in graph.graph[u].neighbors:
                neighbor_coords = stateNameToCoords(n)
                # in order to be a predecessor, neighbor is not occupied
                if (graph.cells[neighbor_coords[1]][neighbor_coords[0]] != -1):
                    updateVertex(graph, heap, n, s_start, k_m)


def nextInShortestPath(graph, s_current):
    min_rhs = float('inf')
    s_next = None
    if graph.graph[s_current].rhs == float('inf'):
        print('There is no known path to the goal')
    else:
        for i in graph.graph[s_current].successors:
            child_cost = graph.graph[i].g + graph.graph[s_current].successors[i]
            if (child_cost) < min_rhs:
                min_rhs = child_cost
                s_next = i
        if s_next:
            return s_next
        else:
            raise ValueError('could not find child for transition!')


def scanForObstacles(graph, heap, s_current, scan_range, k_m):
    states_to_update = {}
    range_checked = 0
    if scan_range >= 1:
        for neighbor in graph.graph[s_current].successors:
            neighbor_coords = stateNameToCoords(neighbor)
            states_to_update[neighbor] = graph.cells[neighbor_coords[1]
                                                     ][neighbor_coords[0]]
        range_checked = 1

    while range_checked < scan_range:
        new_set = {}
        for state in states_to_update:
            new_set[state] = states_to_update[state]
            for neighbor in graph.graph[state].successors:
                if neighbor not in new_set:
                    neighbor_coords = stateNameToCoords(neighbor)
                    new_set[neighbor] = graph.cells[neighbor_coords[1]
                                                    ][neighbor_coords[0]]
        range_checked += 1
        states_to_update = new_set

    new_obstacle = False
    for state in states_to_update:
        if states_to_update[state] < 0:
            for neighbor in graph.graph[state].successors:
                # first time to observe this obstacle where one wasn't before
                if(graph.graph[state].successors[neighbor] != float('inf')):
                    neighbor_coords = stateNameToCoords(state)
                    graph.cells[neighbor_coords[1]][neighbor_coords[0]] = -2
                    graph.graph[neighbor].successors[state] = float('inf')
                    graph.graph[state].successors[neighbor] = float('inf')
                    updateVertex(graph, heap, state, s_current, k_m)
                    new_obstacle = True
    return new_obstacle


def moveAndRescan(graph, heap, s_current, scan_range, k_m):
    if(s_current == graph.goal):
        return 'goal', k_m
    else:
        s_last = s_current
        s_new = nextInShortestPath(graph, s_current)
        new_coords = stateNameToCoords(s_new)

        if(graph.cells[new_coords[1]][new_coords[0]] == -1):  # just ran into new obstacle
            s_new = s_current  # need to hold tight and scan/replan first

        results = scanForObstacles(graph, heap, s_new, scan_range, k_m)
        k_m += heuristic(s_last, s_new)
        computeShortestPath(graph, heap, s_current, k_m)

        return s_new, k_m


def initDStarLite(graph, heap, s_start, s_goal, k_m):
    graph.graph[s_goal].rhs = 0
    insert(graph, heap, s_goal, calculateKey(graph, s_goal, s_start, k_m))
    computeShortestPath(graph, heap, s_start, k_m)

    return (graph, heap, k_m)
