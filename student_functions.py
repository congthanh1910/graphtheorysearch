import numpy as np


def ListVertexAdjacent(A, vertex, N):
    VA = []
    for j in range(N):
        if A[vertex][j] != 0:
            VA.append(j)
    return VA


def DFS(matrix, start, end):
    """
    DFS algorithm:
    Parameters:
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer 
        start node
    end: integer
        ending node

    Returns
    ---------------------
    visited
        The dictionary contains visited nodes, each key is a visited node,
        each value is the adjacent node visited before it.
    path: list
        Founded path
    """
    # TODO:
    path = []
    visited = {}
    stack = []

    n = matrix.shape[0]
    graph = {}
    for i in range(n):
        graph[i] = ListVertexAdjacent(matrix, i, n)

    visited[start] = -1
    #print("visited", visited)
    stack.append(start)

    first = 0
    print("graph", graph)
    while stack:
        vertex = stack.pop()
        #print("stack: ", stack)
        #print("vertex: ", vertex)
        #print("first: ", first)
        if first >= len(graph[vertex]):
            first = 0
            if vertex == start:
                break
            else:
                vertex = stack.pop()
        next_node = graph[vertex][first]

        if next_node not in visited:
            visited[next_node] = vertex
            print("visited", visited)
            stack.append(vertex)
            stack.append(next_node)
            #print("stack: ", stack)
            first = 0
        else:
            first += 1
            stack.append(vertex)

    print("visited", visited)

    node_path = end
    while True:
        if node_path in visited:
            path.append(node_path)
            node_path = visited[node_path]
        if node_path == start:
            path.append(node_path)
            break

    path = path[::-1]
    print("path", path)

    return visited, path


def BFS(matrix, start, end):
    """
    BFS algorithm
     Parameters:
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer 
        start node
    end: integer
        ending node

    Returns
    ---------------------
    visited 
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """

    # TODO:

    path = []
    visited = {}
    queue = []
    n = matrix.shape[0]
    graph = {}
    for i in range(n):
        graph[i] = ListVertexAdjacent(matrix, i, n)

    visited[start] = -1
    print("visited", visited)
    queue.append(start)

    print("graph", graph)
    while queue:
        print("Queue", queue)
        vertex = queue.pop(0)
        list_next_node = graph[vertex]
        for i in range(len(list_next_node)):
            if list_next_node[i] not in visited:
                queue.append(list_next_node[i])
                visited[list_next_node[i]] = vertex
                print("Visited", visited)

    node_path = end
    while True:
        if node_path in visited:
            path.append(node_path)
            node_path = visited[node_path]
        if node_path == start:
            path.append(node_path)
            break

    path = path[::-1]
    print("path", path)

    return visited, path


def List_Vertex_Adjacent_and_Distance(A, vertex, N):
    List_Vertex_Adjacent_and_Distance = {}
    for j in range(N):
        if A[vertex][j] != 0:
            List_Vertex_Adjacent_and_Distance[j] = A[vertex][j]
    return List_Vertex_Adjacent_and_Distance

def UCS(matrix, start, end, pos):
    """
    Uniform Cost Search algorithm
     Parameters:
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer 
        start node
    end: integer
        ending node
    pos: dictionary. keys are nodes, values are positions of graph nodes
         từ điển. khóa là nút, giá trị là vị trí của các nút đồ thị
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """
    # TODO:
    path = []
    visited = {}
    queue = []

    n = matrix.shape[0]
    graph = {}
    for i in range(n):
        graph[i] = List_Vertex_Adjacent_and_Distance(matrix, i, n)
    print("Graph", graph)

    visited[start] = (-1, 0)
    queue.append((start, visited[start][0], visited[start][1]))
    print("queue", queue)

    state = queue.pop(0)[0] #state = 1

    while state != end:
        successors = graph[state]
        print(successors)
        for son in successors:
            visitedExist = False
            total_cost = visited[state][1] + successors.get(son)
            for visitedState in visited:
                if (son == visitedState) and (total_cost >= visited.get(visitedState)[1]):
                    visitedExist = True
                    break
            if not visitedExist:
                queue.append((son, state, successors.get(son)))
                visited[son] = (state, total_cost)
        state = queue.pop(0)[0]
    
    print("visited", visited)
    node_path = end
    while True:
        if node_path in visited:
            path.append(node_path)
            node_path = visited[node_path][0]
        if node_path == start:
            path.append(node_path)
            break

    path = path[::-1]
    print("path", path)

    return visited, path

import random

def heuristic_min(data, max_heuristic):
    min = max_heuristic
    vertex = 0
    for i in range(len(data)):
        if data[i][1] < min:
            min = data[i][1]
            vertex = data[i][0]
    return vertex

def bestFirstSearch(matrix, start, end):
    # TODO:
    path = []
    visited = {}
    visited[start] = -1
    heuristic = {}
    
    n = matrix.shape[0]
    graph = {}
    max_heuristic = 100
    for i in range(n):
        heuristic[i] = random.randrange(max_heuristic)
        graph[i] = ListVertexAdjacent(matrix, i, n)
    heuristic[end] = 0
    print("Heuristic", heuristic)
    print("Graph", graph)

    queue = []
    queue.append((start, heuristic[start]))
    
    state = queue.pop(0)[0]
    while state != end:
        successors = graph[state]
        print("successors", successors)
        for son in successors:
            visitedExist = False
            son_heuristic = heuristic[son]
            for visitedState in visited:
                 if (son == visitedState):
                    visitedExist = True
                    break
            if not visitedExist:
                queue.append((son, son_heuristic))
                visited[son] = (state)
        
        min = heuristic_min(queue, max_heuristic)
        vertex_min = 0
        temp = 0
        for i in range(len(queue)):
            if queue[i][0] == min:
                vertex_min = queue[i][0]
                temp = i
        print("vertex_min", vertex_min)
        state = queue.pop(temp)[0]
    
    print("visited", visited)

    node_path = end
    while True:
        if node_path in visited:
            path.append(node_path)
            node_path = visited[node_path]
        if node_path == start:
            path.append(node_path)
            break

    path = path[::-1]
    print("path", path)
    
    return visited, path

def aStar(matrix, start, end):
    # TODO:
    path = []
    visited = {}
    queue = []
    heuristic = {}

    n = matrix.shape[0]
    graph = {}
    max_heuristic = 100
    for i in range(n):
        heuristic[i] = random.randrange(max_heuristic)
        graph[i] = List_Vertex_Adjacent_and_Distance(matrix, i, n)
    print("Heuristic", heuristic)
    print("Graph", graph)

    visited[start] = (-1, 0)
    queue.append((start, visited[start][0], visited[start][1] + heuristic[start]))
    print("queue", queue)

    state = queue.pop(0)[0] #state = 1

    while state != end:
        successors = graph[state]
        print(successors)
        for son in successors:
            visitedExist = False
            total_cost = visited[state][1] + successors.get(son)
            for visitedState in visited:
                if (son == visitedState) and (total_cost >= visited.get(visitedState)[1]):
                    visitedExist = True
                    break
            if not visitedExist:
                queue.append((son, state, heuristic[state] + successors.get(son)))
                visited[son] = (state, total_cost)
        state = queue.pop(0)[0]
    
    print("visited", visited)
    node_path = end
    while True:
        if node_path in visited:
            path.append(node_path)
            node_path = visited[node_path][0]
        if node_path == start:
            path.append(node_path)
            break

    path = path[::-1]
    print("path", path)

    
    return visited, path