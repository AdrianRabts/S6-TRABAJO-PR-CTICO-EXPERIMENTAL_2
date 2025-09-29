#Algoritmo de Dijkstra
import heapq

def dijkstra(grafo, inicio):
    distancias = {nodo: float('inf') for nodo in grafo}
    distancias[inicio] = 0
    cola_prioridad = [(0, inicio)]
    
    while cola_prioridad:
        distancia_actual, nodo_actual = heapq.heappop(cola_prioridad)
        if distancia_actual > distancias[nodo_actual]:
            continue
        for vecino, peso in grafo[nodo_actual].items():
            distancia = distancia_actual + peso
            if distancia < distancias[vecino]:
                distancias[vecino] = distancia
                heapq.heappush(cola_prioridad, (distancia, vecino))
    return distancias

grafo = {
    'A': {'B': 2, 'C': 5},
    'B': {'A': 2, 'C': 6, 'D': 1},
    'C': {'A': 5, 'B': 6, 'D': 2, 'E': 5},
    'D': {'B': 1, 'C': 2, 'E': 1},
    'E': {'C': 5, 'D': 1}
}

resultado = dijkstra(grafo, 'A')
print("Distancias mínimas desde A:")
for nodo, distancia in resultado.items():
    print(f"A → {nodo}: {distancia}")

# Algoritmo de Floyd-Warshall
def floyd_warshall(grafo):
    nodos = list(grafo.keys())
    n = len(nodos)
    dist = [[float('inf')] * n for _ in range(n)]
    
    for i in range(n):
        dist[i][i] = 0
    for u in grafo:
        for v, peso in grafo[u].items():
            dist[nodos.index(u)][nodos.index(v)] = peso
    
    for k in range(n):
        for i in range(n):
            for j in range(n):
                if dist[i][k] + dist[k][j] < dist[i][j]:
                    dist[i][j] = dist[i][k] + dist[k][j]
                    
    print("Matriz de distancias mínimas:")
    for fila in dist:
        print(fila)

grafo = {
    'A': {'B': 3, 'C': 8},
    'B': {'C': 2, 'D': 5},
    'C': {'D': 1},
    'D': {'A': 2}
}
floyd_warshall(grafo)

# Algoritmo de Búsqueda en Anchura (BFS)
from collections import deque

def bfs(grafo, inicio, meta):
    visitados = set()
    cola = deque([[inicio]])
    while cola:
        camino = cola.popleft()
        nodo = camino[-1]
        if nodo == meta:
            return camino
        if nodo not in visitados:
            for vecino in grafo[nodo]:
                nuevo_camino = list(camino)
                nuevo_camino.append(vecino)
                cola.append(nuevo_camino)
            visitados.add(nodo)
    return None

grafo = {
    'A': ['B', 'C'],
    'B': ['A', 'D', 'E'],
    'C': ['A', 'F'],
    'D': ['B'],
    'E': ['B', 'F'],
    'F': ['C', 'E']
}
print("Camino BFS de A a F:", bfs(grafo, 'A', 'F'))
