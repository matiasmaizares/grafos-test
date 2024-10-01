from collections import defaultdict, deque
import heapq

class Graph:
    def __init__(self, vertices):
        self.V = vertices  # Número de vértices
        self.edges = []    # Lista de aristas (u, v, peso_costo, peso_tiempo)
        self.graph = defaultdict(list)  # Grafo para Dijkstra y Bellman-Ford

    def add_edge(self, u, v, cost, time):
        # Agregar arista para ambos tipos de pesos
        self.edges.append((u, v, cost, time))
        self.graph[u].append((v, cost, time))

    def bellman_ford(self, src):
        """Implementación del algoritmo de Bellman-Ford para detectar ciclos negativos y
        calcular la ruta de menor costo desde un origen a todos los destinos."""
        dist = [float('Inf')] * self.V
        dist[src] = 0

        # Relajar todas las aristas |V| - 1 veces
        for _ in range(self.V - 1):
            for u, v, cost, _ in self.edges:
                if dist[u] != float('Inf') and dist[u] + cost < dist[v]:
                    dist[v] = dist[u] + cost

        # Detectar ciclos negativos
        for u, v, cost, _ in self.edges:
            if dist[u] != float('Inf') and dist[u] + cost < dist[v]:
                raise ValueError("Graph contains negative weight cycle")

        return dist

    def dijkstra(self, src, use_time=False):
        """Implementación de Dijkstra utilizando colas de prioridad para minimizar tiempo o costo."""
        dist = [float('Inf')] * self.V
        dist[src] = 0
        pq = [(0, src)]  # Cola de prioridad: (distancia, nodo)
        
        while pq:
            current_dist, u = heapq.heappop(pq)

            if current_dist > dist[u]:
                continue

            # Recorrer vecinos y aplicar el peso adecuado (costo o tiempo)
            for v, cost, time in self.graph[u]:
                weight = time if use_time else cost
                if dist[u] + weight < dist[v]:
                    dist[v] = dist[u] + weight
                    heapq.heappush(pq, (dist[v], v))

        return dist

    def find_shortest_paths(self, src, destinations, minimize_time=False):
        """Encuentra las rutas más cortas desde el origen a múltiples destinos utilizando
        Bellman-Ford (para ciclos negativos) y Dijkstra (para optimizar tiempo)."""
        try:
            # Usar Bellman-Ford para detectar ciclos negativos
            shortest_paths = self.bellman_ford(src)
        except ValueError:
            print("Ciclo negativo detectado. No se pueden calcular rutas seguras.")
            return None

        # Usar Dijkstra para optimización de tiempo o confirmación de resultados
        if minimize_time:
            shortest_paths = self.dijkstra(src, use_time=True)

        # Mostrar rutas hacia destinos específicos
        return {dest: shortest_paths[dest] for dest in destinations}

# Ejemplo de uso
if __name__ == '__main__':
    g = Graph(5)
    g.add_edge(0, 1, 5, 2)
    g.add_edge(0, 2, 3, 4)
    g.add_edge(1, 3, 6, 1)
    g.add_edge(2, 1, -2, 2)
    g.add_edge(3, 4, 1, 7)

    # Buscar la ruta más corta desde 0 a 3 y 4 minimizando el costo
    print(g.find_shortest_paths(0, [3, 4]))
    
    # Buscar la ruta más corta desde 0 a 3 y 4 minimizando el tiempo
    print(g.find_shortest_paths(0, [3, 4], minimize_time=True))
