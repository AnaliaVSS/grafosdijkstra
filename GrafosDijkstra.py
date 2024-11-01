import heapq  # Heapq se utiliza para usar colas de prioridad
import numpy as np  # Numpy permite manejar matrices de manera eficiente

class GrafoDirigido:
    def __init__(self):
        self.adyacencia = {}  # Diccionario para la lista de adyacencia

    def agregar_nodo(self, nodo): #Agrega un nodo al grafo.
        self.adyacencia.setdefault(nodo, [])  # Inicializa una lista vacía para el nodo si no existe

    def agregar_arista(self, origen, destino, peso=1): #Agrega una arista dirigida con un peso (por defecto 1) al grafo.
        if origen in self.adyacencia and destino in self.adyacencia:
            self.adyacencia[origen].append((destino, peso))  # Añade el destino y peso a la lista de adyacencia

    def mostrar_lista_adyacencia(self): #Muestra la lista de adyacencia del grafo.
        for nodo, aristas in self.adyacencia.items():
            print(f"{nodo}: {aristas}")

    def generar_matriz_adyacencia(self): #Genera una matriz de adyacencia a partir de la lista de adyacencia.
        nodos = list(self.adyacencia)
        n = len(nodos)
        matriz = np.zeros((n, n), dtype=int)  # Inicializa una matriz n x n llena de ceros
        for origen, destinos in self.adyacencia.items():
            i = nodos.index(origen)
            for destino, peso in destinos:
                j = nodos.index(destino)
                matriz[i, j] = peso  # Asigna el peso en la posición correcta
        return matriz, nodos

    def mostrar_matriz_adyacencia(self): #Muestra la matriz de adyacencia del grafo.
        matriz, nodos = self.generar_matriz_adyacencia()  # Genera la matriz de adyacencia
        print("  ", " ".join(nodos))  # Imprime los nombres de los nodos como cabecera
        for nodo, fila in zip(nodos, matriz):
            print(nodo, " ", " ".join(map(str, fila)))  # Imprime cada fila de la matriz

    def dijkstra(self, inicio): #Implementa el algoritmo de Dijkstra para encontrar el camino más corto desde el nodo 'inicio'.
        distancias = {nodo: float('inf') for nodo in self.adyacencia}  # Inicializa las distancias como infinito
        distancias[inicio] = 0  # La distancia al nodo de inicio es 0
        pq = [(0, inicio)]  # Cola de prioridad inicializada con el nodo de inicio
        padres = {nodo: None for nodo in self.adyacencia}  # Diccionario para almacenar el nodo anterior

        while pq:
            distancia_actual, nodo_actual = heapq.heappop(pq)  # Extrae el nodo con la menor distancia
            for vecino, peso in self.adyacencia[nodo_actual]:
                nueva_distancia = distancia_actual + peso  # Calcula la nueva distancia
                if nueva_distancia < distancias[vecino]:  # Si la nueva distancia es menor, actualiza
                    distancias[vecino] = nueva_distancia
                    padres[vecino] = nodo_actual
                    heapq.heappush(pq, (nueva_distancia, vecino))  # Añade el vecino a la cola de prioridad

        return distancias, padres

    def mostrar_camino_mas_corto(self, inicio, fin): #Muestra el camino más corto y su costo desde 'inicio' hasta 'fin' usando Dijkstra.
        distancias, padres = self.dijkstra(inicio)  # Ejecuta Dijkstra para obtener las distancias y padres
        if distancias[fin] == float('inf'):
            print(f"No hay camino de {inicio} a {fin}")
            return
        camino, nodo_actual = [], fin
        while nodo_actual:
            camino.insert(0, nodo_actual)  # Reconstruye el camino desde el nodo final al inicio
            nodo_actual = padres[nodo_actual]
        print(f"Camino más corto de {inicio} a {fin}: {' -> '.join(camino)} con un coste de {distancias[fin]}")

# EJEMPLO DE FUNCIONAMIENTO
grafo = GrafoDirigido() # Añade nodos al grafo
for nodo in ['A', 'B', 'C', 'D']:
    grafo.agregar_nodo(nodo) # Añade aristas al grafo
for origen, destino, peso in [('A', 'B', 1), ('A', 'C', 4), ('B', 'C', 2), ('B', 'D', 5), ('C', 'D', 1)]:
    grafo.agregar_arista(origen, destino, peso)

grafo.mostrar_lista_adyacencia()  # Muestra la lista de adyacencia
grafo.mostrar_matriz_adyacencia()  # Muestra la matriz de adyacencia
grafo.mostrar_camino_mas_corto('A', 'D')  # Muestra el camino más corto de A a D
