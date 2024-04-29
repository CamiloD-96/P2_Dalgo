import os
import heapq

class Nodo:
    contador_id = 0  # Variable global para asignar el id automáticamente

    def __init__(self, valor):
        Nodo.contador_id += 1
        self.id = Nodo.contador_id
        self.valor = valor
        self.esCola = False
        self.elemento = False

# Arreglo global para almacenar la salida
salida_global = []

def procesar_caso_prueba(caso_prueba):
    resultado = []
    for elemento in caso_prueba:
        w1 = elemento[1]
        w2 = elemento[2]
        numeros = []
        for atomos in elemento[3]:
            for numero in atomos:
                numeros.append(numero)
        # Crear un diccionario para contar la frecuencia de cada número
        contador = {}
        for num in numeros:
            contador[num] = contador.get(num, 0) + 1
        
        # Aplicar modulo 2 a cada contador
        for key in contador:
            contador[key] %= 2
        
        # Sumar los contadores
        suma_contadores = sum(contador.values())
        
        # Verificar si la suma es igual a 2
        if suma_contadores != 2:
            resultado.append("NO ES POSIBLE")
        else:
            resultado.append(parte2(w1,w2,contador,numeros))  # Llamar a la función parte2 para continuar con el proceso
    
    salida_global.append(resultado)

def parte2(w1, w2, contador, numeros):
    # Crear nodos para cada valor y ajustar el booleano esCola según el contador
    print(numeros)
    nodos = []
    inicio = False
    fin = False
    for valor in numeros:
        es_cola = True if contador[valor] == 1 else False
        nodo = Nodo(valor)
        nodo.esCola = es_cola
        nodo.elemento = True
        nodos.append(nodo)
    # Crear nodos adicionales para cada llave
    noRepetir = []
    for nodo in nodos:
        valor = abs(nodo.valor)
        if valor not in noRepetir:
            nodos.append(Nodo(valor))
            nodos.append(Nodo(valor*-1))
            noRepetir.append(valor)
    print("Nodos creados:")
    for nodo in nodos:
        print(f"ID: {nodo.id}, Valor: {nodo.valor}, esCola: {nodo.esCola}, esElemento: {nodo.elemento}")
        if(inicio and nodo.esCola): 
            fin = True
            nodoFin = nodo.id -1
        if(nodo.esCola and not fin):
            inicio = True
            nodoInicio = nodo.id -1
            
    return parte3(w1,w2,numeros,nodos,nodoInicio,nodoFin)

def parte3(w1,w2,numeros,nodos,nodoInicio,nodoFin):
   # Crear matriz de adyacencia inicializada con valores "infinitos"
    matriz_adyacencia = [[float('inf')] * len(nodos) for _ in range(len(nodos))]

    # Conectar nodos pares con peso 0
    for i in range(0, len(nodos) - 1, 2):
        matriz_adyacencia[i][i + 1] = 0
        matriz_adyacencia[i+1][i] = 0

    # Calcular los pesos de los caminos entre nodos
    for i in range(len(nodos)):
        for j in range(len(nodos)):
            if i != j:
                if matriz_adyacencia[i][j] == float('inf'):
                    peso = calcular_peso_camino(nodos[i], nodos[j], w1, w2)
                    matriz_adyacencia[i][j] = peso
            else:
                matriz_adyacencia[i][j] = 0

    # Imprimir la matriz de adyacencia
    print("Matriz de adyacencia:")
    for fila in matriz_adyacencia:
        print(fila)
    print("/////////////////////////////////////////////////////////")
    distance, path = bellman_ford(matriz_adyacencia, nodoInicio, nodoFin)
    print("Distancia mínima:", distance)
    print("Camino tomado:", path)
    # Asegurar resultado de matriz de adyacencia y retonar
    return "Parte 3"

def calcular_peso_camino(nodo1, nodo2, w1, w2):
    m1, m2 = nodo1.valor, nodo2.valor
    if (nodo1.elemento == True and nodo2.elemento == True):
        return float('inf')
    c1, c2 = 1 if m1 > 0 else -1, 1 if m2 > 0 else -1
    if m1 != m2 and (nodo1.esCola != True and nodo2.esCola != True):
        if c1 == c2 and m1 != m2:
            return 1 + abs(m1 - m2) % w1
        else:
            return w2 - abs(m1 - m2) % w2
    else:
        return float('inf')


def dijkstra(adj_matrix, start, end):
  # Inicializar las distancias a "infinito"
  distances = {node: float('inf') for node in range(len(adj_matrix))}
  distances[start] = 0

  # Inicializar la cola de prioridad con el nodo de inicio
  pq = [(0, start)]

  # Conjunto de nodos visitados
  visited = set()

  # Diccionario de padres para reconstruir el camino
  parents = {}

  # Iterar hasta que se haya alcanzado el nodo de destino
  while end not in visited and pq:
    _, current = heapq.heappop(pq)
    visited.add(current)

    # Actualizar las distancias de los vecinos
    for neighbor in range(len(adj_matrix)):
      weight = adj_matrix[current][neighbor]
      new_distance = distances[current] + weight
      if new_distance < distances[neighbor]:
        distances[neighbor] = new_distance
        heapq.heappush(pq, (new_distance, neighbor))
        parents[neighbor] = current

  # Reconstruir el camino desde el nodo de destino hacia el nodo de inicio
  path = []
  current = end
  while current is not None:
    path.append(current)
    current = parents.get(current)
  path.reverse()

  return distances[end] if distances[end] != float('inf') else None, path

def bellman_ford(adj_matrix, start, end):
    # Inicializar distancias a infinito, excepto para el nodo de inicio
    distances = [float('inf')] * len(adj_matrix)
    distances[start] = 0

    # Inicializar predecesores a None
    predecessors = [None] * len(adj_matrix)

    # Iterar sobre todos los vértices
    for _ in range(len(adj_matrix) - 1):
        # Iterar sobre todas las aristas
        for u in range(len(adj_matrix)):
            for v in range(len(adj_matrix)):
                if adj_matrix[u][v] != float('inf'):
                    if distances[u] + adj_matrix[u][v] < distances[v]:
                        distances[v] = distances[u] + adj_matrix[u][v]
                        predecessors[v] = u

    # Reconstruir el camino desde el nodo de destino hacia el nodo de inicio
    path = []
    current = end
    while current is not None:
        path.append(current)
        current = predecessors[current]
    path.reverse()

    return distances[end], path



def leer_entrada(nombre_archivo):
    if not os.path.exists(nombre_archivo):
        print(f"El archivo '{nombre_archivo}' no existe.")
        return []

    with open(nombre_archivo, 'r') as file:
        lineas = file.readlines()

    casos_prueba = []
    i = 0
    while i < len(lineas):
        num_casos = int(lineas[i].strip())
        casos_prueba_actual = []
        i += 1

        for _ in range(num_casos):
            n, w1, w2 = map(int, lineas[i].split())
            elementos = []

            for _ in range(n):
                atomos = list(map(int, lineas[i+1].split()))
                elementos.append(atomos)
                i += 1

            casos_prueba_actual.append((n, w1, w2, elementos))
            i += 1

        casos_prueba.append(casos_prueba_actual)

    return casos_prueba

# Ejemplo de uso
nombre_archivo = "./P2.in"  # Nombre del archivo 
casos_prueba = leer_entrada(nombre_archivo)

for caso in casos_prueba:
    procesar_caso_prueba(caso)

print(salida_global)
