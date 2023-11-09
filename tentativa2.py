import sys

def create_graph():
    graph = []
    for _ in range(int(input("Vértices: "))):
        row = []
        for _ in range(int(input("Número das arestas: "))):
            row.append(int(input("Valor da aresta: ")))
        graph.append(row)
    return graph

def dfs(graph, start, visited):
    visited[start] = True
    print(start, end=" ")

    for neighbor in graph[start]:
        if not visited[neighbor]:
            dfs(graph, neighbor, visited)
while True:
    print("\n1: Grafos com busca por profundidade")
    print("2: Grafos Hamiltonianos")
    print("3: Grafos com busca por largura")
    print("4: Árvores Geradoras Mínimas")
    print("5: Algoritmos Gulosos")
    print("6: Ordenação Topológica")
    print("7: Sair")

    choice = int(input("Escolha o algoritmo desejado: "))

    if choice == 7:
        break

    graph = create_graph()
    start_vertex = int(input("Insira o vértice inicial: "))
    visited = [False] * len(graph)

    if choice == 1:
        dfs(graph, start_vertex, visited)
    # ... Implemente os outros algoritmos da mesma forma