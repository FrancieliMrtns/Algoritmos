#Introdução de grafo completa :) 

import string

options = ['Grafos com busca por profundidade', 'Grafos Hamiltonianos', 'Grafos com busca por largura',
           'Árvores Geradoras Mínimas', 'Algoritmos Gulosos', 'Ordenação Topológica.', 'finalizar']

print("~~~~~~~~~~~~~~~~~~MENU~~~~~~~~~~~~~~~~~~")

for i in range(len(options)):
    print('{}: {}'.format(i + 1, options[i]))

print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
selecao = int(input("Selecione um desses algoritmos de acordo com o número: "))
print("Configurando grafo...")
vertices = int(input("Total de vértices desejada?: "))

# Verifique se o número de vértices é válido
if vertices < 1 or vertices > 26:
    print("Número de vértices inválido. O número de vértices deve estar entre 1 e 26.")
else:
    # Mapeie os vértices de 'A' a 'Z' e inicialize as arestas como um dicionário vazio
    graph = {string.ascii_uppercase[i]: [] for i in range(vertices)}

    # Adicione arestas
    for i in range(vertices):
        source_vertex = string.ascii_uppercase[i]
        print(f"\nPara o vértice {source_vertex}:")
        num_arestas = int(input("Quantas arestas deseja adicionar? "))
        
        for j in range(num_arestas):
            destination_vertex = input(f"Digite o vértice de destino para a aresta saindo de {source_vertex}: ")
            peso_aresta = int(input(f"Valor da aresta {j + 1} ({source_vertex} -> {destination_vertex}): "))
            graph[source_vertex].append((destination_vertex, peso_aresta))

    # Imprima o grafo resultante
    print("\nGrafo:")
    for vertex, edges in graph.items():
        print(f"{vertex}: {', '.join([f'{dest} ({peso})' for dest, peso in edges])}")
