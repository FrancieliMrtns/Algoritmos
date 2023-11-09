import collections

options = ['Grafos com busca por profundidade','Grafos Hamiltonianos','Grafos com busca por largura','Árvores Geradoras Mínimas','Algoritmos Gulosos', 'Ordenação Topológica.','finalizar']

print("~~~~~~~~~~~~~~~~~~MENU~~~~~~~~~~~~~~~~~~")

for i in range(0, len(options)):
    print('{}: {}'.format(i+1, options[i]))

print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
selecao = int(input("Selecione um desses algoritmos de acordo com o número: "))
print("Configurando grafo...")
v = int(input("Vértices: "))
na = int(input("Número das arestas: "))
p = input("Seu grafo é ponderado? S/N: ")
if p == "S" or p == "s":
    arestas = []  # Lista para armazenar os valores das arestas
    for i in range(na):
        valor_aresta = int(input(f"Valor da aresta {i+1}: "))
        arestas.append(valor_aresta)
    # Agora, a lista "arestas" contém os valores das arestas
