#CÓDIGO PRINCIPAL


#Francieli Martins
import string #Usado para manipulaçao de caracteries, focado neste caso em gerar letras maiúsculas para o alfabeto,
import heapq #fornece funções para implementar filas de prioridade usando heaps. Isso será usado em algoritmos como Dijkstra e Prim.


#Listinha para exibir os tipos de algoritmos
options = ['Grafos com busca por profundidade', 'Grafos Hamiltonianos', 'Grafos com busca por largura',
           'Árvores Geradoras Mínimas', 'Algoritmos Gulosos', 'Ordenação Topológica.', 'finalizar']

#enfeite
print("~~~~~~~~~~~~~~~~~~MENU~~~~~~~~~~~~~~~~~~")

#loop para mostrar os itens da lista options numerando as opções
for i in range(len(options)):
    print('{}: {}'.format(i + 1, options[i]))

#enfeite
print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
#Escolher o tipo de algoritmo baseado no número que tava no menu
selecao = int(input("Selecione um desses algoritmos de acordo com o número: "))
print("Configurando grafo...")
#obtem a informação da quantidade de vértices para a criação do grafo
vertices = int(input("Total de vértices desejada?: "))

# Conferindo se a letra do vértice é válida
if vertices < 1 or vertices > 26:
    print("Número de vértices inválido. O número de vértices deve estar entre 1 e 26.")
else:
    graph = {string.ascii_uppercase[i]: [] for i in range(vertices)}

#Armazenamento da resposta baseada em número
tem_peso = int(input("Suas arestas têm peso? Sim = 1; Não = 2: "))
#se a aresta tiver peso...
if tem_peso == 1:
    for i in range(vertices):  #laço com o número total de vértices já determinado na variável "vertices"
        source_vertex = string.ascii_uppercase[i] #Define a variável source_vertex como a letra do alfabeto correspondente ao índice atual. Essa letra representa o vértice atual.
        print(f"\nPara o vértice {source_vertex}:") # Mostra qual vértice as informações estão sendo coletadas
        num_arestas = int(input("Quantas arestas deseja adicionar? ")) #Oferece oportunidade de adicionar quantas arestas quiser

        #Loop para acrescentar valor e destino para as aretas; "num_arestas" representa o número de arestas no vértice atual
        for j in range(num_arestas):
            destination_vertex = input(f"Digite o vértice de destino para a aresta saindo de {source_vertex}: ")
            peso_aresta = int(input(f"Valor da aresta {j + 1} ({source_vertex} -> {destination_vertex}): ")) #Acrescenta valor da aresta; j+1 representa o índice da iteração no loop for j in range(num_arestas)
            graph[source_vertex].append((destination_vertex, peso_aresta)) #Adiciona a aresta ao grafo. Se o grafo ainda não possui uma lista de arestas para o vértice atual, cria uma lista vazia. Adiciona à lista a tupla (destination_vertex, peso_aresta) representando a aresta com destino e peso.
#se a aresta NÃO tiver peso...
elif tem_peso == 2:
    for i in range(vertices): #laço com o número total de vértices já determinado na variável "vertices"
        source_vertex = string.ascii_uppercase[i] #source_vertex como a letra do alfabeto, ou seja, como o vértice atual
        print(f"\nPara o vértice {source_vertex}:") #mostra de qual vértice ta sendo coletado
        num_arestas = int(input("Quantas arestas deseja adicionar? "))

        #Loop para acrescentar valor e destino para as aretas; "num_arestas" representa o número de arestas no vértice atual
        for j in range(num_arestas):
            destination_vertex = input(f"Digite o vértice de destino para a aresta saindo de {source_vertex}: ")
            graph[source_vertex].append(destination_vertex) #Adiciona o vértice de destino à lista de arestas do vértice atual no grafo. Neste caso, como as arestas não têm peso, apenas o vértice de destino é adicionado.

#se não for nenhuma das outras opções exibir:
else:
    print("Escolha inválida. Reinicie o programa.")


#Impressão fo grafo formado.
print("\nGrafo:")

#Itera sobre os itens do dicionário graph, onde vertex representa o vértice atual e edges é a lista de arestas associada a esse vértice.
for vertex, edges in graph.items():
   #conferindo se tem arestas no vértice trabalhado
    if edges:
      #lista vazia para pegar as arestas formatas
        formatted_edges = []
        #Itera sobre cada aresta na lista de arestas liagda ao vértice.
        for edge in edges:
            if isinstance(edge, tuple) and len(edge) == 2: #É uma tupla? Tem dois elementos? Então a aresta tem um peso.
                dest, peso = edge #Desempacota a tupla, atribuindo seus elementos às variáveis dest (destino) e peso.
                formatted_edges.append(f'{dest} ({peso})') #Representação formatada da aresta (destino com peso) à lista formatted_edges.
            else:
                formatted_edges.append(str(edge)) #Add representação formatada da aresta (somente destino) à lista formatted_edges.
        print(f"{vertex}: {', '.join(formatted_edges)}") #Se não tiver arestas ele vai printar "sem arestas"
    else:
        print(f"{vertex}: sem arestas")

# ------------------------------------------------------------------------------

#Yuri Moriyama Colucci
#BUSCA POR PROFUNDIDADE

if selecao == 1:
  import networkx as nx  #Para manipular grafos
import matplotlib.pyplot as plt  #Para visualização

#Função de busca em profundidade
def dfs(graph, start):
    visited, stack = set(), [start]  #Conjunto de nós visitados e pilha para a DFS
    while stack:  #Enquanto houver nós na pilha2
        vertex = stack.pop()  #Remove o último nó da pilha
        if vertex not in visited:  #Se o nó não foi visitado
            visited.add(vertex)  #Adiciona o nó aos visitados
            stack.extend(graph[vertex] - visited)  #Adiciona à pilha os vizinhos não visitados do nó atual
    return visited  #Retorna os nós visitados durante a DFS

#Criação de um grafo G e adição de arestas
G = nx.Graph()
G.add_edges_from([(1, 2), (1, 3), (2, 4), (3, 5), (4, 6), (5, 6)])

#Calcula o layout dos nós do grafo G usando o algoritmo spring_layout do NetworkX
pos = nx.spring_layout(G)

#Desenha os nós do grafo G
nx.draw_networkx_nodes(G, pos)
#Desenha as arestas do grafo G
nx.draw_networkx_edges(G, pos)
#Adiciona rótulos aos nós do grafo G
nx.draw_networkx_labels(G, pos)

#Mostra o gráfico do grafo usando Matplotlib
plt.show()



#Yuri Moriyama Colucci
#HAMILTONIANOS
if selecao == 2:
       import matplotlib.pyplot as plt #Importa a biblioteca NetworkX para manipular grafos
import networkx as nx

# Função para encontrar um caminho Hamiltoniano em um grafo
def hamilton(G):
    F = [(G,[next(iter(G.nodes()))])]  # Fila com grafo e caminho iniciado com um nó arbitrário
    n = G.number_of_nodes()  # Número total de nós no grafo
    while F:
        graph, path = F.pop()  # Retira o grafo e caminho da fila
        confs = []  # Lista para armazenar possíveis caminhos
        for node in graph.neighbors(path[-1]):  # Para cada vizinho do último nó no caminho atual
            conf_p = path[:]  # Copia o caminho atual
            conf_p.append(node)  # Adiciona o próximo nó ao caminho
            conf_g = nx.Graph(graph)  # Copia do grafo atual
            conf_g.remove_node(path[-1])  # Remove o último nó visitado
            confs.append((conf_g, conf_p))  # Adiciona o novo caminho e grafo à lista de configurações possíveis
        for g, p in confs:
            if len(p) == n:  # Se o caminho encontrado tiver todos os nós do grafo
                return p  # Retorna o caminho Hamiltoniano
            else:
                F.append((g, p))  # Adiciona o novo grafo e caminho à fila
    return None  # Retorna None se nenhum caminho Hamiltoniano for encontrado

# Criação de um grafo
G = nx.Graph()
edges = [(1, 2), (1, 3), (2, 3), (2, 4), (3, 4)]  # Arestas do grafo
G.add_edges_from(edges)  # Adiciona as arestas ao grafo G

# Encontra um ciclo Hamiltoniano no grafo G
H = hamilton(G)

# Layout dos nós do grafo G usando o algoritmo spring_layout do NetworkX
pos = nx.spring_layout(G)

# Desenha os nós do grafo G
nx.draw_networkx_nodes(G, pos)
# Adiciona rótulos aos nós do grafo G
nx.draw_networkx_labels(G, pos)
# Desenha as arestas do grafo G
nx.draw_networkx_edges(G, pos, edgelist=edges)

# Mostra o gráfico do grafo
plt.show()

# Verifica se um ciclo Hamiltoniano foi encontrado e imprime o ciclo se existir
if not H:
    print("O gráfico não tem um ciclo hamiltoniano.")
else:
    print("Um ciclo hamiltoniano no gráfico é:")
    print(H)


# ------------------------------------------------------------------------------

#Francieli Martins
#BUSCA POR LARGURA
if selecao == 3:
  if tem_peso == 1:
    print("Executando Dijkstra")
    def dijkstra(grafo, inicio, destino):
      if inicio not in grafo or destino not in grafo:
          return None

      fila = [(0, inicio, [inicio])]

      while fila:
          peso_total, vertice, caminho = heapq.heappop(fila)

          if vertice == destino:
              return caminho

          for vizinho, peso_aresta in grafo.get(vertice, []):
              novo_caminho = caminho + [vizinho]
              novo_peso_total = peso_total + peso_aresta
              heapq.heappush(fila, (novo_peso_total, vizinho, novo_caminho))

      return None

  elif tem_peso == 2:
    print("Algoritmo de Busca em Largura")
    def busca_largura(grafo, inicio, destino):
        if inicio not in grafo or destino not in grafo:
            return None

        fila = [(inicio, [inicio])]

        while fila:
            vertice, caminho = fila.pop(0)

            if vertice == destino:
                return caminho

            for vizinho in grafo.get(vertice, []):
                if vizinho not in caminho:
                    novo_caminho = caminho + [vizinho]
                    fila.append((vizinho, novo_caminho))

        return None

  print("Busca por Largura:")
  start_vertex = input("Digite o vértice de partida para a busca em largura: ")
  end_vertex = input("Digite o vértice de destino para a busca em largura: ")

  caminho_largura = busca_largura(graph, start_vertex, end_vertex)

  if caminho_largura:
      print("Caminho mais curto:", ' -> '.join(caminho_largura))
  else:
      print("Não há caminho entre os vértices.")


#Francieli Martins
# ÁRVORES GERADORAS MÍNIMAS
if selecao == 4: #se selecionou arvores geradoras mínimas...
    if tem_peso == 1:
        def prim(graph, start_vertex): # Declaração da função prim, que recebe um grafo e um vértice inicial.
            visited = set() #Vai servir para "rastrear" quais vértices foram visitados
            min_heap = [(0, start_vertex, None)] #heap para fila de prioridade; tupla inicial representando a aresta de peso 0 saindo do vértice inicial.
            minimum_spanning_tree = [] #lista para armazenamento do algoritmo da árvore

            #enquanto tiver arestas na nossa "fila de prioridade"...
            while min_heap:
                weight, current_vertex, previous_vertex = heapq.heappop(min_heap) # Remoção da menor aresta da fila de prioridade.

                #Verifica se o vértice atual já foi visitado.
                if current_vertex not in visited:
                    visited.add(current_vertex) #Add o vértice atual ao conjunto de vértices visitados.

                    #Conferir se não estamo sno priemri vértice
                    if previous_vertex is not None:
                        minimum_spanning_tree.append((previous_vertex, current_vertex, weight)) #Add a aresta à árvore geradora mínima.

                    for neighbor, edge_weight in graph[current_vertex]:# Itera sobre os vizinhos do vértice atual no grafo.
                        if neighbor not in visited: #O vizinho foi visitado?
                            heapq.heappush(min_heap, (edge_weight, neighbor, current_vertex)) #Add aresta à fila de prioridade.

            return minimum_spanning_tree

        # Escolha do vértice inicial
        vertice_inicial_prim = input("Digite o vértice inicial para iniciar o algoritmo de Prim: ")

        # Verifique se o vértice inicial é válido
        if vertice_inicial_prim not in graph:
            print("Vértice inicial inválido. Reinicie o programa.")
        else:
            #Execução do algoritmo de prim -->Executa o algoritmo de Prim com o grafo e o vértice inicial fornecidos, armazenando a árvore geradora mínima resultante na variável arvore_minima_prim.
            arvore_minima_prim = prim(graph, vertice_inicial_prim)

            # Imprima a árvore geradora mínima resultante
            print("\nÁrvore Geradora Mínima de Prim:")
            for aresta in arvore_minima_prim:
                if len(aresta) == 2:  # Aresta sem peso
                    origem, destino = aresta
                    print(f"{origem} - {destino}")
                elif len(aresta) == 3:  # Aresta com peso
                    origem, destino, peso = aresta
                    print(f"{origem} - {destino} ({peso})")

    elif tem_peso == 2:
        print("Não é possível construir uma árvore geradora de valor mínimo sem os valores")

# ------------------------------------------------------------------------------

#Maria Eduarda
#ALGORITIMOS GULOSOS
elif selecao == 5:

  # Importa as bibliotecas necessárias


# Função para criar um grafo com base nas entradas do usuário
 def criar_grafo():
    # Solicita ao usuário o número de vértices e arestas
    num_vertices = int(input("Digite o número de vértices: "))
    num_arestas = int(input("Digite o número de arestas: "))

    # Cria um grafo vazio
    grafo = nx.Graph()

    # Adiciona vértices ao grafo
    for i in range(1, num_vertices + 1):
        grafo.add_node(i)

    # Adiciona arestas ao grafo com pesos
    for _ in range(num_arestas):
        origem = int(input("Digite o vértice de origem da aresta: "))
        destino = int(input("Digite o vértice de destino da aresta: "))
        peso = int(input("Digite o peso da aresta: "))
        grafo.add_edge(origem, destino, weight=peso)

    return grafo

# Função que aplica o algoritmo de Prim para encontrar a Árvore de Abrangência Mínima
def algoritmo_guloso_arvore_geradora_minima_prim(grafo):
    # Aplica o algoritmo de Prim para encontrar a Árvore de Abrangência Mínima
    arvore_minima = nx.minimum_spanning_tree(grafo, algorithm='prim')
    return arvore_minima

# Função principal que executa o código
def main():
    # Cria um grafo com base nas informações do usuário
    grafo = criar_grafo()

    # Desenha o grafo original
    pos = nx.spring_layout(grafo)
    nx.draw(grafo, pos, with_labels=True, font_weight='bold', node_color='lightblue', font_color='black', font_size=8, node_size=700, edgelist=grafo.edges())
    plt.title("Grafo Original")
    plt.show()

    # Aplica o algoritmo guloso para encontrar a Árvore de Abrangência Mínima (Prim)
    arvore_minima = algoritmo_guloso_arvore_geradora_minima_prim(grafo)

    # Desenha a Árvore de Abrangência Mínima
    pos_arvore_minima = nx.spring_layout(grafo)
    nx.draw(grafo, pos_arvore_minima, with_labels=True, font_weight='bold', node_color='lightblue', font_color='black', font_size=8, node_size=700, edgelist=arvore_minima.edges(), edge_color='red', width=2)
    plt.title("Árvore de Abrangência Mínima (Prim)")
    plt.show()

# Verifica se o código está sendo executado diretamente (não importado como módulo)
if __name__ == "__main__":
    main()


#MARIA EDUARDA
# OEDENAÇÃO TOPOLOGICA
def ordenacao_topologica(grafo):
    # Lista que irá armazenar a ordem topológica
    ordem_topologica = []

    # Dicionário para armazenar os graus de entrada de cada vértice
    graus_entrada = {v: 0 for v in grafo}

    # Calcula os graus de entrada
    for vizinhos in grafo.values():
        for vizinho in vizinhos:
            # Incrementa o grau de entrada do vizinho
            if vizinho in graus_entrada:
                graus_entrada[vizinho] += 1
            else:
                # Se o vizinho não for um vértice válido, exibe uma mensagem de erro
                print(f"Erro: Vizinho {vizinho} não é um vértice válido.")

    # Cria uma fila de vértices com grau de entrada zero
    fila = [v for v in grafo if graus_entrada[v] == 0]

    # Processa a fila removendo vértices e atualizando os graus de entrada
    while fila:
        vertice = fila.pop()
        ordem_topologica.append(vertice)

        # Atualiza os graus de entrada dos vizinhos do vértice removido
        for vizinho in grafo[vertice]:
            graus_entrada[vizinho] -= 1
            # Adiciona vizinhos com grau de entrada zero à fila
            if graus_entrada[vizinho] == 0:
                fila.append(vizinho)

    # Retorna a ordem topológica
    return ordem_topologica

# Função para criar um grafo com entrada do usuário
def criar_grafo():
    # Dicionário para armazenar o grafo
    grafo = {}
    # Obtém o número de vértices do usuário
    num_vertices = int(input("Digite o número de vértices: "))

    # Preenche o grafo com os vizinhos de cada vértice
    for i in range(num_vertices):
        while True:
            try:
                # Solicita ao usuário os vizinhos do vértice i
                vizinhos = input(f"Digite os vértices vizinhos do vértice {i} separados por espaço (ou 'enter' para nenhum): ").strip()
                # Converte a entrada em uma lista de inteiros, apenas se forem números válidos
                grafo[i] = [int(v) for v in vizinhos.split() if v.isdigit() and int(v) < num_vertices]
                break
            except ValueError:
                # Exibe uma mensagem de erro se o usuário inserir algo que não é um número
                print("Erro: Insira apenas números válidos.")

    # Retorna o grafo criado
    return grafo

# Exemplo de uso
# Solicita ao usuário a criação do grafo
grafo_usuario = criar_grafo()
# Chama a função de ordenação topológica com o grafo criado
ordem = ordenacao_topologica(grafo_usuario)
# Exibe a ordem topológica resultante
print("Ordem topológica:", ordem)

import networkx as nx

def caminho_mais_curto(grafo, origem, destino):
    # Utiliza o algoritmo de Dijkstra para encontrar o caminho mais curto
    caminho = nx.shortest_path(grafo, source=origem, target=destino, weight='weight')
    comprimento = nx.shortest_path_length(grafo, source=origem, target=destino, weight='weight')

    return caminho, comprimento

def main():
    # Cria um grafo ponderado de exemplo
    grafo_exemplo = nx.Graph()
    grafo_exemplo.add_weighted_edges_from([(1, 2, 3), (1, 3, 1), (2, 3, 2), (2, 4, 1), (3, 4, 4)])

    # Define os vértices de origem e destino
    origem = 1
    destino = 4

    # Chama o algoritmo para encontrar o caminho mais curto
    caminho_resultante, comprimento_resultante = caminho_mais_curto(grafo_exemplo, origem, destino)

    # Exibe o resultado
    print(f"Caminho mais curto de {origem} para {destino}:")
    print(caminho_resultante)
    print(f"Comprimento Total: {comprimento_resultante}")

if __name__ == "__main__":
    main()

import networkx as nx
import matplotlib.pyplot as plt

def criar_grafo():
    # Solicita ao usuário o número de vértices e arestas
    num_vertices = int(input("Digite o número de vértices: "))
    num_arestas = int(input("Digite o número de arestas: "))

    # Cria um grafo vazio
    grafo = nx.Graph()

    # Adiciona vértices ao grafo
    for i in range(1, num_vertices + 1):
        grafo.add_node(i)

    # Adiciona arestas ao grafo com pesos
    for _ in range(num_arestas):
        origem = int(input("Digite o vértice de origem da aresta: "))
        destino = int(input("Digite o vértice de destino da aresta: "))
        peso = int(input("Digite o peso da aresta: "))
        grafo.add_edge(origem, destino, weight=peso)

    return grafo

def algoritmo_guloso_arvore_geradora_minima_prim(grafo):
    # Aplica o algoritmo de Prim para encontrar a Árvore de Abrangência Mínima
    arvore_minima = nx.minimum_spanning_tree(grafo, algorithm='prim')
    return arvore_minima

def main():
    # Cria um grafo com base nas informações do usuário
    grafo = criar_grafo()

    # Desenha o grafo original
    pos = nx.spring_layout(grafo)
    nx.draw(grafo, pos, with_labels=True, font_weight='bold', node_color='lightblue', font_color='black', font_size=8, node_size=700, edgelist=grafo.edges())
    plt.title("Grafo Original")
    plt.show()

    # Aplica o algoritmo guloso para encontrar a Árvore de Abrangência Mínima (Prim)
    arvore_minima = algoritmo_guloso_arvore_geradora_minima_prim(grafo)

    # Desenha a Árvore de Abrangência Mínima
    pos_arvore_minima = nx.spring_layout(grafo)
    nx.draw(grafo, pos_arvore_minima, with_labels=True, font_weight='bold', node_color='lightblue', font_color='black', font_size=8, node_size=700, edgelist=arvore_minima.edges(), edge_color='red', width=2)
    plt.title("Árvore de Abrangência Mínima (Prim)")
    plt.show()

if __name__ == "__main__":
    main()

import networkx as nx
import matplotlib.pyplot as plt

def criar_grafo():
    # Solicita ao usuário o número de vértices e arestas
    num_vertices = int(input("Digite o número de vértices: "))
    num_arestas = int(input("Digite o número de arestas: "))

    # Cria um grafo vazio
    grafo = nx.Graph()

    # Adiciona vértices ao grafo
    for i in range(1, num_vertices + 1):
        grafo.add_node(i)

    # Adiciona arestas ao grafo com pesos
    for _ in range(num_arestas):
        origem = int(input("Digite o vértice de origem da aresta: "))
        destino = int(input("Digite o vértice de destino da aresta: "))
        peso = int(input("Digite o peso da aresta: "))
        grafo.add_edge(origem, destino, weight=peso)

    return grafo

def algoritmo_guloso_arvore_geradora_minima_prim(grafo):
    # Aplica o algoritmo de Prim para encontrar a Árvore de Abrangência Mínima
    arvore_minima = nx.minimum_spanning_tree(grafo, algorithm='prim')
    return arvore_minima

def main():
    # Adiciona uma escolha para o usuário (neste caso, escolha 1 para executar o código)
    selecao = int(input("Escolha o algoritmo (1 para Prim): "))

    if selecao == 1:
        # Cria um grafo com base nas informações do usuário
        grafo = criar_grafo()

        # Desenha o grafo original
        pos = nx.spring_layout(grafo)
        nx.draw(grafo, pos, with_labels=True, font_weight='bold', node_color='lightblue', font_color='black', font_size=8, node_size=700, edgelist=grafo.edges())
        plt.title("Grafo Original")
        plt.show()

        # Aplica o algoritmo guloso para encontrar a Árvore de Abrangência Mínima (Prim)
        arvore_minima = algoritmo_guloso_arvore_geradora_minima_prim(grafo)

        # Desenha a Árvore de Abrangência Mínima
        pos_arvore_minima = nx.spring_layout(grafo)
        nx.draw(grafo, pos_arvore_minima, with_labels=True, font_weight='bold', node_color='lightblue', font_color='black', font_size=8, node_size=700, edgelist=arvore_minima.edges(), edge_color='red', width=2)
        plt.title("Árvore de Abrangência Mínima (Prim)")
        plt.show()
    else:
        print("Opção inválida.")

if __name__ == "__main__":
    main()

#Maria Eduarda
#ALGORITIMOS GULOSOS
elif selecao == 5:


  def troco_moedas(sistema_moedas, valor):
    # Ordena as moedas disponíveis em ordem decrescente
    sistema_moedas = sorted(sistema_moedas, reverse=True)

    # Inicializa uma lista para armazenar as moedas do troco
    troco = []

    # Loop pelas moedas disponíveis
    for moeda in sistema_moedas:
        # Enquanto o valor restante for maior ou igual ao valor da moeda
        while valor >= moeda:
            # Adiciona a moeda ao troco
            troco.append(moeda)
            # Subtrai o valor da moeda do valor total
            valor -= moeda

    # Retorna a lista de moedas do troco
    return troco

def main():
    print("Bem-vindo ao sistema de troco de moedas!")

    # Solicita ao usuário as moedas disponíveis como entrada
    sistema_moedas = [int(moeda) for moeda in input("Digite as moedas disponíveis separadas por espaço: ").split()]

    # Solicita ao usuário o valor para o qual deseja receber o troco
    valor_troco = int(input("Digite o valor para o qual deseja receber o troco: "))

    # Chama a função de troco de moedas e armazena o resultado
    troco = troco_moedas(sistema_moedas, valor_troco)

    # Exibe o resultado formatado
    print(f"\nTroco para {valor_troco} centavos:")
    print(troco)

# Verifica se o script está sendo executado como o programa principal

import networkx as nx

def caminho_mais_curto(grafo, origem, destino):
    # Utiliza o algoritmo de Dijkstra para encontrar o caminho mais curto
    caminho = nx.shortest_path(grafo, source=origem, target=destino, weight='weight')
    comprimento = nx.shortest_path_length(grafo, source=origem, target=destino, weight='weight')

    return caminho, comprimento

def criar_grafo():
    # Solicita ao usuário o número de vértices e arestas
    num_vertices = int(input("Digite o número de vértices: "))
    num_arestas = int(input("Digite o número de arestas: "))

    # Cria um grafo vazio
    grafo = nx.Graph()

    # Adiciona vértices ao grafo
    for i in range(1, num_vertices + 1):
        grafo.add_node(i)

    # Adiciona arestas ao grafo com pesos
    for _ in range(num_arestas):
        origem = int(input("Digite o vértice de origem da aresta: "))
        destino = int(input("Digite o vértice de destino da aresta: "))
        peso = int(input("Digite o peso da aresta: "))
        grafo.add_edge(origem, destino, weight=peso)

    return grafo

def main():
    grafo_exemplo = None



        if opcao == '0':
            break
        elif opcao == '1':
            # Cria o grafo ponderado de exemplo
            grafo_exemplo = nx.Graph()
            grafo_exemplo.add_weighted_edges_from([(1, 2, 3), (1, 3, 1), (2, 3, 2), (2, 4, 1), (3, 4, 4)])
            break
        elif opcao == '2':
            # Cria um novo grafo com base na entrada do usuário
            grafo_exemplo = criar_grafo()
            break
        else:
            print("Opção inválida. Tente novamente.")

    if grafo_exemplo:
        # Solicita ao usuário os vértices de origem e destino
        origem = int(input("Digite o vértice de origem: "))
        destino = int(input("Digite o vértice de destino: "))

        # Chama o algoritmo para encontrar o caminho mais curto
        caminho_resultante, comprimento_resultante = caminho_mais_curto(grafo_exemplo, origem, destino)

        # Exibe o resultado
        print(f"\nCaminho mais curto de {origem} para {destino}:")
        print(caminho_resultante)
        print(f"Comprimento Total: {comprimento_resultante}")

if __name__ == "__main__":
    main()

import networkx as nx

def cobertura_minima_vertices(grafo):
    vertices_selecionados = set()

    # Ordena as arestas com base no número de vizinhos
    arestas_ordenadas = sorted(grafo.edges(), key=lambda x: len(set(x[0]).union(set(x[1]))))

    # Itera sobre as arestas ordenadas
    for aresta in arestas_ordenadas:
        vertices_selecionados.update(aresta)

    return list(vertices_selecionados)

def caixeiro_viajante_vizinho_mais_proximo(grafo):
    caminho = [list(grafo.nodes())[0]]
    nao_visitados = set(grafo.nodes())
    nao_visitados.remove(caminho[0])

    while nao_visitados:
        vertice_atual = caminho[-1]
        vizinhos = list(grafo.neighbors(vertice_atual))
        vizinhos_nao_visitados = list(filter(lambda x: x in nao_visitados, vizinhos))

        if not vizinhos_nao_visitados:
            break

        # Escolhe o vizinho mais próximo
        proximo_vertice = min(vizinhos_nao_visitados, key=lambda x: grafo[vertice_atual][x]['weight'])

        caminho.append(proximo_vertice)
        nao_visitados.remove(proximo_vertice)

    # Adiciona o vértice de origem ao final do caminho para formar um ciclo
    caminho.append(caminho[0])

    # Calcula o comprimento total do caminho
    comprimento_total = sum(grafo[caminho[i]][caminho[i+1]]['weight'] for i in range(len(caminho)-1))

    return caminho, comprimento_total

def main():
    selecao = int(input("Escolha uma opção (1 - Cobertura Mínima de Vértices, 2 - Caixeiro Viajante): "))

    if selecao == 1:
        # Solicita ao usuário o número de vértices
        num_vertices = int(input("Digite o número de vértices: "))
        arestas = []

        # Solicita ao usuário informações sobre as arestas
        for _ in range(num_vertices):
            origem = int(input("Digite o vértice de origem da aresta: "))
            destino = int(input("Digite o vértice de destino da aresta: "))
            arestas.append((origem, destino))

        # Cria um grafo não direcionado
        grafo = nx.Graph()
        grafo.add_edges_from(arestas)

        # Chama a função de cobertura mínima de vértices
        vertices_selecionados = cobertura_minima_vertices(grafo)

        # Exibe o resultado
        print("\nVértices Selecionados para Cobertura Mínima:")
        print(vertices_selecionados)

    elif selecao == 2:
        # Solicita ao usuário o número de vértices
        num_vertices = int(input("Digite o número de vértices: "))
        arestas = []

        # Solicita ao usuário informações sobre as arestas
        for _ in range(num_vertices - 1):
            origem = int(input("Digite o vértice de origem da aresta: "))
            destino = int(input("Digite o vértice de destino da aresta: "))
            peso = float(input("Digite o peso da aresta: "))
            arestas.append((origem, destino, peso))

        # Cria um grafo direcionado ponderado
        grafo = nx.Graph()
        grafo.add_weighted_edges_from(arestas)

        # Chama a função do Caixeiro Viajante
        caminho_aproximado, comprimento = caixeiro_viajante_vizinho_mais_proximo(grafo)

        # Exibe o resultado
        print("\nCaminho Aproximado:")
        print(caminho_aproximado)
        print(f"Comprimento Total: {comprimento}")

    else:
        print("Opção inválida.")

if __name__ == "__main__":
    main()
