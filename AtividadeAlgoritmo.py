#CÓDIGO PRINCIPAL

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

#BUSCA PRO PROFUNDIDADE --> PRECISA SER REVISADO
if selecao == 1:
  print('teste')

#HAMILTONIANOS --> PRECISA SER REVISADO
if selecao ==2:
  print('teste')

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

#ALGORITIMOS GULOSOS
if selecao == 5:

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
if __name__ == "__main__":
    # Chama a função principal
    main()

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
