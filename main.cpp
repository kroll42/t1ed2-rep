/***************************************************************
* Alunos: Gabriel Silva e Karolyne Muniz
* Curso:Bacharelado em Ciências da Computação
*
* Trabalho 1: Grafos e Aplicações *
* *
* Estrutura de Dados II -- 2024 -- DACC/UNIR, turma 2 -- Prof.Carolina Watanabe *
* Compilador: DEV-CPP ISO C99 versão 5.11 *
* Sistema Operacional: Windows 10  *
***************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

#define MAX_VERTICES 100

typedef struct {
    int matriz[MAX_VERTICES][MAX_VERTICES];
    int numero_vertices;
} Grafo;

void inicializar_grafo(Grafo *grafo, int numero_vertices) {
    grafo->numero_vertices = numero_vertices;
    for (int i = 0; i < numero_vertices; i++) {
        for (int j = 0; j < numero_vertices; j++) {
            grafo->matriz[i][j] = 0;
        }
    }
}

void carregar_grafo(Grafo *grafo, const char *nome_arquivo) {
    FILE *arquivo = fopen(nome_arquivo, "r");
    if (arquivo == NULL) {
        printf("Erro ao abrir o arquivo.\n");
        return;
    }

    int numero_vertices;
    fscanf(arquivo, "%d", &numero_vertices);
    inicializar_grafo(grafo, numero_vertices);

    int vertice_origem, vertice_destino, peso;
    while (fscanf(arquivo, "%d %d %d", &vertice_origem, &vertice_destino, &peso) != EOF) {
        grafo->matriz[vertice_origem][vertice_destino] = peso;
        grafo->matriz[vertice_destino][vertice_origem] = peso;  // Se for um grafo nÃ£o direcionado
    }

    fclose(arquivo);
}

void imprimir_grafo(Grafo *grafo) {
    for (int i = 0; i < grafo->numero_vertices; i++) {
        for (int j = 0; j < grafo->numero_vertices; j++) {
            printf("%d ", grafo->matriz[i][j]);
        }
        printf("\n");
    }
}

void busca_em_largura(Grafo *grafo, int vertice_inicial) {
    int fila[MAX_VERTICES], visitado[MAX_VERTICES], inicio = 0, fim = 0;
    for (int i = 0; i < grafo->numero_vertices; i++) {
        visitado[i] = 0;
    }

    visitado[vertice_inicial] = 1;
    fila[fim++] = vertice_inicial;

    while (inicio != fim) {
        int vertice_atual = fila[inicio++];
        printf("Visitando o vertice %d\n", vertice_atual);

        for (int i = 0; i < grafo->numero_vertices; i++) {
            if (grafo->matriz[vertice_atual][i] != 0 && !visitado[i]) {
                fila[fim++] = i;
                visitado[i] = 1;
            }
        }
    }
}

void busca_em_profundidade(Grafo *grafo, int vertice_atual, int *visitado) {
    printf("Visitando o vertice %d\n", vertice_atual);
    visitado[vertice_atual] = 1;

    for (int i = 0; i < grafo->numero_vertices; i++) {
        if (grafo->matriz[vertice_atual][i] != 0 && !visitado[i]) {
            busca_em_profundidade(grafo, i, visitado);
        }
    }
}

void iniciar_dfs(Grafo *grafo, int vertice_inicial) {
    int visitado[MAX_VERTICES];
    for (int i = 0; i < grafo->numero_vertices; i++) {
        visitado[i] = 0;
    }

    busca_em_profundidade(grafo, vertice_inicial, visitado);
}

int encontrar_minimo(int chave[], int incluido[], int numero_vertices) {
    int min = INT_MAX, min_index;

    for (int v = 0; v < numero_vertices; v++) {
        if (incluido[v] == 0 && chave[v] < min) {
            min = chave[v], min_index = v;
        }
    }

    return min_index;
}

void arvore_geradora_minima_prim(Grafo *grafo) {
    int chave[MAX_VERTICES];
    int incluido[MAX_VERTICES];
    int pai[MAX_VERTICES];

    for (int i = 0; i < grafo->numero_vertices; i++) {
        chave[i] = INT_MAX;
        incluido[i] = 0;
    }

    chave[0] = 0;
    pai[0] = -1;  // Primeiro nao e a raiz da MST

    for (int count = 0; count < grafo->numero_vertices - 1; count++) {
        int u = encontrar_minimo(chave, incluido, grafo->numero_vertices);
        incluido[u] = 1;

        for (int v = 0; v < grafo->numero_vertices; v++) {
            if (grafo->matriz[u][v] && incluido[v] == 0 && grafo->matriz[u][v] < chave[v]) {
                pai[v] = u, chave[v] = grafo->matriz[u][v];
            }
        }
    }

    printf("Arestas na Arvore Geradora Minima (MST):\n");
    for (int i = 1; i < grafo->numero_vertices; i++) {
        printf("%d - %d com peso %d\n", pai[i], i, grafo->matriz[i][pai[i]]);
    }
}

int encontrar_menor_distancia(int distancia[], int visitado[], int numero_vertices) {
    int min = INT_MAX, min_index;

    for (int v = 0; v < numero_vertices; v++) {
        if (visitado[v] == 0 && distancia[v] <= min) {
            min = distancia[v], min_index = v;
        }
    }

    return min_index;
}

void caminhos_minimos_dijkstra(Grafo *grafo, int vertice_origem) {
    int distancia[MAX_VERTICES];
    int visitado[MAX_VERTICES];

    for (int i = 0; i < grafo->numero_vertices; i++) {
        distancia[i] = INT_MAX;
        visitado[i] = 0;
    }

    distancia[vertice_origem] = 0;

    for (int count = 0; count < grafo->numero_vertices - 1; count++) {
        int u = encontrar_menor_distancia(distancia, visitado, grafo->numero_vertices);
        visitado[u] = 1;

        for (int v = 0; v < grafo->numero_vertices; v++) {
            if (!visitado[v] && grafo->matriz[u][v] && distancia[u] != INT_MAX && distancia[u] + grafo->matriz[u][v] < distancia[v]) {
                distancia[v] = distancia[u] + grafo->matriz[u][v];
            }
        }
    }

    printf("Vertice \t Distancia da Origem\n");
    for (int i = 0; i < grafo->numero_vertices; i++) {
        printf("%d \t\t %d\n", i, distancia[i]);
    }
}

int main() {
    Grafo grafo;
    carregar_grafo(&grafo, "grafo.txt");

    printf("Grafo carregado:\n");
    imprimir_grafo(&grafo);

    printf("\nBusca em Largura (BFS) a partir do vertice 0:\n");
    busca_em_largura(&grafo, 0);

    printf("\nBusca em Profundidade (DFS) a partir do vertice 0:\n");
    iniciar_dfs(&grafo, 0);

    printf("\nArvore Geradora Minima (Prim):\n");
    arvore_geradora_minima_prim(&grafo);

    printf("\nCaminhos Minimos (Dijkstra) a partir do vertice 0:\n");
    caminhos_minimos_dijkstra(&grafo, 0);
    
    printf("\nPressione Enter para sair...");
    getchar(); 
    return 0;
}

