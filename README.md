## Descrição

Este projeto implementa algoritmos para encontrar caminhos em mapas com obstáculos.

## Bibliotecas

Este projeto utiliza apenas bibliotecas padrão da linguagem C++:

- `<iostream>`: Para entrada e saída de dados.
- `<fstream>`: Para leitura de arquivos.
- `<stack>`: Para gerenciar pilhas em IDS.
- `<vector>`: Para manipulação de matrizes e listas.
- `<tuple>`: Para armazenar coordenadas e custos em uma única estrutura.
- `<queue>`: Para gerenciar a fila de prioridade.
- `<cmath>`: Para calcular a distância euclidiana e definir valores infinitos (`std::numeric_limits<double>::infinity()`).
- `<iomanip>`: Para formatação de saída (e.g., precisão decimal).
- `<chrono>` : Para cronometrar o tempo.
- `<filesystem>` : Para criar diretorio `csv_file` se nao existir.

## Ambiente de Desenvolvimento

Este projeto foi desenvolvido e testado utilizando o compilador **g++ versão 11.4.0** em uma máquina com o sistema operacional **Ubuntu 22.04**.

## Compilação

Compile o programa usando o comando abaixo:

```bash
g++ -Wall pathfinder.cpp -o pathfinder
```

## Execução

Execute o programa com o seguinte formato:

```bash
./pathfinder [caminho para arquivo mapa] [identificador método] yi xi yf xf
```
O arquivo do mapa contera na linha 1 as dimensoes da matriz, seguido nas proximas linhas pela matriz.

## Makefile

Pode ser utilizado os comando `build` e `clean`, para gerar e limpar executaveis:

```bash
make build
make clean
```

## Plotar graficos

Apos executar o compilado, sera gerado arquivos no formato `.csv` no diretorio `csv_file` da raiz do projeto (sera criado automatico se nao existir) contendo informacoes sobre quantidade de nos expandidos com seu tempo necessario e o custo total pelo algoritmo que foi necessario para chegar ao destino.
Podera entao executar o script python `graphic.py` que ira gerar o grafico para visualizar de todos os arquivos `.csv` presentes no diretorio.
Pode ser passado a flag `--step` que salta no arquivo a quantidade de parametros que sao lidos para gerar o grafico (padrao step=1).

```bash
python3 graphic.py --step 100
```

## Parâmetros

- [caminho para arquivo mapa]: Caminho para o arquivo que contém o mapa.
- [identificador método]: Identificador do algoritmo de busca (ver tabela abaixo).
- yi e xi: Coordenadas iniciais no mapa (coluna e linha).
- yf e xf: Coordenadas finais no mapa (coluna e linha).

## Propriedades do Mapa

|   Terreno   |   Simbolo   |    Custo    |
| ----------- | ----------- | ----------- |
| Grama       | .           |1.0          |
|Grama Alta   | ;           |1.5          |
|Agua         |+            |2.5          |
|Fogo         |x            |6.0          |
|Parede       |@            |∞            |


## Métodos Disponíveis

|Identificador|  Algoritmo  |
| ----------- | ----------- |
|BFS          | Busca em Largura|
|IDS          | Busca Iterativa em Profundidade|
|UCS          | Busca de Custo Uniforme|
|Greedy       | Busca Gulosa|
|Astar        | Busca A*|