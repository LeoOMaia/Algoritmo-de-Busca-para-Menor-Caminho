#include <iostream>
#include <fstream>
#include <stack>
#include <vector>
#include <tuple>
#include <queue>
#include <cmath>
#include <iomanip>
#include <chrono>
#include <filesystem>

// retorna matriz do mapa
std::pair<std::pair<int, int>, std::vector<std::vector<char>>> read_map(const std::string &file_path) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        throw std::runtime_error("Erro ao abrir o arquivo de mapa.");
    }

    // Ler a primeira linha para o tamanho do mapa
    std::string size_line;
    if (!std::getline(file, size_line)) {
        throw std::runtime_error("O arquivo está vazio ou não contém informações suficientes.");
    }

    int rows, cols;
    if (sscanf(size_line.c_str(), "%d %d", &rows, &cols) != 2) {
        throw std::runtime_error("Formato inválido para a primeira linha do arquivo. Esperado: '<colunas> <linhas>'");
    }

    // Ler o restante do mapa
    std::vector<std::vector<char>> map;
    std::string line;
    while (std::getline(file, line)) {
        map.emplace_back(line.begin(), line.end());
    }

    return {{cols, rows}, map};
}

// Define o custo de cada tipo de terreno
double get_cost(char terrain) {
    switch (terrain) {
        case '.': return 1.0; // Grama
        case ';': return 1.5; // Grama Alta
        case '+': return 2.5; // Água
        default: return 6.0; // Fogo
    }
}

void bfs(const std::vector<std::vector<char>> &map, int &xi, int &yi, int &xf, int &yf, std::pair<int, int> &size, std::ofstream &outfile){
    // Variaveis para Comparacao
    int expanded = 0;
    auto start_time = std::chrono::high_resolution_clock::now(); // Inicia o temporizador

    // Movimento possível: cima, baixo, esquerda, direita
    const std::vector<std::pair<int, int>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

    std::vector<std::vector<double>> cost(size.first, std::vector<double>(size.second, std::numeric_limits<double>::infinity()));
    std::vector<std::vector<bool>> visited(size.first, std::vector<bool>(size.second, false));
    std::vector<std::vector<std::pair<int, int>>> parent(size.first, std::vector<std::pair<int, int>>(size.second, {-1, -1}));

    // Fila para o BFS: armazena (x, y) e o custo atual
    std::queue<std::tuple<int, int, double>> queue;
    queue.push({xi, yi, 0});
    cost[xi][yi] = 0.0;
    visited[xi][yi] = true;

    while (!queue.empty()) {
        auto [x, y, current_cost] = queue.front();
        queue.pop();

        // Se chegarmos ao destino, saímos do loop
        if (x == xf && y == yf) {
            break;
        }

        // Contabilizando que expandiu no
        expanded++;

        // Explorar as direções possíveis
        for (const auto &[dx, dy] : directions) {
            int nx = x + dx;
            int ny = y + dy;

            // Verificar se a nova posição está dentro dos limites e não é um obstáculo
            if (nx >= 0 && ny >= 0 && nx < size.first && ny < size.second && map[nx][ny] != '@') {

                // Atualizar o custo e o pai se um caminho melhor for encontrado
                if (!visited[nx][ny]) {
                    visited[nx][ny] = true;
                    cost[nx][ny] = current_cost + get_cost(map[nx][ny]);
                    parent[nx][ny] = {x, y};
                    queue.push({nx, ny, cost[nx][ny]});
                }
            }
        }

        // Registrar tempo de execução
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end_time - start_time;
        // Gravar o número de estados expandidos e o tempo de execução no arquivo CSV
        outfile << expanded << "," << duration.count() << "\n";

        if (visited[xf][yf]){
            break;
        }
    }

    // Rastrear o caminho do destino até a origem (se encontrado)
    if (visited[xf][yf]) {
        // Imprimir custo
        std::cout << std::fixed << std::setprecision(1);
        std::cout << cost[xf][yf];
        std::cout.unsetf(std::ios::fixed); // Remove o formato fixo

        // Reconstrução do caminho do destino até a origem
        std::vector<std::pair<int, int>> path;
        for (int cx = xf, cy = yf; cx != -1 && cy != -1; std::tie(cx, cy) = parent[cx][cy]) {
            path.push_back({cx, cy});
        }

        // Imprimir o caminho do início ao fim (sem a necessidade de inverter)
        for (int i = path.size() - 1; i >= 0; --i) {
            std::cout << " (" << path[i].second << "," << path[i].first << ")";
        }
        std::cout << std::endl;
    }
    else {
        std::cout << "Nenhum caminho encontrado." << std::endl;
    }

    // Adiciona custo encontrado para chegar no destino
    outfile << cost[xf][yf];
}

void ids(const std::vector<std::vector<char>> &map, int &xi, int &yi, int &xf, int &yf, std::pair<int, int> &size, std::ofstream &outfile){
    // Variaveis para Comparacao
    int expanded = 0;
    auto start_time = std::chrono::high_resolution_clock::now(); // Inicia o temporizador
    
    // Movimentos possíveis: cima, baixo, esquerda, direita
    const std::vector<std::pair<int, int>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}}; 

    int depth_limit = 0; // Iniciar com o limite de profundidade 0
    while (true) {
        // Redefinir as matrizes de custo, visitados e de pais para os valores iniciais
        std::vector<std::vector<double>> cost(size.first, std::vector<double>(size.second, std::numeric_limits<double>::infinity()));
        std::vector<std::vector<bool>> visited(size.first, std::vector<bool>(size.second, false));
        std::vector<std::vector<std::pair<int, int>>> parent(size.first, std::vector<std::pair<int, int>>(size.second, {-1, -1}));

        std::stack<std::tuple<int, int, int, double>> stack; // (x, y, profundidade, custo)
        stack.push({xi, yi, 0, 0.0}); // Iniciar a pilha com o ponto inicial
        cost[xi][yi] = 0.0;
        visited[xi][yi] = true;

        while (!stack.empty()) {
            auto [x, y, depth, current_cost] = stack.top();
            stack.pop();

            // Se ultrapassar o limite de profundidade, não explore mais
            if (depth > depth_limit)
                continue;

            // Se chegarmos ao destino, saímos do loop
            if (x == xf && y == yf) {
                break;
            }

            // Contabilizando que expandiu no
            expanded++;

            // Explorar as direções possíveis
            for (const auto &[dx, dy] : directions) {
                int nx = x + dx;
                int ny = y + dy;

                // Verificar se a nova posição está dentro dos limites e não é um obstáculo
                if (nx >= 0 && ny >= 0 && nx < size.first && ny < size.second && map[nx][ny] != '@') {
                    // Adicionar visinho nao visitado 
                    if (!visited[nx][ny]) {
                        visited[nx][ny] = true;
                        cost[nx][ny] = current_cost + get_cost(map[nx][ny]);
                        parent[nx][ny] = {x, y};
                        stack.push({nx, ny, depth + 1, cost[nx][ny]});
                    }
                }
            }

            // Registrar tempo de execução
            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> duration = end_time - start_time;
            // Gravar o número de estados expandidos e o tempo de execução no arquivo CSV
            outfile << expanded << "," << duration.count() << "\n";

            if (visited[xf][yf]){
                break;
            }
        }

        // Rastrear o caminho do destino até a origem (se encontrado)
        if (visited[xf][yf]) {
            // Imprimir custo
            std::cout << std::fixed << std::setprecision(1);
            std::cout << cost[xf][yf];
            std::cout.unsetf(std::ios::fixed); // Remove o formato fixo

            // Reconstrução do caminho do destino até a origem
            std::vector<std::pair<int, int>> path;
            for (int cx = xf, cy = yf; cx != -1 && cy != -1; std::tie(cx, cy) = parent[cx][cy]) {
                path.push_back({cx, cy});
            }

            // Imprimir o caminho do início ao fim (sem a necessidade de inverter)
            for (int i = path.size() - 1; i >= 0; --i) {
                std::cout << " (" << path[i].second << "," << path[i].first << ")";
            }
            std::cout << std::endl;
            
            // Adiciona custo encontrado para chegar no destino
            outfile << cost[xf][yf];
            return;
        }

        // Se não encontrou, aumenta o limite de profundidade
        depth_limit++; 
    }
    std::cout << "Nenhum caminho encontrado." << std::endl;
}

void ucs(const std::vector<std::vector<char>> &map, int &xi, int &yi, int &xf, int &yf, std::pair<int, int> &size, std::ofstream &outfile) {
    // Variaveis para Comparacao
    int expanded = 0;
    auto start_time = std::chrono::high_resolution_clock::now(); // Inicia o temporizador

    // Movimento possível: cima, baixo, esquerda, direita
    const std::vector<std::pair<int, int>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

    std::vector<std::vector<double>> cost(size.first, std::vector<double>(size.second, std::numeric_limits<double>::infinity()));
    std::vector<std::vector<bool>> visited(size.first, std::vector<bool>(size.second, false));
    std::vector<std::vector<std::pair<int, int>>> parent(size.first, std::vector<std::pair<int, int>>(size.second, {-1, -1}));

    // Fila de prioridade para UCS: armazena (custo acumulado, x, y)
    using State = std::tuple<double, int, int>;
    std::priority_queue<State, std::vector<State>, std::greater<State>> queue;

    // Inicializar a busca
    cost[xi][yi] = 0.0;
    queue.push({cost[xi][yi], xi, yi});

    while (!queue.empty()) {
        auto [current_cost, x, y] = queue.top();
        queue.pop();

        // Se já visitamos esse nó, ignoramos
        if (visited[x][y])
            continue;
        visited[x][y] = true;

        // Verificar se chegamos ao destino
        if (x == xf && y == yf) {
            break;
        }

        // Contabilizando que expandiu no
        expanded++;

        // Explorar vizinhos
        for (const auto &[dx, dy] : directions) {
            int nx = x + dx;
            int ny = y + dy;

            // Verificar limites e se a célula não é um obstáculo
            if (nx >= 0 && ny >= 0 && nx < size.first && ny < size.second && map[nx][ny] != '@') {
                double move_cost = get_cost(map[nx][ny]);
                double new_cost = current_cost + move_cost;

                // Atualizar se encontrarmos um caminho mais barato
                if (new_cost < cost[nx][ny]) {
                    cost[nx][ny] = new_cost;
                    parent[nx][ny] = {x, y};
                    queue.push({new_cost, nx, ny});
                }
            }
        }

        // Registrar tempo de execução
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end_time - start_time;
        // Gravar o número de estados expandidos e o tempo de execução no arquivo CSV
        outfile << expanded << "," << duration.count() << "\n";
    }

    // Rastrear o caminho e imprimir o resultado
    if (visited[xf][yf]) {
        // Imprimir custo total
        std::cout << std::fixed << std::setprecision(1);
        std::cout << cost[xf][yf];
        std::cout.unsetf(std::ios::fixed); // Remove o formato fixo
    
        // Reconstrução do caminho do destino até a origem
        std::vector<std::pair<int, int>> path;
        for (int cx = xf, cy = yf; cx != -1 && cy != -1; std::tie(cx, cy) = parent[cx][cy]) {
            path.push_back({cx, cy});
        }

        // Imprimir o caminho do início ao fim
        for (int i = path.size() - 1; i >= 0; --i) {
            std::cout << " (" << path[i].second << "," << path[i].first << ")";
        }
        std::cout << std::endl;
    }
    else {
        std::cout << "Nenhum caminho encontrado." << std::endl;
    }

    // Adiciona custo encontrado para chegar no destino
    outfile << cost[xf][yf];
}

double euclidean_distance(int x1, int y1, int x2, int y2) {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

void greedy(const std::vector<std::vector<char>> &map, int &xi, int &yi, int &xf, int &yf, std::pair<int, int> &size, std::ofstream &outfile) {
    // Variaveis para Comparacao
    int expanded = 0;
    auto start_time = std::chrono::high_resolution_clock::now(); // Inicia o temporizador

    // Movimento possível: cima, baixo, esquerda, direita
    const std::vector<std::pair<int, int>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

    std::vector<std::vector<bool>> visited(size.first, std::vector<bool>(size.second, false));
    std::vector<std::vector<std::pair<int, int>>> parent(size.first, std::vector<std::pair<int, int>>(size.second, {-1, -1}));
    std::vector<std::vector<double>> cost(size.first, std::vector<double>(size.second, std::numeric_limits<double>::infinity()));
    
    // Fila de prioridade para Greedy: armazena (custo distancia euclidiana, x, y)
    using State = std::tuple<double, int, int>;
    std::priority_queue<State, std::vector<State>, std::greater<State>> queue;

    // Início da busca
    visited[xi][yi] = true;
    cost[xi][yi] = 0.0;
    queue.push({euclidean_distance(xi, yi, xf, yf), xi, yi});

    while (!queue.empty()) {
        auto [heuristic, x, y] = queue.top();
        queue.pop();

        // Verificar se chegou ao destino
        if (x == xf && y == yf) {
            break;
        }

        // Contabilizando que expandiu no
        expanded++;

        // Explorar vizinhos
        for (const auto &[dx, dy] : directions) {
            int nx = x + dx;
            int ny = y + dy;

            // Verificar se está dentro dos limites e não é obstáculo
            if (nx >= 0 && ny >= 0 && nx < size.first && ny < size.second && map[nx][ny] != '@') {
                double move_cost = get_cost(map[nx][ny]);
                double new_cost = cost[x][y] + move_cost;

                if (!visited[nx][ny] || new_cost < cost[nx][ny]) {
                    visited[nx][ny] = true;
                    cost[nx][ny] = new_cost;
                    parent[nx][ny] = {x, y};
                    queue.push({euclidean_distance(nx, ny, xf, yf), nx, ny});
                }
            }
        }

        // Registrar tempo de execução
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end_time - start_time;
        // Gravar o número de estados expandidos e o tempo de execução no arquivo CSV
        outfile << expanded << "," << duration.count() << "\n";
    }

    // Reconstruir o caminho se possível
    if (visited[xf][yf]) {
        // Imprimir custo total
        std::cout << std::fixed << std::setprecision(1);
        std::cout << cost[xf][yf];
        std::cout.unsetf(std::ios::fixed); // Remove o formato fixo

        std::vector<std::pair<int, int>> path;
        for (int cx = xf, cy = yf; cx != -1 && cy != -1; std::tie(cx, cy) = parent[cx][cy]) {
            path.push_back({cx, cy});
        }

        // Imprimir o caminho do início ao fim
        for (int i = path.size() - 1; i >= 0; --i) {
            std::cout << " (" << path[i].second << "," << path[i].first << ")";
        }
        std::cout << std::endl;
    }
    else {
        std::cout << "Nenhum caminho encontrado." << std::endl;
    }

    // Adiciona custo encontrado para chegar no destino
    outfile << cost[xf][yf];
}

void astar(const std::vector<std::vector<char>> &map, int &xi, int &yi, int &xf, int &yf, std::pair<int, int> &size, std::ofstream &outfile) {
    // Variaveis para Comparacao
    int expanded = 0;
    auto start_time = std::chrono::high_resolution_clock::now(); // Inicia o temporizador

    // Movimento possível: cima, baixo, esquerda, direita
    const std::vector<std::pair<int, int>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

    std::vector<std::vector<bool>> visited(size.first, std::vector<bool>(size.second, false));
    std::vector<std::vector<std::pair<int, int>>> parent(size.first, std::vector<std::pair<int, int>>(size.second, {-1, -1}));
    std::vector<std::vector<double>> cost(size.first, std::vector<double>(size.second, std::numeric_limits<double>::infinity()));
    
    // Fila de prioridade para UCS: armazena (custo acumulado + distancia euclidiana, x, y)
    using State = std::tuple<double, int, int>;
    std::priority_queue<State, std::vector<State>, std::greater<State>> queue;

    // Início da busca
    visited[xi][yi] = true;
    cost[xi][yi] = 0.0;
    queue.push({euclidean_distance(xi, yi, xf, yf) + cost[xi][yi], xi, yi});

    while (!queue.empty()) {
        auto [heuristic, x, y] = queue.top();
        queue.pop();

        // Verificar se chegou ao destino
        if (x == xf && y == yf) {
            break;
        }

        // Contabilizando que expandiu no
        expanded++;

        // Explorar vizinhos
        for (const auto &[dx, dy] : directions) {
            int nx = x + dx;
            int ny = y + dy;

            // Verificar se está dentro dos limites e não é obstáculo
            if (nx >= 0 && ny >= 0 && nx < size.first && ny < size.second && map[nx][ny] != '@') {
                double move_cost = get_cost(map[nx][ny]);
                double new_cost = cost[x][y] + move_cost;

                if (!visited[nx][ny] || new_cost < cost[nx][ny]) {
                    visited[nx][ny] = true;
                    cost[nx][ny] = new_cost;
                    parent[nx][ny] = {x, y};
                    queue.push({euclidean_distance(nx, ny, xf, yf) + cost[nx][ny], nx, ny});
                }
            }
        }

        // Registrar tempo de execução
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end_time - start_time;
        // Gravar o número de estados expandidos e o tempo de execução no arquivo CSV
        outfile << expanded << "," << duration.count() << "\n";
    }

    // Reconstruir o caminho se possível
    if (visited[xf][yf]) {
        // Imprimir custo total
        std::cout << std::fixed << std::setprecision(1);
        std::cout << cost[xf][yf];
        std::cout.unsetf(std::ios::fixed); // Remove o formato fixo

        std::vector<std::pair<int, int>> path;
        for (int cx = xf, cy = yf; cx != -1 && cy != -1; std::tie(cx, cy) = parent[cx][cy]) {
            path.push_back({cx, cy});
        }

        // Imprimir o caminho do início ao fim
        for (int i = path.size() - 1; i >= 0; --i) {
            std::cout << " (" << path[i].second << "," << path[i].first << ")";
        }
        std::cout << std::endl;
    }
    else {
        std::cout << "Nenhum caminho encontrado." << std::endl;
    }

    // Adiciona custo encontrado para chegar no destino
    outfile << cost[xf][yf];
}

int main(int argc, char *argv[]) {
    if (argc != 7) {
        std::cerr << "Uso: ./pathfinder [caminho para arquivo mapa] [identificador metodo] xi yi xf yf\n";
        return 1;
    }

    // Lendo parametros
    std::string map_path = argv[1];
    std::string method_id = argv[2];
    int yi = std::stoi(argv[3]);
    int xi = std::stoi(argv[4]);
    int yf = std::stoi(argv[5]);
    int xf = std::stoi(argv[6]);
    auto [size, map] = read_map(map_path);

    // Verifica se tem algum parametro errado
    if (xi < 0 || xi >= size.first || yi < 0 || yi >= size.second || xf < 0 || xf >= size.first || yf < 0 || yf >= size.second) {
        throw std::invalid_argument("Pontos iniciais ou finais fora do mapa.");
    }
    if (get_cost(map[xi][yi]) == std::numeric_limits<double>::infinity() || 
        get_cost(map[xf][yf]) == std::numeric_limits<double>::infinity()) {
        throw std::invalid_argument("Pontos iniciais ou finais são intransponíveis.");
    }

    // Criar o diretório "csv_files" se não existir
    std::string directory = "./csv_files";
    if (!std::filesystem::exists(directory)) {
        if (!std::filesystem::create_directory(directory)) {
            std::cerr << "Erro ao criar o diretório: " << directory << std::endl;
            return 1; // Retorna um erro se a criação falhar
        }
    }

    // Criar arquivo CSV para salvar os resultados
    std::ostringstream oss;
    oss << "./csv_files/" << method_id << "_(" << xi << "," << yi << ")-(" << xf << "," << yf << ").csv";
    std::string name_file = oss.str();

    std::ofstream outfile(name_file);
    outfile << "Nós Expandidos, Tempo de Execução (segundos)\n";

    if(method_id == "BFS"){
        bfs(map, xi, yi, xf, yf, size, outfile);
    }
    else if(method_id == "IDS"){
        ids(map, xi, yi, xf, yf, size, outfile);
    }
    else if(method_id == "UCS"){
        ucs(map, xi, yi, xf, yf, size, outfile);
    }
    else if(method_id == "Greedy"){
        greedy(map, xi, yi, xf, yf, size, outfile);
    }
    else if(method_id == "Astar"){
        astar(map, xi, yi, xf, yf, size, outfile);
    }
    else{
        std::cerr << "Metodo Invalido, use os metodos: BFS, IDS, UCS, Greedy ou Astar";
    }

    // Fechar o arquivo CSV
    outfile.close();
}