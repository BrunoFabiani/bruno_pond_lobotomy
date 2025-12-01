#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/msg/robot_sensors.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"

#include <vector>
#include <queue>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <unordered_set>

struct Pos { 
    int x, y; 
    bool operator==(const Pos& o) const { return x == o.x && y == o.y; } // operador de comparação para checar se duas posições são iguais
};

// Hash para usar Pos em unordered_set (evita processar células repetidas)
struct PosHash {
    size_t operator()(const Pos& p) const {
        return std::hash<int>()(p.x) ^ (std::hash<int>()(p.y) << 1); // combina hash de x e y
    }
};

class Pathfinder : public rclcpp::Node {
public:
    Pathfinder() : Node("pathfinder") {
        sensor_sub_ = create_subscription<cg_interfaces::msg::RobotSensors>(
            "/culling_games/robot_sensors", 1,
            std::bind(&Pathfinder::on_sensor, this, std::placeholders::_1) // manda a mensagem do sensor_sub pro on_sensor usar, o this indica qual objeto usar da classe pathfinder
        ); // create_subscription recebe estes 3 argumentos

        move_client_ = create_client<cg_interfaces::srv::MoveCmd>("move_command"); // faz o pathfinder virar cliente do move_command

        internal_map.assign(MAP_SIZE, std::vector<char>(MAP_SIZE, '?')); // cria uma matriz com tamanho do MAP_SIZE x MAP_SIZE com ?. usa biblioteca vector
        robot = {MAP_SIZE/2, MAP_SIZE/2}; // põe o robô no meio do mapa
        internal_map[robot.x][robot.y] = 'f'; // marca o lugar do robô como f (livre)
        start_pos = robot; // guarda posição inicial
        
        RCLCPP_INFO(get_logger(), "=== PARTE 2: MAPEAMENTO COMPLETO DO LABIRINTO ===");
    }

private: // callbacks e chamadas tem que serem feitas estritamente dentro da classe
    // atributos da classe pathfinder
    static constexpr int MAP_SIZE = 200; // constexpr é boa prática de programação

    std::vector<std::vector<char>> internal_map; // como vector muda em tempo de execução posso deixar assim
    Pos robot; // robô é do tipo que tem coordenadas
    Pos start_pos; // guarda onde o robô começou
    Pos finish_pos; // guarda onde está o objetivo
    std::vector<Pos> path; // caminho que o robô irá seguir
    size_t step_idx = 0; // índice do passo atual no caminho
    bool finish_found = false; // flag pra indicar se encontrou o finish
    bool mapping_complete = false; // flag pra indicar se terminou de mapear tudo
    bool path_being_executed = false; // flag pra indicar se está seguindo um caminho

    rclcpp::Subscription<cg_interfaces::msg::RobotSensors>::SharedPtr sensor_sub_; // criando o subscription, pegando da biblio do ros, configurado para lidar com mensagem do tipo RobotSensor
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_client_; // mesma lógica que o do serviço

    // ========== SENSOR CALLBACK toca a operação ==========
    void on_sensor(const cg_interfaces::msg::RobotSensors::SharedPtr msg) {
        update_map(msg); // atualiza o mapa com os dados do sensor

        // Se já mapeou tudo, não faz nada
        if (mapping_complete) return;

        // Se está executando um caminho, continua movendo
        if (path_being_executed && step_idx < path.size()) {
            move_one_step();
            return;
        }

        // Procura finish se ainda não encontrou
        if (!finish_found) {
            Pos finish = find_finish();
            if (finish.x != -1) { // se encontrou (coordenadas válidas)
                finish_found = true;
                finish_pos = finish;
                RCLCPP_INFO(get_logger(), "Finish encontrado em (%d, %d)", finish.x, finish.y);
            }
        }

        // Continua mapeando mesmo depois de achar o finish (precisa explorar tudo)
        std::vector<Pos> frontier = find_frontier_cells(); // acha células que podem ser exploradas
        
        if (frontier.empty()) { // se não há mais fronteiras, mapeamento completo
            RCLCPP_INFO(get_logger(), "\n=== MAPEAMENTO COMPLETO! ===");
            mapping_complete = true;
            save_mapped_data(); // salva o mapa criado
            reproduce_parte1_algorithm(); // reproduz o algoritmo da parte 1
            return;
        }

        // Seleciona melhor fronteira e planeja caminho até ela
        Pos best = select_closest_frontier(frontier); // pega a fronteira mais próxima
        plan_path_dijkstra(robot, best); // calcula caminho usando Dijkstra
        if (!path.empty()) { // se achou caminho válido
            path_being_executed = true; // marca que está executando caminho
            step_idx = 0; // reseta índice
            move_one_step(); // começa a mover
        }
    }

    // ========== MAP UPDATE ==========
    void update_map(const cg_interfaces::msg::RobotSensors::SharedPtr msg) {
        auto mark = [&](int dx, int dy, const std::string &val) { // [&] indica que esta lambda captura todas as variáveis do escopo externo por referência
            char c = '?'; // ser uma lambda faz com que ele acesse o robot e internal_map sem problemas
            if (val == "b") c = 'b'; // bloqueado
            else if (val == "f") c = 'f'; // livre
            else if (val == "t") c = 't'; // target/finish
            int nx = robot.x + dx, ny = robot.y + dy; // calcula coordenadas do vizinho
            if (nx >= 0 && nx < MAP_SIZE && ny >= 0 && ny < MAP_SIZE) { // checa se está dentro dos limites
                internal_map[nx][ny] = c; // marca no mapa
            }
        };

        // marca no mapa todas as coisas que o sensor tá pegando e o robô pode ver
        mark(-1, 0, msg->up);
        mark(1, 0, msg->down);
        mark(0, -1, msg->left);
        mark(0, 1, msg->right);
        internal_map[robot.x][robot.y] = 'f'; // marca a posição do robô como livre, já que ele está nela
    }

    // ========== FIND FINISH ==========
    Pos find_finish() { // sempre checa se encontramos o final
        for (int i = 0; i < MAP_SIZE; i++)
            for (int j = 0; j < MAP_SIZE; j++)
                if (internal_map[i][j] == 't') // se achou o target
                    return {i, j};
        return {-1, -1}; // como vector não tem coordenada negativa, isto basicamente sinaliza que não encontramos o final
    }

    // ========== FRONTIER ==========
    bool has_unknown_neighbor(int x, int y) { // checa se tem vizinhos desconhecidos (fronteira)
        if (x > 0 && internal_map[x-1][y] == '?') return true; // checa cima
        if (x < MAP_SIZE-1 && internal_map[x+1][y] == '?') return true; // checa baixo
        if (y > 0 && internal_map[x][y-1] == '?') return true; // checa esquerda
        if (y < MAP_SIZE-1 && internal_map[x][y+1] == '?') return true; // checa direita
        return false; // se chegou aqui, não tem vizinho desconhecido
    }

    std::vector<Pos> find_frontier_cells() { // acha as células que podem ser exploradas
        std::vector<Pos> f;
        f.reserve(1000); // pre-aloca memória pra evitar realocações
        
        for (int i = 0; i < MAP_SIZE; i++)
            for (int j = 0; j < MAP_SIZE; j++)
                if (internal_map[i][j] == 'f' && has_unknown_neighbor(i, j)) // se é livre e tem vizinho desconhecido
                    f.push_back({i,j});
        
        return f; // depois de checar todas células do mapa, retorna f pras fronteiras
    }

    Pos select_closest_frontier(const std::vector<Pos> &f) { // checa os f's e pega o mais perto
        Pos best = f[0]; // começa com o primeiro
        int best_h = 1e9; // valor enorme pra garantir que qualquer distância seja menor
        for (auto &p : f) { // percorre cada célula dentro do vetor
            int h = std::abs(p.x - robot.x) + std::abs(p.y - robot.y); // distância Manhattan
            if (h < best_h) { best_h = h; best = p; } // se achou melhor, atualiza
        }
        return best; // retorna a fronteira mais próxima
    }

    // ========== DIJKSTRA  ==========
    void plan_path_dijkstra(const Pos &start, const Pos &goal) { // o mapa é constantemente re-atualizado, então ele está sempre calculando a melhor rota
        static std::vector<std::vector<int>> dist(MAP_SIZE, std::vector<int>(MAP_SIZE)); // guarda o custo mínimo conhecido 
        static std::vector<std::vector<Pos>> parent(MAP_SIZE, std::vector<Pos>(MAP_SIZE)); // guarda pai de cada célula pra reconstruir caminho
        
        const int INF = 1e9;
        for (int i = 0; i < MAP_SIZE; i++)
            for (int j = 0; j < MAP_SIZE; j++) {
                dist[i][j] = INF; // inicializa com infinito
                parent[i][j] = {-1, -1}; // sem pai no início
            }

        auto cmp = [&](Pos a, Pos b) { return dist[a.x][a.y] > dist[b.x][b.y]; }; // função de comparação para a fila de prioridade
        std::priority_queue<Pos, std::vector<Pos>, decltype(cmp)> pq(cmp); // cria a priority queue que vai guardar o menor custo

        dist[start.x][start.y] = 0; // distância do start é zero
        pq.push(start); // adiciona start na fila

        std::unordered_set<Pos, PosHash> visited; // evita processar células repetidas

        while (!pq.empty()) { // enquanto houver células pra explorar
            Pos u = pq.top(); pq.pop(); // pega célula com menor custo
            
            if (u.x == goal.x && u.y == goal.y) break; // se chegou no objetivo, para
            if (visited.count(u)) continue; // se já visitou, pula
            visited.insert(u); // marca como visitado

            const int dx[4] = {1, -1, 0, 0}; 
            const int dy[4] = {0, 0, 1, -1};
            
            for (int k = 0; k < 4; k++) { // percorre os 4 vizinhos ortogonais
                int nx = u.x + dx[k], ny = u.y + dy[k]; // calcula as coordenadas reais do vizinho
                
                if (nx < 0 || nx >= MAP_SIZE || ny < 0 || ny >= MAP_SIZE) continue; // checagem de borda
                if (internal_map[nx][ny] == 'b' || internal_map[nx][ny] == '?') continue; // não faz sentido ver vizinhos que não podem ser visitados
                if (visited.count({nx, ny})) continue; // se já visitou, pula

                int new_dist = dist[u.x][u.y] + 1; // como cada passo tem 1 de valor incrementamos 1 pro custo real acumulado
                if (new_dist < dist[nx][ny]) { // se achou caminho melhor
                    dist[nx][ny] = new_dist; // atualiza distância
                    parent[nx][ny] = u; // marca pai
                    pq.push({nx, ny}); // adiciona na fila
                }
            }
        }

        // Reconstrói caminho
        path.clear(); // para garantir que não tenha nenhum caminho antigo armazenado
        for (Pos p = goal; !(p.x == -1); p = parent[p.x][p.y]) { // com o p = parent ele vai pegando o pai de cada posição pra reconstruir o caminho
            path.push_back(p); // vai adicionando o elemento p no final do vetor path
            if (p == start) break; // se chegou no start, para
        }
        std::reverse(path.begin(), path.end()); // inverte pq isto vai pegando o pai de cada célula
        
        if (path.empty() || !(path[0] == start)) { // se caminho inválido
            path.clear(); // limpa
        }
    }

    // ========== MOVE ==========
    void move_one_step() { // lógica pra mover o robô
        if (path.empty() || step_idx + 1 >= path.size()) { // se não tem caminho ou chegou no final
            path_being_executed = false; // para de executar
            return;
        }

        Pos curr = path[step_idx], next = path[step_idx + 1]; // pega posição atual e próxima
        std::string d; // direção do movimento

        if (next.x == curr.x + 1) d = "down"; // se x aumenta, vai pra baixo
        else if (next.x == curr.x - 1) d = "up"; // se x diminui, vai pra cima
        else if (next.y == curr.y + 1) d = "right"; // se y aumenta, vai pra direita
        else if (next.y == curr.y - 1) d = "left"; // se y diminui, vai pra esquerda
        else { path_being_executed = false; return; } // movimento inválido

        auto req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>(); // cria requisição
        req->direction = d; // seta direção

        move_client_->async_send_request(
            req,
            [this, next](rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedFuture future) { // callback assíncrono
                auto res = future.get(); // pega resposta
                if (res->success) { // se movimento foi aceito
                    robot = next; // atualiza posição do robô
                    step_idx++; // avança pro próximo passo
                    
                    if (step_idx + 1 >= path.size()) { // se chegou no final do caminho
                        path_being_executed = false; // libera pra novo planejamento
                    }
                } else { // se movimento falhou
                    internal_map[next.x][next.y] = 'b'; // marca como obstáculo
                    path_being_executed = false; // força replanejamento
                }
            }
        );
    }

    // ========== SAVE MAP ==========
    void save_mapped_data() {
        std::ofstream file("mapa_mapeado.txt"); // abre arquivo pra salvar
        file << "=== MAPA CRIADO POR EXPLORAÇÃO (PARTE 2) ===\n";
        file << "Start: (" << start_pos.x << ", " << start_pos.y << ")\n";
        file << "Finish: (" << finish_pos.x << ", " << finish_pos.y << ")\n\n";

        for (int i = 0; i < MAP_SIZE; i++) { // percorre linhas
            for (int j = 0; j < MAP_SIZE; j++) { // percorre colunas
                file << internal_map[i][j]; // escreve cada célula
            }
            file << "\n"; // pula linha
        }
        file.close(); // fecha arquivo
        RCLCPP_INFO(get_logger(), "Mapa salvo em 'mapa_mapeado.txt'");
    }

    // ========== REPRODUZ ALGORITMO DA PARTE 1 ==========
    void reproduce_parte1_algorithm() {
        RCLCPP_INFO(get_logger(), "\n=== REPRODUZINDO ALGORITMO DA PARTE 1 ===");
        RCLCPP_INFO(get_logger(), "Convertendo mapa mapeado para formato da Parte 1...");
        
        // Converte internal_map para grid (formato da Parte 1: 0=livre, 1=bloqueado, 2=start, 3=goal)
        std::vector<std::vector<int>> grid(MAP_SIZE, std::vector<int>(MAP_SIZE));
        
        for (int i = 0; i < MAP_SIZE; i++) {
            for (int j = 0; j < MAP_SIZE; j++) {
                if (internal_map[i][j] == 'b' || internal_map[i][j] == '?') { // se é obstáculo ou desconhecido
                    grid[i][j] = 1; // marca como bloqueado
                } else if (internal_map[i][j] == 'f') { // se é livre
                    grid[i][j] = 0; // marca como livre
                } else if (internal_map[i][j] == 't') { // se é target
                    grid[i][j] = 3; // marca como goal
                }
            }
        }
        
        grid[start_pos.x][start_pos.y] = 2; // marca o start
        
        RCLCPP_INFO(get_logger(), "Executando Dijkstra da Parte 1...");
        
        // Roda Dijkstra (mesmo código da Parte 1)
        auto path_parte1 = run_dijkstra_parte1(grid, start_pos, finish_pos);
        
        if (path_parte1.empty()) { // se não achou caminho
            RCLCPP_ERROR(get_logger(), "FALHA: Não foi possível reproduzir rota!");
            return;
        }
        
        RCLCPP_INFO(get_logger(), "✅ SUCESSO! Rota reproduzida com sucesso!");
        RCLCPP_INFO(get_logger(), "Tamanho do caminho: %ld passos", path_parte1.size());
        RCLCPP_INFO(get_logger(), "Start(%d,%d) -> Finish(%d,%d)", 
                    start_pos.x, start_pos.y, finish_pos.x, finish_pos.y);
        
        // Salva o caminho
        std::ofstream pathfile("caminho_parte1_reproduzido.txt");
        pathfile << "=== CAMINHO REPRODUZIDO USANDO ALGORITMO DA PARTE 1 ===\n";
        pathfile << "Mapa usado: Mapa criado por exploração (Parte 2)\n";
        pathfile << "Algoritmo: Dijkstra (mesmo da Parte 1)\n\n";
        pathfile << "Start: (" << start_pos.x << ", " << start_pos.y << ")\n";
        pathfile << "Finish: (" << finish_pos.x << ", " << finish_pos.y << ")\n";
        pathfile << "Total de passos: " << path_parte1.size() << "\n\n";
        
        for (size_t i = 0; i < path_parte1.size(); i++) { // percorre cada passo
            pathfile << "Passo " << i << ": (" << path_parte1[i].x << ", " << path_parte1[i].y << ")\n";
        }
        
        pathfile << "\n=== CONCLUSÃO ===\n";
        pathfile << "O mapa criado durante a exploração (Parte 2) foi SUFICIENTE\n";
        pathfile << "para reproduzir o algoritmo da Parte 1 e encontrar o caminho!\n";
        
        pathfile.close();
        RCLCPP_INFO(get_logger(), "Caminho salvo em 'caminho_parte1_reproduzido.txt'");
        RCLCPP_INFO(get_logger(), "\n=== PARTE 2 CONCLUÍDA COM SUCESSO! ===\n");
    }

    // ========== DIJKSTRA ==========
    std::vector<Pos> run_dijkstra_parte1(const std::vector<std::vector<int>> &grid,
                                          const Pos &start, const Pos &goal) {
        int rows = grid.size(), cols = grid[0].size(); // pega dimensões do mapa
        const int INF = 1e9;
        std::vector<std::vector<int>> dist(rows, std::vector<int>(cols, INF)); // distância mínima conhecida
        std::vector<std::vector<Pos>> prev(rows, std::vector<Pos>(cols, {-1, -1})); // guarda pai de cada célula para reconstruir caminho

        auto cmp = [&](Pos a, Pos b){ return dist[a.x][a.y] > dist[b.x][b.y]; }; // compara distâncias para prioridade
        std::priority_queue<Pos, std::vector<Pos>, decltype(cmp)> pq(cmp); // fila de prioridade do Dijkstra

        dist[start.x][start.y] = 0; // distância inicial do start
        pq.push(start); // adiciona start na fila

        const int dx[4] = {1, -1, 0, 0}; // movimentos ortogonais
        const int dy[4] = {0, 0, 1, -1};

        while (!pq.empty()) { // enquanto houver células para explorar
            Pos u = pq.top(); pq.pop(); // pega célula com menor custo
            if (u.x == goal.x && u.y == goal.y) break; // se chegou no objetivo, para

            for (int k = 0; k < 4; k++) { // percorre vizinhos ortogonais
                int nx = u.x + dx[k], ny = u.y + dy[k];
                if (nx < 0 || ny < 0 || nx >= rows || ny >= cols) continue; // checagem de borda
                if (grid[nx][ny] == 1) continue; // ignora obstáculos

                int newcost = dist[u.x][u.y] + 1; // custo acumulado para vizinho
                if (newcost < dist[nx][ny]) { // se custo é melhor que anterior
                    dist[nx][ny] = newcost; // atualiza distância
                    prev[nx][ny] = u; // marca pai
                    pq.push({nx, ny}); // adiciona vizinho na fila
                }
            }
        }

        std::vector<Pos> p;
        for (Pos at = goal; at.x != -1; at = prev[at.x][at.y]) { // reconstrói caminho do objetivo para start
            p.push_back(at); // adiciona célula ao caminho
            if (at.x == start.x && at.y == start.y) break; // se chegou no start, para
        }
        std::reverse(p.begin(), p.end()); // inverte caminho para ficar start->goal
        return p; // retorna caminho
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Pathfinder>()); // cria o node Pathfinder e mantém ele ativo
    rclcpp::shutdown();
    return 0;
}
