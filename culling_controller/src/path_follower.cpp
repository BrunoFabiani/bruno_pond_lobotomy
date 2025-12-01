#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/get_map.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"
#include <queue>
#include <vector>
#include <string>
#include <algorithm>

struct NodePos { int x, y; }; // estrutura para guardar coordenadas de cada célula

class Pathfinder : public rclcpp::Node {
public:
    Pathfinder() : Node("pathfinder") {
        map_client_  = create_client<cg_interfaces::srv::GetMap>("/get_map"); // cliente para pegar o mapa
        move_client_ = create_client<cg_interfaces::srv::MoveCmd>("move_command"); // cliente para mover o robô

        timer_ = create_wall_timer(
            std::chrono::milliseconds(500), // timer que chama step a cada 500ms
            std::bind(&Pathfinder::step, this) // chama step usando o this para referenciar o objeto atual
        );
    }

private:
    static constexpr int MAP_SIZE = 200; // tamanho máximo do mapa

    std::vector<std::vector<char>> internal_map; // mapa global com caracteres ('b'=bloqueado, 'f'=livre, '?'=desconhecido, 't'=target)
    std::vector<NodePos> path; // caminho calculado pelo Dijkstra
    size_t step_idx = 0; // índice atual do passo que o robô está seguindo
    bool map_loaded = false; // flag para indicar se o mapa já foi carregado
    int map_rows = 0, map_cols = 0; // dimensões reais do mapa recebido

    void step() { 
        if (!map_loaded) { 
            request_map(); // se mapa não carregado, solicita o mapa
            return; 
        }
        follow_path(); // se mapa já carregado, segue o caminho planejado
    }

    // MAP REQUEST
    void request_map() {
        auto req = std::make_shared<cg_interfaces::srv::GetMap::Request>(); // cria requisição do serviço GetMap

        map_client_->async_send_request(
            req,
            [this](std::shared_future<cg_interfaces::srv::GetMap::Response::SharedPtr> future) {
                auto res = future.get(); // pega a resposta do serviço

                map_rows = res->occupancy_grid_shape[0]; // número de linhas do mapa
                map_cols = res->occupancy_grid_shape[1]; // número de colunas do mapa
                
                internal_map.assign(map_rows, std::vector<char>(map_cols, '?')); // inicializa internal_map com caracteres

                // Converte o grid recebido (0,1,2,3) para internal_map (caracteres)
                for (int i = 0; i < map_rows; i++) {
                    for (int j = 0; j < map_cols; j++) {
                        int val = std::stoi(res->occupancy_grid_flattened[i * map_cols + j]); // pega valor do grid flatten
                        
                        if (val == 0) internal_map[i][j] = 'f'; // 0 = livre (free)
                        else if (val == 1) internal_map[i][j] = 'b'; // 1 = bloqueado (blocked)
                        else if (val == 2) internal_map[i][j] = 's'; // 2 = start
                        else if (val == 3) internal_map[i][j] = 't'; // 3 = target/goal
                    }
                }

                path = run_dijkstra(); // calcula o caminho usando Dijkstra
                map_loaded = true; // marca que o mapa já foi carregado

                RCLCPP_INFO(get_logger(), "Map received. Path length: %ld", path.size()); // loga info do tamanho do caminho
            }
        );
    }

    // DIJKSTRA
    std::vector<NodePos> run_dijkstra() {
        NodePos start{}, goal{}; 
        bool foundT = false; // flag para indicar se achou o objetivo

        // Procura start e goal no internal_map
        for (int i = 0; i < map_rows; i++) {
            for (int j = 0; j < map_cols; j++) {
                if (internal_map[i][j] == 's') start = {i, j}; // célula de início (start)
                if (internal_map[i][j] == 't') { goal = {i, j}; foundT = true; } // célula do objetivo (target)
            }
        }
        if (!foundT) return {}; // se não encontrou objetivo, retorna caminho vazio

        const int INF = 1e9; 
        std::vector<std::vector<int>> dist(map_rows, std::vector<int>(map_cols, INF)); // distância mínima conhecida
        std::vector<std::vector<NodePos>> prev(map_rows, std::vector<NodePos>(map_cols, {-1, -1})); // guarda pai de cada célula para reconstruir caminho

        auto cmp = [&](NodePos a, NodePos b){ return dist[a.x][a.y] > dist[b.x][b.y]; }; // compara distâncias para prioridade
        std::priority_queue<NodePos, std::vector<NodePos>, decltype(cmp)> pq(cmp); // fila de prioridade do Dijkstra

        dist[start.x][start.y] = 0; // distância inicial do start
        pq.push(start); // adiciona start na fila

        const int dx[4] = {1, -1, 0, 0}; // movimentos ortogonais
        const int dy[4] = {0, 0, 1, -1};

        while (!pq.empty()) { // enquanto houver células para explorar
            NodePos u = pq.top(); pq.pop(); // pega célula com menor custo
            if (u.x == goal.x && u.y == goal.y) break; // se chegou no objetivo, para

            for (int k = 0; k < 4; k++) { // percorre vizinhos ortogonais
                int nx = u.x + dx[k], ny = u.y + dy[k];
                if (nx < 0 || ny < 0 || nx >= map_rows || ny >= map_cols) continue; // checagem de borda
                if (internal_map[nx][ny] == 'b') continue; // ignora obstáculos (bloqueados)

                int newcost = dist[u.x][u.y] + 1; // custo acumulado para vizinho
                if (newcost < dist[nx][ny]) { // se custo é melhor que anterior
                    dist[nx][ny] = newcost; // atualiza distância
                    prev[nx][ny] = u; // marca pai
                    pq.push({nx, ny}); // adiciona vizinho na fila
                }
            }
        }

        std::vector<NodePos> p; 
        for (NodePos at = goal; at.x != -1; at = prev[at.x][at.y]) { // reconstrói caminho do objetivo para start
            p.push_back(at); // adiciona célula ao caminho
            if (at.x == start.x && at.y == start.y) break; // se chegou no start, para
        }
        std::reverse(p.begin(), p.end()); // inverte caminho para ficar start->goal
        return p; // retorna caminho
    }

    // MOVE ROBOT
    std::string get_direction(NodePos a, NodePos b) { // calcula direção entre duas células ortogonais
        if (b.x == a.x + 1) return "down";
        if (b.x == a.x - 1) return "up";
        if (b.y == a.y + 1) return "right";
        if (b.y == a.y - 1) return "left";
        return ""; // não move se não for ortogonal
    }

    void follow_path() { 
        if (step_idx + 1 >= path.size()) return; // se já está no final, não faz nada

        NodePos curr = path[step_idx]; 
        NodePos next = path[step_idx + 1]; 
        std::string dir = get_direction(curr, next); // calcula direção do próximo passo

        if (dir.empty()) return; // se direção inválida, retorna

        auto req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>(); 
        req->direction = dir; // seta direção da requisição

        move_client_->async_send_request(
            req,
            [this](std::shared_future<cg_interfaces::srv::MoveCmd::Response::SharedPtr> future) { 
                auto res = future.get(); 
                if (res->success) // se movimento foi aceito
                    step_idx++; // atualiza índice do passo atual
            }
        );
    }

    rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr map_client_; // cliente do serviço GetMap
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_client_; // cliente do serviço MoveCmd
    rclcpp::TimerBase::SharedPtr timer_; // timer para chamada periódica da função step
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<Pathfinder>()); // cria o node Pathfinder e mantém ele ativo
    rclcpp::shutdown(); 
    return 0; 
}
