#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/msg/robot_sensors.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"

#include <vector>
#include <queue>
#include <algorithm>
#include <limits>
#include <cmath>

struct Pos { int x, y; };

class Pathfinder : public rclcpp::Node {
public:
    Pathfinder() : Node("pathfinder") {
        sensor_sub_ = create_subscription<cg_interfaces::msg::RobotSensors>(
            "/culling_games/robot_sensors", 1,
            std::bind(&Pathfinder::on_sensor, this, std::placeholders::_1) //manda a mensagem do sensor_sub pro on_sensor usar, o this indica qual objeto usar da classe pathfinder, como apenas recebo 1 mensagem do subscribor ele recebe o primeiro argumento
        ); //create_subscription recebe estes 3 argumentos. 

        move_client_ = create_client<cg_interfaces::srv::MoveCmd>("move_command"); //faz o pathfinder virar cliente do move_command.  

        internal_map.assign(MAP_SIZE, std::vector<char>(MAP_SIZE, '?')); //cria uma matriz com tamanho do MAP_SIZE x MAP_SIZE com ?. usa biblioteca vector. 
        robot = {MAP_SIZE/2, MAP_SIZE/2}; // poẽ o robo no meio do mapa
        internal_map[robot.x][robot.y] = 'r'; //marca o lugar do robô como r (como que estpa no mapa)
    }

private: // callbacks e chamadas tem que serem feitas estritamente dentro da classe, evita bleh bleh bleh
    // atributos da classe pathdfinder
    static constexpr int MAP_SIZE = 200; //constexpr é boa pratica de programação i guess

    std::vector<std::vector<char>> internal_map; //como vector muda em tempo de execução posso deixar assim
    Pos robot; //robo é do tipo que tem coordenadas
    std::vector<Pos> path;// caminho que o robô ira seguir
    size_t step_idx = 0;

    rclcpp::Subscription<cg_interfaces::msg::RobotSensors>::SharedPtr sensor_sub_; //criando o subscription, pegando da biblio do ros, configurado para lidar com mensagem do tipo RobotSensor. O Shared_Ptr é o que o ROS usa pra evitar ter que ficar usando o std. Estou declarando aqui.
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_client_; //mesma logica que o do serviço mas 

    //SENSOR CALLBACK toca a operação
    //SENSOR CALLBACK toca a operação
    void on_sensor(const cg_interfaces::msg::RobotSensors::SharedPtr msg) {
    update_map(msg);

    // checa se o final está visível no mapa
    Pos finish = find_finish();
    if (finish.x != -1) {
        // tenta planejar diretamente para o finish
        plan_path(robot, finish);
        if (!path.empty()) {
            move_one_step();
            return;
        }

        // se A* não encontrou caminho direto para o finish,
        // tenta planejar para cada vizinho ortogonal do finish (prioridade: cima/baixo/esq/dir)
        const int nd[4][2] = {{1,0},{-1,0},{0,1},{0,-1}};
        for (auto &o : nd) {
            int nx = finish.x + o[0], ny = finish.y + o[1];
            // checagem de borda do mapa
            if (nx < 0 || nx >= MAP_SIZE || ny < 0 || ny >= MAP_SIZE) continue;
            // só tenta se não for obstáculo (aceita 'f' ou mesmo 't' — '?' ainda pode ser problemático)
            if (internal_map[nx][ny] == 'b') continue;

            Pos candidate = {nx, ny};
            plan_path(robot, candidate);
            if (!path.empty()) {
                move_one_step();
                return;
            }
        }

        // se chegou aqui, não foi possível achar caminho até o finish nem até nenhum vizinho ortogonal
        RCLCPP_WARN(get_logger(), "Finish detected but no reachable adjacent cell found");
        return;
    }

    // se não há finish conhecido, segue exploração por fronteiras
    std::vector<Pos> frontier = find_frontier_cells();
    if (frontier.empty()) {
        RCLCPP_ERROR(get_logger(), "Map fully explored, finish does not exist");
        return;
    }

    Pos best = select_closest_frontier(frontier);
    plan_path(robot, best);
    if (!path.empty()) move_one_step();
    }


    //MAP UPDATE
    void update_map(const cg_interfaces::msg::RobotSensors::SharedPtr msg) {
        auto mark = [&](int dx, int dy, const std::string &val) { // [&] indica que esta lambda captura todas as variáveis do escopo externo por referência.
            char c = '?'; // ser uma lambda faz com que ele acesse o robot e internal_map sem problemas
            if (val == "b") c = 'b';
            else if (val == "f") c = 'f';
            else if (val == "t") c = 't';
            int nx = robot.x + dx;
            int ny = robot.y + dy;
            internal_map[nx][ny] = c;
        };
        //marca no mapa todas as coisas que o sensor ta pegando e o robô pode ver, ver o objetivo na diagonal quebra a lógica e n faz sentido
        mark(-1, 0, msg->up);
        mark(1, 0, msg->down);
        mark(0, -1, msg->left);
        mark(0, 1, msg->right);

        internal_map[robot.x][robot.y] = 'f'; //marca a posição do robô cmoo livre, já que ele está nela.
    }

    //FIND FINISH
    Pos find_finish() {
        for (int i = 0; i < MAP_SIZE; i++)
            for (int j = 0; j < MAP_SIZE; j++)
                if (internal_map[i][j] == 't')
                    return {i, j};
        return {-1, -1}; // como vector n tem coordenada negativa, isto basicamente sinaliza que não encontramo o final
    } // sempre checa se encontramo o final, 

    //FRONTIER
    bool has_unknown_neighbor(int x, int y) { // checa se todos foram explorados
        const int d[4][2] = {{1,0},{-1,0},{0,1},{0,-1}}; //d é um 2d array de inteiros
        for (auto &o : d) { 
            int nx = x + o[0], ny = y + o[1];
            if (internal_map[nx][ny] == '?') return true;
        }
        return false;
    }

    std::vector<Pos> find_frontier_cells() { //acha as células que podem ser exploradas 
        std::vector<Pos> f;
        for (int i = 0; i < MAP_SIZE; i++)
            for (int j = 0; j < MAP_SIZE; j++)
                if (internal_map[i][j] == 'f' && has_unknown_neighbor(i, j))
                    f.push_back({i,j});
        return f; //depois de checar todas células do mapa, retorna f pras livres.
    }

    Pos select_closest_frontier(const std::vector<Pos> &f) { //checa os f's 
        Pos best = f[0];
        double best_h = 1e4; //valor enorme  pra garantir que qualquer distancia seja menor
        for (auto &p : f) { //percorre cada célula dentro do vetor
            double h = std::abs(p.x - robot.x) + std::abs(p.y - robot.y); 
            if (h < best_h) { best_h = h; best = p; }
        }
        return best;
    }

//A* 
    void plan_path(const Pos &start, const Pos &goal) { // o mapa é constantemente re-atualizado, então ele está sempre calculando a melhor rota
        const int INF = 1e9;
        std::vector<std::vector<int>> g(MAP_SIZE, std::vector<int>(MAP_SIZE, INF)); //guarda o custo minimo conhecido 
        std::vector<std::vector<Pos>> parent(MAP_SIZE, std::vector<Pos>(MAP_SIZE, {-1,-1}));

        // Manhattan como heurística para mover apenas ortogonalmente
        auto h = [&](Pos p) { return abs(p.x-goal.x) + abs(p.y-goal.y); }; //retorna a distancia Manhattan de p até o goal

        auto cmp = [&](Pos a, Pos b) {
            return (g[a.x][a.y] + h(a)) > (g[b.x][b.y] + h(b));
        }; //função de comparação para a fila de prioridade
        std::priority_queue<Pos, std::vector<Pos>, decltype(cmp)> open(cmp); //

        g[start.x][start.y] = 0;
        open.push(start); //cria a priority que vai guardar o menor custo estimado 

        const int d[4][2] = {{1,0},{-1,0},{0,1},{0,-1}}; //só ortogonal, evita diagonais

        while (!open.empty()) { //enquanto houver células pra explorar
            Pos u = open.top(); open.pop();
            if (u.x == goal.x && u.y == goal.y) break;

            for (auto &o : d) { //percorre os 4 vizinhos ortogonais
                int nx = u.x + o[0], ny = u.y + o[1]; //calcula as coordenadas reais do vizinho, somando a célula atual u
                if (internal_map[nx][ny] == 'b' || internal_map[nx][ny] == '?') continue; //n faz sentido ver vizinhos que n podem ser visitados pq são células invalidas
                int new_g = g[u.x][u.y] + 1; //como cada passo tem 1 de valor incrementamos 1 pro custo real acumulado
                if (new_g < g[nx][ny]) {
                    g[nx][ny] = new_g;
                    parent[nx][ny] = u;
                    open.push({nx, ny});
                }
            }
        }

    path.clear(); //para garantir que não tenha nenhum caminho antigo armazenado
        step_idx = 0;
        for (Pos p = goal; p.x != -1; p = parent[p.x][p.y]) { //com o p = parent ele vai pegando o pai de cada posição pra reconstruir o caminho
            path.push_back(p); //vai adicionando o elemento p no final do vetor path
            if (p.x == start.x && p.y == start.y) break;
        }
        std::reverse(path.begin(), path.end()); // inverte pq isto vai pegando o pai de cada célula
    }   

    //MOVE 
    void move_one_step() { //logica pra mover o robô
        if (path.empty() || step_idx + 1 >= path.size()) return;

        Pos curr = path[step_idx], next = path[step_idx + 1];
        std::string d;

        if (next.x == curr.x + 1) d = "down";
        else if (next.x == curr.x - 1) d = "up";
        else if (next.y == curr.y + 1) d = "right";
        else if (next.y == curr.y - 1) d = "left";
        else return;

        auto req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        req->direction = d;

        move_client_->async_send_request(
            req,
            [this, next](rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedFuture future) {
                auto res = future.get();
                if (res->success) {
                    robot = next;
                    step_idx++;
                } else {
                    internal_map[next.x][next.y] = 'b';
                }
            }
        );
    }

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Pathfinder>());
    rclcpp::shutdown();
    return 0;
}
