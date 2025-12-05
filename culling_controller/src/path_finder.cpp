#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/msg/robot_sensors.hpp"
#include "cg_interfaces/srv/get_map.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"

#include <queue>
#include <vector>
#include <string>
#include <algorithm>
#include <memory>
#include <future>
#include <chrono>
#include <thread>

struct NodePos { 
    int x, y;
    bool operator==(const NodePos& other) const {
        return x == other.x && y == other.y;
    }
};

enum class MoveStatus {
    Success,
    Failed,   // service responded with success=false
    Timeout,  // no response in time
};


class Pathfinder : public rclcpp::Node {
public:
    Pathfinder()
    : Node("pathfinder_explore"),
      map_size_(100),
      rx_(map_size_/2), ry_(map_size_/2),
      start_set_(false), goal_set_(false), exploring_(true),
      executing_plan_(false), plan_step_(0)
    {
        // Assina sensores
        sensor_sub_ = create_subscription<cg_interfaces::msg::RobotSensors>(
            "/culling_games/robot_sensors", 10,
            std::bind(&Pathfinder::on_sensor, this, std::placeholders::_1)
        );
        
        // NEW: helper node only for movement service calls
        move_helper_node_ = rclcpp::Node::make_shared("pathfinder_move_helper");
        move_client_primary_ = move_helper_node_->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
        
        // keep /get_map client on this node
        map_client_ = create_client<cg_interfaces::srv::GetMap>("/get_map");

        // mapa desconhecido inicial: '?' = unknown, 'f','b','r','t'
        map_.assign(map_size_, std::vector<char>(map_size_, '?'));
        map_[rx_][ry_] = 'r';

        // timer para passo do explorador
        timer_ = create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&Pathfinder::step, this)
        );

        RCLCPP_INFO(get_logger(), "Pathfinder explorer started. map_size=%d", map_size_);
        RCLCPP_INFO(get_logger(), "Robot starting at (%d, %d)", rx_, ry_);
    }

private:
    int map_size_;
    int rx_, ry_;
    NodePos start_, goal_;
    bool start_set_, goal_set_;
    bool exploring_;
    
    // Estado de execução de plano
    bool executing_plan_;
    size_t plan_step_;
    std::vector<NodePos> current_plan_;

    // NEW: helper node for blocking move calls
    rclcpp::Node::SharedPtr move_helper_node_;

    std::vector<std::vector<char>> map_;
    std::vector<std::vector<char>> map_true_from_service_;

    rclcpp::Subscription<cg_interfaces::msg::RobotSensors>::SharedPtr sensor_sub_;
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_client_primary_;
    rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr map_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    // ---- sensores ----------------------------------------------------------------
    void on_sensor(const cg_interfaces::msg::RobotSensors::SharedPtr msg) {
        auto s = msg;

        auto set_if_valid = [&](int dx, int dy, const std::string& str){
            if(str.empty()) return;
            char c = str[0];
            
            int nx = rx_ + dx;
            int ny = ry_ + dy;
            
            if(nx >= 0 && ny >= 0 && nx < map_size_ && ny < map_size_) {
                // Atualiza a célula se ainda for desconhecida ou se for informação mais específica
                if(map_[nx][ny] == '?' || c == 't') {
                    map_[nx][ny] = c;
                }
                
                if(c == 't') { 
                    goal_ = {nx, ny}; 
                    goal_set_ = true;
                    RCLCPP_INFO(get_logger(), "Target found at (%d, %d)", nx, ny);
                }
            }
        };

        // Atualiza células ao redor
        set_if_valid(-1,  0, s->up);
        set_if_valid( 1,  0, s->down);
        set_if_valid( 0, -1, s->left);
        set_if_valid( 0,  1, s->right);
        set_if_valid(-1, -1, s->up_left);
        set_if_valid(-1,  1, s->up_right);
        set_if_valid( 1, -1, s->down_left);
        set_if_valid( 1,  1, s->down_right);

        // Marca posição atual como livre/robot
        map_[rx_][ry_] = 'r';
        if(!start_set_) { 
            start_ = {rx_, ry_}; 
            start_set_ = true;
            RCLCPP_INFO(get_logger(), "Start position set at (%d, %d)", rx_, ry_);
        }
    }

    // ---- passo principal ---------------------------------------------------------
            void step() {
                    if (!start_set_) {
                    return;
                }
            
                // NEW: if we are standing on the goal, stop everything and verify
                if (goal_set_ && rx_ == goal_.x && ry_ == goal_.y) {
                    RCLCPP_INFO(get_logger(),
                                "Goal reached at (%d,%d). Stopping exploration.",
                                rx_, ry_);
                    exploring_ = false;
                    executing_plan_ = false;
                    current_plan_.clear();
                    timer_->cancel();
                    verify_and_report();
                    return;
                }
            
                if (!exploring_) {
                    return;
                }
            
                // Se já está executando um plano, continua executando
                if (executing_plan_) {
                    if (plan_step_ >= current_plan_.size()) {
                        // Plano concluído
                        executing_plan_ = false;
                        plan_step_ = 0;
                        current_plan_.clear();
                        RCLCPP_INFO(get_logger(), "Plan completed, searching for new frontier...");
                        return;
                    }
                
                    // Executa próximo passo do plano
                    if (plan_step_ > 0) {
                        std::string dir = get_direction(current_plan_[plan_step_-1], current_plan_[plan_step_]);
                        if (dir.empty()) {
                            RCLCPP_WARN(get_logger(), "Invalid direction, aborting plan");
                            executing_plan_ = false;
                            plan_step_ = 0;
                            current_plan_.clear();
                            return;
                        }
                    
                        MoveStatus status = send_move_and_wait(dir);
                    
                        if (status == MoveStatus::Timeout) {
                            // Só avisa e tenta de novo no próximo tick
                            RCLCPP_WARN(get_logger(), "Move timed out, will retry same step");
                            return;
                        }
                    
                        if (status == MoveStatus::Failed) {
                            RCLCPP_WARN(get_logger(), "Move failed, marking target as blocked");

                            if (!current_plan_.empty()) {
                                NodePos blocked = current_plan_.back();
                            
                                // Again, don’t block the goal cell
                                if (!(goal_set_ && blocked.x == goal_.x && blocked.y == goal_.y) &&
                                    blocked.x >= 0 && blocked.y >= 0 &&
                                    blocked.x < map_size_ && blocked.y < map_size_) {
                                    map_[blocked.x][blocked.y] = 'b';
                                    RCLCPP_INFO(get_logger(),
                                                "Marked frontier (%d, %d) as blocked",
                                                blocked.x, blocked.y);
                                } else {
                                    RCLCPP_WARN(get_logger(),
                                                "Planned endpoint (%d,%d) is the goal; not marking as blocked",
                                                blocked.x, blocked.y);
                                }
                            }
                        
                            executing_plan_ = false;
                            plan_step_ = 0;
                            current_plan_.clear();
                            return;
                        }

                    
                        // Success
                        plan_step_++;
                        return;
                    } else {
                        // Primeiro "passo" é só para alinhar índice
                        plan_step_++;
                        return;
                    }
                }
            
                // Não está executando plano, então busca nova frontier
                auto frontier = find_frontiers();
            
                if (frontier.empty()) {
                    exploring_ = false;
                    RCLCPP_INFO(get_logger(), "Exploration completed!");
                    verify_and_report();
                    timer_->cancel();
                    return;
                }
            
                RCLCPP_INFO(get_logger(), "Found %zu frontiers, choosing best one...", frontier.size());
            
                // Escolhe frontier mais próxima que seja ALCANÇÁVEL
                NodePos target = pick_nearest_frontier(frontier);
            
                if (target.x == -1) {
                    RCLCPP_WARN(get_logger(), "No reachable frontier found, exploration may be complete");
                    exploring_ = false;
                    verify_and_report();
                    timer_->cancel();
                    return;
                }
            
                RCLCPP_INFO(get_logger(), "Targeting frontier at (%d, %d)", target.x, target.y);
            
                // Planeja caminho
                auto plan = run_dijkstra_on_known_map({rx_, ry_}, target);
            
                if (plan.empty()) {
                    // Não conseguiu planejar para essa frontier
                    map_[target.x][target.y] = 'b';
                    RCLCPP_WARN(get_logger(),
                                "Cannot plan to frontier (%d,%d) — marked as blocked, will try another",
                                target.x, target.y);
                    return;
                }
            
                RCLCPP_INFO(get_logger(), "Plan has %zu steps", plan.size());
            
                // Inicia execução do plano
                current_plan_ = plan;
                executing_plan_ = true;
                plan_step_ = 0;
            }


    // ---- frontiers & selection ---------------------------------------------------
    std::vector<NodePos> find_frontiers() {
        std::vector<NodePos> frontiers;
        const int dx[4] = {1,-1,0,0};
        const int dy[4] = {0,0,1,-1};
        
        for(int i = 0; i < map_size_; ++i) {
            for(int j = 0; j < map_size_; ++j) {
                // Só considera células livres ou do robô
                if(map_[i][j] != 'f' && map_[i][j] != 'r') continue;
                
                // Verifica se qualquer vizinho é desconhecido '?'
                bool is_frontier = false;
                for(int k=0;k<4;++k){
                    int ni = i + dx[k], nj = j + dy[k];
                    if(ni<0||nj<0||ni>=map_size_||nj>=map_size_) continue;
                    if(map_[ni][nj] == '?') { 
                        is_frontier = true; 
                        break; 
                    }
                }
                
                if(is_frontier) {
                    frontiers.push_back({i,j});
                }
            }
        }
        
        RCLCPP_INFO(get_logger(), "Found %zu frontiers", frontiers.size());
        return frontiers;
    }

    NodePos pick_nearest_frontier(const std::vector<NodePos>& frontiers) {
        size_t best_len = SIZE_MAX;
        NodePos best = {-1, -1};
        
        for(const auto& f : frontiers) {
            auto p = run_dijkstra_on_known_map({rx_, ry_}, f);
            if(!p.empty() && p.size() < best_len) {
                best_len = p.size();
                best = f;
            }
        }
        
        // Se não encontrou nenhuma frontier alcançável, retorna a primeira mesmo assim
        if(best.x == -1 && !frontiers.empty()) {
            best = frontiers.front();
        }
        
        return best;
    }

    // ---- Dijkstra sobre o mapa conhecido ----------------------------------------
    std::vector<NodePos> run_dijkstra_on_known_map(NodePos start, NodePos goal) {
        int rows = map_size_, cols = map_size_;
        const int INF = 1e9;

        std::vector<std::vector<int>> dist(rows, std::vector<int>(cols, INF));
        std::vector<std::vector<NodePos>> prev(rows, std::vector<NodePos>(cols, {-1,-1}));

        auto cmp = [&](NodePos a, NodePos b){ return dist[a.x][a.y] > dist[b.x][b.y]; };
        std::priority_queue<NodePos, std::vector<NodePos>, decltype(cmp)> pq(cmp);

        if(!is_walkable(start.x, start.y) || !is_walkable(goal.x, goal.y))
            return {};

        dist[start.x][start.y] = 0;
        pq.push(start);

        const int dx[4] = {1,-1,0,0};
        const int dy[4] = {0,0,1,-1};

        while(!pq.empty()) {
            NodePos u = pq.top(); pq.pop();
            
            if(u.x == goal.x && u.y == goal.y) break;
            
            // Skip if we already found a better path
            if(dist[u.x][u.y] < INF) {
                for(int k=0;k<4;k++){
                    int nx = u.x + dx[k];
                    int ny = u.y + dy[k];
                
                    if(nx<0||ny<0||nx>=rows||ny>=cols) 
                        continue;
                
                    if(!is_walkable(nx,ny)) 
                        continue;
                
                    int newcost = dist[u.x][u.y] + 1;
                
                    if(newcost < dist[nx][ny]) {
                        dist[nx][ny] = newcost;
                        prev[nx][ny] = u;
                        pq.push({nx,ny});
                    }
                }
            }
        }

        // reconstrói caminho
        std::vector<NodePos> path;
        NodePos at = goal;

        if(dist[goal.x][goal.y] == INF) {
            return {};
        }

        while(at.x != -1 && at.y != -1) {
            path.push_back(at);
            if(at.x == start.x && at.y == start.y) break;
            at = prev[at.x][at.y];
        }

        if(path.empty() || !(path.back().x == start.x && path.back().y == start.y))
            return {};

        std::reverse(path.begin(), path.end());
        return path;
    }

    bool is_walkable(int x,int y) {
        if(x<0||y<0||x>=map_size_||y>=map_size_) 
            return false;

        char c = map_[x][y];
        return (c == 'f' || c == 'r' || c == 't');
    }

    // ---- movimento --------------------------------------------------------------
MoveStatus send_move_and_wait(const std::string &dir) {
    auto client = move_client_primary_;
    if (!client->service_is_ready()) {
        RCLCPP_WARN(get_logger(), "MoveCmd service not ready");
        return MoveStatus::Timeout;
    }

    auto req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
    req->direction = dir;
    auto future = client->async_send_request(req);

    auto ret = rclcpp::spin_until_future_complete(
        move_helper_node_, future, std::chrono::seconds(5));
    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_WARN(get_logger(), "Timeout or error on move command");
        return MoveStatus::Timeout;
    }

    auto res = future.get();
    if (!res) {
        RCLCPP_WARN(get_logger(), "Null response from move_command");
        return MoveStatus::Timeout;
    }

    // Compute the intended target cell in *our* map
    int tx = rx_, ty = ry_;
    if (dir == "up")       tx -= 1;
    else if (dir == "down") tx += 1;
    else if (dir == "left") ty -= 1;
    else if (dir == "right")ty += 1;
    else {
        RCLCPP_WARN(get_logger(), "Unknown direction '%s'", dir.c_str());
        return MoveStatus::Failed;
    }

    if (!res->success) {
        // Mark that cell as blocked (but don't block the goal)
        if (tx >= 0 && ty >= 0 && tx < map_size_ && ty < map_size_) {
            if (!(goal_set_ && tx == goal_.x && ty == goal_.y)) {
                map_[tx][ty] = 'b';
                RCLCPP_INFO(get_logger(),
                            "Marked blocked due to failed move: (%d,%d)", tx, ty);
            }
        }
        return MoveStatus::Failed;
    }

    // Success: move robot internally
    int oldx = rx_, oldy = ry_;
    if (oldx >= 0 && oldy >= 0 && oldx < map_size_ && oldy < map_size_) {
        if (map_[oldx][oldy] == 'r') map_[oldx][oldy] = 'f';
    }

    rx_ = tx;
    ry_ = ty;

    if (rx_ >= 0 && ry_ >= 0 && rx_ < map_size_ && ry_ < map_size_) {
        map_[rx_][ry_] = 'r';
    }

    RCLCPP_INFO(get_logger(), "Moved %s to (%d, %d)", dir.c_str(), rx_, ry_);
    return MoveStatus::Success;
}






    std::string get_direction(const NodePos& a, const NodePos& b) {
        if(b.x == a.x + 1) return "down";
        if(b.x == a.x - 1) return "up";
        if(b.y == a.y + 1) return "right";
        if(b.y == a.y - 1) return "left";
        return "";
    }

    // ---- verificação final ------------------------------------------------------
    void verify_and_report() {
        if(!start_set_ || !goal_set_) {
            RCLCPP_WARN(get_logger(), "Start or goal not detected in mapped area. Cannot compare routes.");
            return;
        }

        RCLCPP_INFO(get_logger(), "Running Dijkstra on mapped map...");
        auto path_mapped = run_dijkstra_on_known_map(start_, goal_);
        if(path_mapped.empty()) {
            RCLCPP_ERROR(get_logger(), "Dijkstra on mapped map found no path!");
        } else {
            RCLCPP_INFO(get_logger(), "Path on mapped map: length=%ld", path_mapped.size());
            
            // Print the path
            RCLCPP_INFO(get_logger(), "Path coordinates:");
            for(size_t i = 0; i < path_mapped.size(); i++) {
                RCLCPP_INFO(get_logger(), "  Step %zu: (%d, %d)", i, path_mapped[i].x, path_mapped[i].y);
            }
        }

        // Tenta comparar com o mapa verdadeiro se disponível
        if (map_client_->service_is_ready()) {
            RCLCPP_INFO(get_logger(), "Requesting /get_map for verification...");
            auto req = std::make_shared<cg_interfaces::srv::GetMap::Request>();
        
            auto prom = std::make_shared<std::promise<cg_interfaces::srv::GetMap::Response::SharedPtr>>();
            auto fut  = prom->get_future();
        
            map_client_->async_send_request(
                req,
                [prom](rclcpp::Client<cg_interfaces::srv::GetMap>::SharedFuture future) {
                    try {
                        prom->set_value(future.get());
                    } catch(...) {
                        prom->set_value(nullptr);
                    }
                }
            );
        
            auto status = fut.wait_for(std::chrono::seconds(5));
            if (status == std::future_status::timeout) {
                RCLCPP_WARN(get_logger(), "Timeout waiting for /get_map — cannot compare.");
                return;
            }
        
            auto res_ptr = fut.get();
            if (!res_ptr) {
                RCLCPP_WARN(get_logger(), "Null response from /get_map — cannot compare.");
                return;
            }
        
            // Processa o mapa verdadeiro e compara
            compare_with_ground_truth(res_ptr, path_mapped);
        } else {
            RCLCPP_INFO(get_logger(), "/get_map not available — service verification not performed.");
        }

    }

    void compare_with_ground_truth(
        std::shared_ptr<cg_interfaces::srv::GetMap::Response> res_ptr,
        const std::vector<NodePos>& path_mapped)
    {
        int rows = res_ptr->occupancy_grid_shape[0];
        int cols = res_ptr->occupancy_grid_shape[1];
        
        RCLCPP_INFO(get_logger(), "Ground truth map size: %dx%d", rows, cols);
        
        // Converte para char map
        std::vector<std::vector<char>> true_map(rows, std::vector<char>(cols, 'b'));
        NodePos true_start{-1,-1}, true_goal{-1,-1};
        
        for(int i=0; i<rows; i++){
            for(int j=0; j<cols; j++){
                std::string cell = res_ptr->occupancy_grid_flattened[i*cols + j];
                if(!cell.empty()) {
                    char c = cell[0];
                    true_map[i][j] = c;
                    
                    if(c == 'r' || c == 's') true_start = {i,j};
                    if(c == 't') true_goal = {i,j};
                }
            }
        }
        
        if(true_start.x == -1 || true_goal.x == -1) {
            RCLCPP_WARN(get_logger(), "Could not find start/goal in ground truth map.");
            return;
        }
        
        RCLCPP_INFO(get_logger(), "Ground truth: start=(%d,%d) goal=(%d,%d)", 
                   true_start.x, true_start.y, true_goal.x, true_goal.y);
        
        // Roda Dijkstra no mapa verdadeiro
        auto path_true = run_dijkstra_on_arbitrary_map(true_map, true_start, true_goal);
        
        if(path_true.empty()) {
            RCLCPP_ERROR(get_logger(), "Dijkstra on ground truth found no path!");
        } else {
            RCLCPP_INFO(get_logger(), "Path on ground truth: length=%ld", path_true.size());
        }
        
        // Compara os caminhos
        if(!path_mapped.empty() && !path_true.empty()) {
            if(path_mapped.size() == path_true.size()) {
                RCLCPP_INFO(get_logger(), "SUCCESS: Both paths have the same length!");
            } else {
                RCLCPP_WARN(get_logger(), "Path lengths differ: mapped=%ld, truth=%ld", 
                           path_mapped.size(), path_true.size());
            }
        }
    }

    std::vector<NodePos> run_dijkstra_on_arbitrary_map(
        const std::vector<std::vector<char>>& amap,
        NodePos start, NodePos goal)
    {
        int rows = amap.size();
        if(rows == 0) return {};
        int cols = amap[0].size();
        const int INF = 1e9;
        
        std::vector<std::vector<int>> dist(rows, std::vector<int>(cols, INF));
        std::vector<std::vector<NodePos>> prev(rows, std::vector<NodePos>(cols, {-1,-1}));

        auto cmp = [&](NodePos a, NodePos b){ return dist[a.x][a.y] > dist[b.x][b.y]; };
        std::priority_queue<NodePos, std::vector<NodePos>, decltype(cmp)> pq(cmp);

        auto walkable = [&](int x,int y)->bool{
            if(x<0||y<0||x>=rows||y>=cols) return false;
            char c = amap[x][y];
            return (c == 'f' || c == 'r' || c == 't' || c == 's');
        };

        if(!walkable(start.x,start.y) || !walkable(goal.x,goal.y)) return {};

        dist[start.x][start.y] = 0;
        pq.push(start);
        const int dx[4] = {1,-1,0,0};
        const int dy[4] = {0,0,1,-1};

        while(!pq.empty()) {
            NodePos u = pq.top(); pq.pop();
            if(u.x == goal.x && u.y == goal.y) break;
            
            for(int k=0;k<4;k++){
                int nx = u.x + dx[k], ny = u.y + dy[k];
                if(!walkable(nx,ny)) continue;
                
                int newcost = dist[u.x][u.y] + 1;
                if(newcost < dist[nx][ny]) {
                    dist[nx][ny] = newcost;
                    prev[nx][ny] = u;
                    pq.push({nx,ny});
                }
            }
        }

        std::vector<NodePos> p;
        NodePos at = goal;
        
        if(dist[goal.x][goal.y] == INF) return {};
        
        while(at.x != -1 && at.y != -1) {
            p.push_back(at);
            if(at == start) break;
            at = prev[at.x][at.y];
        }
        
        if(p.empty() || !(p.back() == start)) return {};
        std::reverse(p.begin(), p.end());
        return p;
    }

};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Pathfinder>();

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
