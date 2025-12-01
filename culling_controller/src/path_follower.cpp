#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/get_map.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"
#include <queue>
#include <vector>
#include <string>
#include <algorithm>

struct NodePos { int x, y; };

class Pathfinder : public rclcpp::Node {
public:
    Pathfinder() : Node("pathfinder") {
        map_client_  = create_client<cg_interfaces::srv::GetMap>("/get_map");
        move_client_ = create_client<cg_interfaces::srv::MoveCmd>("move_command");

        timer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&Pathfinder::step, this)
        );
    }

private:
    std::vector<std::vector<int>> grid; // 0=free,1=blocked,2=start,3=goal,4=robot start
    std::vector<NodePos> path;
    size_t step_idx = 0;
    bool map_loaded = false;

    void step() {
        if (!map_loaded) {
            request_map();
            return;
        }
        follow_path();
    }

    void request_map() {
        auto req = std::make_shared<cg_interfaces::srv::GetMap::Request>();
        map_client_->async_send_request(
            req,
            [this](std::shared_future<cg_interfaces::srv::GetMap::Response::SharedPtr> future) {
                auto res = future.get();
                int rows = res->occupancy_grid_shape[0];
                int cols = res->occupancy_grid_shape[1];
                grid.assign(rows, std::vector<int>(cols, 1)); // default blocked

                for (int i = 0; i < rows; i++) {
                    for (int j = 0; j < cols; j++) {
                        std::string cell = res->occupancy_grid_flattened[i * cols + j];
                        if (cell.empty()) { grid[i][j] = 1; continue; }
                        switch (cell[0]) {
                        case 'f': grid[i][j] = 0; break;
                        case 'b': grid[i][j] = 1; break;
                        case 's': grid[i][j] = 2; break;
                        case 't': grid[i][j] = 3; break;
                        case 'r': grid[i][j] = 4; break; // start do robô
                        default:  grid[i][j] = 1; break;
                        }
                    }
                }

                // log grid
                RCLCPP_INFO(get_logger(), "Grid:");
                for (int i = 0; i < rows; i++) {
                    std::string line;
                    for (int j = 0; j < cols; j++)
                        line += std::to_string(grid[i][j]);
                    RCLCPP_INFO(get_logger(), "%s", line.c_str());
                }

                path = run_dijkstra();
                if (path.empty())
                    RCLCPP_WARN(get_logger(), "No path found!");
                else
                    RCLCPP_INFO(get_logger(), "Path length: %ld", path.size());

                map_loaded = true;
            }
        );
    }

    std::vector<NodePos> run_dijkstra() {
        int rows = grid.size();
        int cols = grid[0].size();
        NodePos start{-1,-1}, goal{-1,-1};

        // CORREÇÃO: start reconhece 2 (s) ou 4 (r)
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++) {
                if (grid[i][j] == 2 || grid[i][j] == 4) start = {i,j};
                if (grid[i][j] == 3) goal = {i,j};
            }

        if (start.x == -1 || goal.x == -1) {
            RCLCPP_ERROR(get_logger(), "Start or goal not found!");
            return {};
        }

        const int INF = 1e9;
        std::vector<std::vector<int>> dist(rows, std::vector<int>(cols, INF));
        std::vector<std::vector<NodePos>> prev(rows, std::vector<NodePos>(cols, {-1,-1}));
        auto cmp = [&](NodePos a, NodePos b){ return dist[a.x][a.y] > dist[b.x][b.y]; };
        std::priority_queue<NodePos, std::vector<NodePos>, decltype(cmp)> pq(cmp);

        dist[start.x][start.y] = 0;
        pq.push(start);

        const int dx[4] = {1,-1,0,0};
        const int dy[4] = {0,0,1,-1};

        while(!pq.empty()) {
            NodePos u = pq.top(); pq.pop();
            if(u.x == goal.x && u.y == goal.y) break;

            for(int k = 0; k < 4; k++) {
                int nx = u.x + dx[k];
                int ny = u.y + dy[k];
                if(nx < 0 || ny < 0 || nx >= rows || ny >= cols) continue;
                if(grid[nx][ny] == 1) continue;

                int newcost = dist[u.x][u.y] + 1;
                if(newcost < dist[nx][ny]) {
                    dist[nx][ny] = newcost;
                    prev[nx][ny] = u;
                    pq.push({nx,ny});
                }
            }
        }

        // reconstrução do caminho
        std::vector<NodePos> p;
        NodePos at = goal;
        while(at.x != -1 && at.y != -1) {
            p.push_back(at);
            if(at.x == start.x && at.y == start.y) break;
            at = prev[at.x][at.y];
        }
        std::reverse(p.begin(), p.end());
        return p;
    }

    std::string get_direction(NodePos a, NodePos b) {
        if(b.x == a.x + 1) return "down";
        if(b.x == a.x - 1) return "up";
        if(b.y == a.y + 1) return "right";
        if(b.y == a.y - 1) return "left";
        return "";
    }

    void follow_path() {
        if(step_idx + 1 >= path.size()) return;
        NodePos curr = path[step_idx];
        NodePos next = path[step_idx+1];
        std::string dir = get_direction(curr,next);
        if(dir.empty()) return;

        auto req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        req->direction = dir;

        move_client_->async_send_request(
            req,
            [this](std::shared_future<cg_interfaces::srv::MoveCmd::Response::SharedPtr> future){
                auto res = future.get();
                if(res->success) step_idx++;
            }
        );
    }

    rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr map_client_;
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc,char* argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<Pathfinder>());
    rclcpp::shutdown();
    return 0;
}
