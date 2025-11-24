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
    std::vector<std::vector<int>> grid;
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

    // -------- MAP REQUEST ----------
    void request_map()
    {
        auto req = std::make_shared<cg_interfaces::srv::GetMap::Request>();

        map_client_->async_send_request(
            req,
            [this](std::shared_future<cg_interfaces::srv::GetMap::Response::SharedPtr> future) {
                auto res = future.get();

                int rows = res->occupancy_grid_shape[0];
                int cols = res->occupancy_grid_shape[1];
                grid.assign(rows, std::vector<int>(cols));

                for (int i = 0; i < rows; i++)
                    for (int j = 0; j < cols; j++)
                        grid[i][j] = std::stoi(res->occupancy_grid_flattened[i * cols + j]);

                path = run_dijkstra();
                map_loaded = true;

                RCLCPP_INFO(get_logger(), "Map received. Path length: %ld", path.size());
            }
        );
    }

    // -------- DIJKSTRA ----------
    std::vector<NodePos> run_dijkstra() {
        int rows = grid.size(), cols = grid[0].size();
        NodePos start{}, goal{};
        bool foundT = false;

        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++) {
                if (grid[i][j] == 2) start = {i, j};
                if (grid[i][j] == 3) { goal = {i, j}; foundT = true; }
            }
        if (!foundT) return {};

        const int INF = 1e9;
        std::vector<std::vector<int>> dist(rows, std::vector<int>(cols, INF));
        std::vector<std::vector<NodePos>> prev(rows, std::vector<NodePos>(cols, {-1, -1}));

        auto cmp = [&](NodePos a, NodePos b){ return dist[a.x][a.y] > dist[b.x][b.y]; };
        std::priority_queue<NodePos, std::vector<NodePos>, decltype(cmp)> pq(cmp);

        dist[start.x][start.y] = 0;
        pq.push(start);

        const int dx[4] = {1, -1, 0, 0};
        const int dy[4] = {0, 0, 1, -1};

        while (!pq.empty()) {
            NodePos u = pq.top(); pq.pop();
            if (u.x == goal.x && u.y == goal.y) break;

            for (int k = 0; k < 4; k++) {
                int nx = u.x + dx[k];
                int ny = u.y + dy[k];
                if (nx < 0 || ny < 0 || nx >= rows || ny >= cols) continue;
                if (grid[nx][ny] == 1) continue;

                int newcost = dist[u.x][u.y] + 1;
                if (newcost < dist[nx][ny]) {
                    dist[nx][ny] = newcost;
                    prev[nx][ny] = u;
                    pq.push({nx, ny});
                }
            }
        }

        std::vector<NodePos> p;
        for (NodePos at = goal; at.x != -1; at = prev[at.x][at.y]) {
            p.push_back(at);
            if (at.x == start.x && at.y == start.y) break;
        }
        std::reverse(p.begin(), p.end());
        return p;
    }

    // -------- MOVE ROBOT ----------
    std::string get_direction(NodePos a, NodePos b) {
        if (b.x == a.x + 1) return "down";
        if (b.x == a.x - 1) return "up";
        if (b.y == a.y + 1) return "right";
        if (b.y == a.y - 1) return "left";
        return "";
    }

    void follow_path()
    {
        if (step_idx + 1 >= path.size())
            return;

        NodePos curr = path[step_idx];
        NodePos next = path[step_idx + 1];
        std::string dir = get_direction(curr, next);

        if (dir.empty()) return;

        auto req = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        req->direction = dir;

        move_client_->async_send_request(
            req,
            [this](std::shared_future<cg_interfaces::srv::MoveCmd::Response::SharedPtr> future) {
                auto res = future.get();
                if (res->success)
                    step_idx++;
            }
        );
    }

    rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr map_client_;
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Pathfinder>());
    rclcpp::shutdown();
    return 0;
}
