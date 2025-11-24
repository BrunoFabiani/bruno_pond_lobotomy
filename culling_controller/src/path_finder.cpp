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
            "/culling_games/robot_sensors", 10,
            std::bind(&Pathfinder::on_sensor, this, std::placeholders::_1)
        );

        move_client_ = create_client<cg_interfaces::srv::MoveCmd>("move_command");

        internal_map.assign(MAP_SIZE, std::vector<char>(MAP_SIZE, '?'));
        robot = {MAP_SIZE/2, MAP_SIZE/2};
        internal_map[robot.x][robot.y] = 'r';
    }

private:
    static constexpr int MAP_SIZE = 200;

    std::vector<std::vector<char>> internal_map;
    Pos robot;
    std::vector<Pos> path;
    size_t step_idx = 0;

    rclcpp::Subscription<cg_interfaces::msg::RobotSensors>::SharedPtr sensor_sub_;
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_client_;

    //SENSOR CALLBACK
    void on_sensor(const cg_interfaces::msg::RobotSensors::SharedPtr msg) {
        update_map(msg);

        Pos finish = find_finish();
        if (finish.x != -1) {
            plan_path(robot, finish);
            move_one_step();
            return;
        }

        std::vector<Pos> frontier = find_frontier_cells();
        if (frontier.empty()) {
            RCLCPP_ERROR(get_logger(), "Map fully explored, finish does not exist");
            return;
        }

        Pos best = select_closest_frontier(frontier);
        plan_path(robot, best);
        move_one_step();
    }

    //MAP UPDATE
    void update_map(const cg_interfaces::msg::RobotSensors::SharedPtr msg) {
        auto mark = [&](int dx, int dy, const std::string &val) {
            char c = '?';
            if (val == "b") c = 'b';
            else if (val == "f") c = 'f';
            else if (val == "t") c = 't';
            int nx = robot.x + dx;
            int ny = robot.y + dy;
            internal_map[nx][ny] = c;
        };

        mark(-1, 0, msg->up);
        mark(1, 0, msg->down);
        mark(0, -1, msg->left);
        mark(0, 1, msg->right);
        mark(-1, -1, msg->up_left);
        mark(-1, 1, msg->up_right);
        mark(1, -1, msg->down_left);
        mark(1, 1, msg->down_right);

        internal_map[robot.x][robot.y] = 'f';
    }

    //FIND FINISH
    Pos find_finish() {
        for (int i = 0; i < MAP_SIZE; i++)
            for (int j = 0; j < MAP_SIZE; j++)
                if (internal_map[i][j] == 't')
                    return {i, j};
        return {-1, -1};
    }

    //FRONTIER
    bool has_unknown_neighbor(int x, int y) {
        const int d[4][2] = {{1,0},{-1,0},{0,1},{0,-1}};
        for (auto &o : d) {
            int nx = x + o[0], ny = y + o[1];
            if (internal_map[nx][ny] == '?') return true;
        }
        return false;
    }

    std::vector<Pos> find_frontier_cells() {
        std::vector<Pos> f;
        for (int i = 0; i < MAP_SIZE; i++)
            for (int j = 0; j < MAP_SIZE; j++)
                if (internal_map[i][j] == 'f' && has_unknown_neighbor(i, j))
                    f.push_back({i,j});
        return f;
    }

    Pos select_closest_frontier(const std::vector<Pos> &f) {
        Pos best = f[0];
        double best_h = 1e18;
        for (auto &p : f) {
            double h = std::abs(p.x - robot.x) + std::abs(p.y - robot.y);
            if (h < best_h) { best_h = h; best = p; }
        }
        return best;
    }

    //A*
    void plan_path(const Pos &start, const Pos &goal) {
        const int INF = 1e9;
        std::vector<std::vector<int>> g(MAP_SIZE, std::vector<int>(MAP_SIZE, INF));
        std::vector<std::vector<Pos>> parent(MAP_SIZE, std::vector<Pos>(MAP_SIZE, {-1,-1}));

        auto h = [&](Pos p) { return abs(p.x-goal.x) + abs(p.y-goal.y); };

        auto cmp = [&](Pos a, Pos b) {
            return (g[a.x][a.y] + h(a)) > (g[b.x][b.y] + h(b));
        };
        std::priority_queue<Pos, std::vector<Pos>, decltype(cmp)> open(cmp);

        g[start.x][start.y] = 0;
        open.push(start);

        const int d[4][2] = {{1,0},{-1,0},{0,1},{0,-1}};

        while (!open.empty()) {
            Pos u = open.top(); open.pop();
            if (u.x == goal.x && u.y == goal.y) break;

            for (auto &o : d) {
                int nx = u.x + o[0], ny = u.y + o[1];
                if (internal_map[nx][ny] == 'b' || internal_map[nx][ny] == '?') continue;
                int new_g = g[u.x][u.y] + 1;
                if (new_g < g[nx][ny]) {
                    g[nx][ny] = new_g;
                    parent[nx][ny] = u;
                    open.push({nx, ny});
                }
            }
        }

        path.clear();
        step_idx = 0;
        for (Pos p = goal; p.x != -1; p = parent[p.x][p.y]) {
            path.push_back(p);
            if (p.x == start.x && p.y == start.y) break;
        }
        std::reverse(path.begin(), path.end());
    }

    //MOVE 
    void move_one_step() {
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
