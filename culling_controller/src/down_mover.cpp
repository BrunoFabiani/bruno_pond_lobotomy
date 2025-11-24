#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"


class DownMover : public rclcpp::Node {
public:
    DownMover() : Node("down_mover") {
        client_ = this->create_client<cg_interfaces::srv::MoveCmd>("move_command");

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&DownMover::timer_callback, this)
        );
    }

private:
    void timer_callback() {
        if (!client_->wait_for_service(std::chrono::milliseconds(100))) {
            RCLCPP_WARN(this->get_logger(), "move_command service not ready yet");
            return;
        }

        auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        request->direction = "down";

        auto future = client_->async_send_request(request,
            [this](rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedFuture result) {
                const auto response = result.get();

                RCLCPP_INFO(
                    this->get_logger(),
                    "Moved? %s | Pos = (%d, %d)",
                    response->success ? "yes" : "no",
                    response->robot_pos[0],
                    response->robot_pos[1]
                );

                if (!response->success) {
                    RCLCPP_WARN(this->get_logger(), "Hit a wall — staying here.");
                }
            }
        );
    }

    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DownMover>());
    rclcpp::shutdown();
    return 0;
}
