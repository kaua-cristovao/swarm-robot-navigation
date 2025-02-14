#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <chrono>
#include <cmath>

class SimpleNavigatorServer : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigate = rclcpp_action::ServerGoalHandle<NavigateToPose>;

    SimpleNavigatorServer() : Node("simple_navigator_server") {
        // Criando o servidor de ação
        action_server_ = rclcpp_action::create_server<NavigateToPose>(
            this,
            "navigate_to_pose",
            std::bind(&SimpleNavigatorServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&SimpleNavigatorServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&SimpleNavigatorServer::handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Servidor de navegação iniciado!");
    }

private:
    rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;

    // Callback para aceitar ou rejeitar a meta recebida
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const NavigateToPose::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Recebido objetivo para (%.2f, %.2f)", goal->pose.pose.position.x, goal->pose.pose.position.y);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Callback para tratar cancelamento de objetivo
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleNavigate> goal_handle) {
        RCLCPP_WARN(this->get_logger(), "Objetivo cancelado!");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // Callback para iniciar a execução da meta aceita
    void handle_accepted(const std::shared_ptr<GoalHandleNavigate> goal_handle) {
        std::thread{std::bind(&SimpleNavigatorServer::execute, this, goal_handle)}.detach();
    }

    // Execução da meta de navegação
    void execute(const std::shared_ptr<GoalHandleNavigate> goal_handle) {
        auto feedback = std::make_shared<NavigateToPose::Feedback>();
        auto result = std::make_shared<NavigateToPose::Result>();

        auto goal = goal_handle->get_goal();
        double x_goal = goal->pose.pose.position.x;
        double y_goal = goal->pose.pose.position.y;
        
        double distance_remaining = std::sqrt(x_goal * x_goal + y_goal * y_goal);

        while (distance_remaining > 0.1) {
            if (goal_handle->is_canceling()) {
                result->result_code = nav2_msgs::action::NavigateToPose::Result::RESULT_CANCELED;
                goal_handle->canceled(result);
                RCLCPP_WARN(this->get_logger(), "Objetivo cancelado durante a execução.");
                return;
            }

            distance_remaining *= 0.9;  // Simula a redução da distância

            feedback->distance_remaining = distance_remaining;
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Distância restante: %.2f", distance_remaining);

            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        result->result_code = nav2_msgs::action::NavigateToPose::Result::RESULT_SUCCESS;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Navegação concluída!");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleNavigatorServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
