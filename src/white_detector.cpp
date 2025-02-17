#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/string.hpp"


#define gray 102

using namespace std::chrono_literals;

class WhiteObjectDetector : public rclcpp::Node {
public:
    WhiteObjectDetector() : Node("white_object_detector_pure") {
        // Inscreve-se no tópico da câmera
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&WhiteObjectDetector::image_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<std_msgs::msg::String>(
            "/processed_image_data", 10);
            timer_ = this->create_wall_timer(
            500ms, std::bind(&WhiteObjectDetector::timer_callback, this));
        
        // Serviço que retorna a fração de pixels brancos
        service_ = this->create_service<std_srvs::srv::Trigger>(
            "/white_fraction",
            std::bind(&WhiteObjectDetector::handle_service, this, std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(this->get_logger(), "Nó de detecção de objetos brancos iniciado.");
    }

private:

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    double white_fraction_ = 0.0; // Fração de pixels brancos na última imagem 
    double direction_ = 0.0; //Rotacao para a esquerda é positiva pela regra da mão direita

    bool get_threshold(int color, int thr){
        return (color > gray - thr) && (color < gray + thr);
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Verifica se a imagem recebida é do tipo RGB8
        if (msg->encoding != "rgb8" && msg->encoding != "bgr8") {
            RCLCPP_ERROR(this->get_logger(), "Formato de imagem não suportado: %s", msg->encoding.c_str());
            return;
        }

        // Acessa os dados da imagem diretamente
        const uint8_t* data = msg->data.data();
        int width = msg->width;
        int height = msg->height;
        int step = msg->step; // Número de bytes por linha

        int white_pixels = 0;
        int total_pixels = width * height;
        int half_width = width / 2;
        int direction = 0 ;


        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int index = y * step + x * 3; // Cada pixel tem 3 bytes (R, G, B)
                uint8_t r = data[index];
                uint8_t g = data[index + 1];
                uint8_t b = data[index + 2];

                // Verifica se o pixel é branco (limiar de intensidade acima de 240)
                if (get_threshold(r,10) && get_threshold(g,10) && get_threshold(b,10)) {
                    white_pixels++;
                    direction += half_width - x;
                }
            }
        }

        white_fraction_ = static_cast<double>(white_pixels) / total_pixels;
        direction_ = static_cast<double>(direction) / white_pixels;

        // Se mais de 5% da imagem for branca, gera um log
        // if (white_fraction_ > 0.05) {
        // RCLCPP_WARN(this->get_logger(), "Objeto branco detectado! (%.2f%% da imagem, direcao %.2f)", white_fraction_ * 100, direction_);
        // }
    }

    void handle_service(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response
    ) {
        (void) request; // Não usado
        response->success = true;
        response->message = "Fração de pixels brancos: " + std::to_string(white_fraction_) + " Direcao: " + std::to_string(direction_);
        RCLCPP_INFO(this->get_logger(), "Serviço chamado: %s", response->message.c_str());
    }

    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Fração de pixels brancos: " + std::to_string(white_fraction_) + " Direcao: " + std::to_string(direction_);
      RCLCPP_INFO(this->get_logger(), "Publicando: '%s'", message.data.c_str());
      publisher_->publish(message);
    }

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WhiteObjectDetector>());
    rclcpp::shutdown();
    return 0;
}
