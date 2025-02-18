#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <laser_geometry/laser_geometry.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <vector>
#include <mutex>

class MergeScansNode : public rclcpp::Node {
public:
    MergeScansNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) 
        : Node("merge_scans_node", options) {
        
        // Parâmetros do nó
        this->declare_parameter("publish_frequency", 1.0);
        double publish_frequency = this->get_parameter("publish_frequency").as_double();
        
        this->declare_parameter("scan_topics", std::vector<std::string>());
        scan_topics_ = this->get_parameter("scan_topics").as_string_array();

        if (scan_topics_.empty()) {
            RCLCPP_WARN(this->get_logger(), "Nenhum tópico de scan específico fornecido. Usando o tópico padrão /scan");
            scan_topics_.push_back("/scan");
        }

        this->declare_parameter("target_frame", "map");
        target_frame_ = this->get_parameter("target_frame").as_string();
        RCLCPP_INFO(this->get_logger(), "Transformando para o frame %s", target_frame_.c_str());

        // Inicializa o TF Listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Inscreve-se em todos os tópicos de scan
        for (const auto & topic : scan_topics_) {
            scan_subs_.push_back(this->create_subscription<sensor_msgs::msg::LaserScan>(
                topic, rclcpp::SensorDataQoS(),
                std::bind(&MergeScansNode::scanCallback, this, std::placeholders::_1)
            ));
            RCLCPP_INFO(this->get_logger(), "Inscrito no tópico %s", topic.c_str());
        }

        // Publicador do PointCloud2 combinado
        merged_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/merged_pointcloud", 10);
        
        // Configuração do timer para publicação periódica
        auto publish_period_ms = static_cast<int>(1000.0 / publish_frequency);
        RCLCPP_INFO(this->get_logger(), "Publicando a cada %d ms (%.1f Hz)", publish_period_ms, publish_frequency);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(publish_period_ms),
            std::bind(&MergeScansNode::publishMergedCloud, this));
    }

private:
    std::vector<std::string> scan_topics_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> scan_subs_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_cloud_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::mutex cloud_mutex_;
    std::string target_frame_;
    laser_geometry::LaserProjection projector_;

    std::vector<sensor_msgs::msg::PointCloud2> scans_;

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::string frame_id = msg->header.frame_id;

        // Converter LaserScan para PointCloud2
        sensor_msgs::msg::PointCloud2 cloud;
        try {
            // Projetar laser scan para point cloud
            projector_.projectLaser(*msg, cloud);
            
            // Transformar para o frame alvo
            sensor_msgs::msg::PointCloud2 transformed_cloud;
            if (frame_id != target_frame_) {
                geometry_msgs::msg::TransformStamped transform_stamped;
                transform_stamped = tf_buffer_->lookupTransform(
                    target_frame_, cloud.header.frame_id, 
                    cloud.header.stamp, tf2::durationFromSec(0.1));
                tf2::doTransform(cloud, transformed_cloud, transform_stamped);
            } else {
                transformed_cloud = cloud;
            }

            // Armazena a nuvem transformada
            std::lock_guard<std::mutex> lock(cloud_mutex_);
            scans_.push_back(transformed_cloud);
            
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Falha ao transformar %s para %s: %s", 
                        frame_id.c_str(), target_frame_.c_str(), ex.what());
        }
    }

    void publishMergedCloud() {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        if (scans_.empty()) return;
    
        // Inicia a nuvem mesclada com a primeira mensagem
        sensor_msgs::msg::PointCloud2 merged_cloud = scans_[0];
        
        // Calcule o total de pontos
        uint32_t total_points = merged_cloud.width;  // assume height = 1
    
        // Mescla os dados das demais nuvens
        for (size_t i = 1; i < scans_.size(); i++) {
            // Assumindo que todas as nuvens têm o mesmo point_step e height = 1
            total_points += scans_[i].width;
            merged_cloud.data.insert(
                merged_cloud.data.end(),
                scans_[i].data.begin(),
                scans_[i].data.end()
            );
        }
    
        // Atualiza a largura para refletir o número total de pontos
        merged_cloud.width = total_points;
        // Se todas as nuvens são não organizadas, height permanece 1
    
        // Publica a mensagem
        merged_cloud.header.stamp = this->now();
        merged_cloud.header.frame_id = target_frame_;
        merged_cloud_pub_->publish(merged_cloud);
    
        // Limpa os dados para o próximo ciclo
        scans_.clear();
        
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MergeScansNode>());
    rclcpp::shutdown();
    return 0;
}
