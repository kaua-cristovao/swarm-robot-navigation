#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <mutex>
#include <map>


using mapa_t = nav_msgs::msg::OccupancyGrid;
using mapa_pt = std::shared_ptr<mapa_t>;
class MergeMapsNode : public rclcpp::Node {
public:
    MergeMapsNode() : Node("merge_maps_node") {
        // Declare parâmetro para os namespaces dos robôs
        this->declare_parameter("robot_namespaces", std::vector<std::string>{});
        robot_namespaces_ = this->get_parameter("robot_namespaces").as_string_array();

        if (robot_namespaces_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Nenhum namespace de robô especificado!");
            rclcpp::shutdown();
        }

        // Criar assinaturas para cada tópico de mapa
        for (const auto & ns : robot_namespaces_) {
            std::string topic = "/" + ns + "/map";
            auto sub = this->create_subscription<mapa_t>(
                topic, 10,
                [this, ns](mapa_pt msg) {
                    this->map_callback(msg, ns);
                });
            map_subs_[ns] = sub;
        }

        // Criar publicador para o mapa mesclado
        merged_map_pub_ = this->create_publisher<mapa_t>("/merged_map", 10);

        // Criar timer para publicar periodicamente
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&MergeMapsNode::publish_merged_map, this));
    }

private:
    std::vector<std::string> robot_namespaces_;
    std::map<std::string, mapa_pt> maps_;
    std::map<std::string, rclcpp::Subscription<mapa_t>::SharedPtr> map_subs_;
    rclcpp::Publisher<mapa_t>::SharedPtr merged_map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::mutex map_mutex_;

    void map_callback(const mapa_pt msg, const std::string &ns) {
        std::lock_guard<std::mutex> lock(map_mutex_);
        maps_[ns] = msg;
    }

    uint8_t get_occupation_value(const mapa_t &base_map, double x_wanted, double y_wanted) {   
        if (x_wanted < base_map.info.origin.position.x || 
            x_wanted > base_map.info.origin.position.x + base_map.info.width * base_map.info.resolution ||
            y_wanted < base_map.info.origin.position.y || 
            y_wanted > base_map.info.origin.position.y + base_map.info.height * base_map.info.resolution) {
            return -1;
        }
    
        unsigned int i_index = static_cast<unsigned int>((x_wanted - base_map.info.origin.position.x) / base_map.info.resolution);
        unsigned int j_index = static_cast<unsigned int>((y_wanted - base_map.info.origin.position.y) / base_map.info.resolution);
    
        if (i_index >= base_map.info.width || j_index >= base_map.info.height) {
            return -1;
        }
        
        return base_map.data[j_index * base_map.info.width + i_index];
    }
    
    void set_occupation_value(mapa_t &base_map, double x_wanted, double y_wanted, uint8_t occup) {   
        if (x_wanted < base_map.info.origin.position.x || 
            x_wanted > base_map.info.origin.position.x + base_map.info.width * base_map.info.resolution ||
            y_wanted < base_map.info.origin.position.y || 
            y_wanted > base_map.info.origin.position.y + base_map.info.height * base_map.info.resolution) {
            return;
        }
    
        unsigned int i_index = static_cast<unsigned int>((x_wanted - base_map.info.origin.position.x) / base_map.info.resolution);
        unsigned int j_index = static_cast<unsigned int>((y_wanted - base_map.info.origin.position.y) / base_map.info.resolution);
    
        if (i_index >= base_map.info.width || j_index >= base_map.info.height) {
            return;
        }
        
        base_map.data[j_index * base_map.info.width + i_index] = occup;
    }
    
    mapa_t create_minimum_map() {
        mapa_t retorno;
        retorno.header.stamp = rclcpp::Clock().now();
        retorno.header.frame_id = "map";
    
        // Encontrar os limites extremos de todos os mapas
        double min_x = std::numeric_limits<double>::max();
        double min_y = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::lowest();
        double max_y = std::numeric_limits<double>::lowest();
        
        for (const auto& map : maps_) {
            if (!map.second) continue;
            
            double map_min_x = map.second->info.origin.position.x;
            double map_min_y = map.second->info.origin.position.y;
            double map_max_x = map_min_x + map.second->info.width * map.second->info.resolution;
            double map_max_y = map_min_y + map.second->info.height * map.second->info.resolution;
            
            min_x = std::min(min_x, map_min_x);
            min_y = std::min(min_y, map_min_y);
            max_x = std::max(max_x, map_max_x);
            max_y = std::max(max_y, map_max_y);
        }
    
        retorno.info.resolution = maps_.begin()->second->info.resolution;
        retorno.info.origin.position.x = min_x;
        retorno.info.origin.position.y = min_y;
        
        retorno.info.width = std::ceil((max_x - min_x) / retorno.info.resolution);
        retorno.info.height = std::ceil((max_y - min_y) / retorno.info.resolution);
        
        retorno.data.assign(retorno.info.width * retorno.info.height, -1);
        return retorno;
    }
    
    void merge_map(mapa_t& mapa1, const mapa_pt mapa2) {
        for (unsigned int i = 0; i < mapa2->info.width; ++i) {
            for (unsigned int j = 0; j < mapa2->info.height; ++j) {
                if (mapa2->data[j * mapa2->info.width + i] == -1) continue;
    
                double x_world = mapa2->info.origin.position.x + i * mapa2->info.resolution;
                double y_world = mapa2->info.origin.position.y + j * mapa2->info.resolution;
                
                auto current_value = get_occupation_value(mapa1, x_world, y_world);
                auto new_value = mapa2->data[j * mapa2->info.width + i];
                
                if (current_value == -1 || new_value < current_value) {
                    set_occupation_value(mapa1, x_world, y_world, new_value);
                }
            }
        }
    }

    void publish_merged_map() {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (maps_.empty()) {
            RCLCPP_WARN(get_logger(), "No maps received yet");
            return;
        }
    
        // Garantir que o primeiro mapa é válido
        if (!maps_.begin()->second) {
            RCLCPP_ERROR(get_logger(), "Invalid first map!");
            return;
        }
    

        auto merged_map = create_minimum_map();
    
        for (const auto& map : maps_) {
            if (!map.second) continue;
    
            // Verificar compatibilidade de resolução
            if (std::abs(map.second->info.resolution - merged_map.info.resolution) > 1e-6) {
                RCLCPP_WARN(get_logger(), "Map resolutions differ: %f vs %f", 
                           map.second->info.resolution, merged_map.info.resolution);
                continue;
            }

            merge_map(merged_map,map.second);
        }
        // Atualizar timestamp e publicar
        merged_map.header.stamp = this->now();
        merged_map_pub_->publish(merged_map);
        // maps_.clear();
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MergeMapsNode>());
    rclcpp::shutdown();
    return 0;
}
