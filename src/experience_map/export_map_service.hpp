#ifndef EXPORT_MAP_SERVICE_HPP
#define EXPORT_MAP_SERVICE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>  // <-- CORRIGIDO: sem /detail/
#include <fstream>
#include <json/json.h>
#include "experience_map.h"

namespace neoslam {

class ExportMapService : public rclcpp::Node
{
public:
    ExportMapService(ExperienceMap* em_ptr) 
        : Node("export_map_service"), em_(em_ptr)
    {
        // Serviço para exportação simples (nome padrão)
        export_service_ = this->create_service<std_srvs::srv::Empty>(
            "/experience_map/export_graph",
            std::bind(&ExportMapService::export_graph_callback, 
                      this, 
                      std::placeholders::_1, 
                      std::placeholders::_2)
        );
        
        // Serviço para exportação com nome customizado via parâmetro
        export_with_name_service_ = this->create_service<std_srvs::srv::Empty>(
            "/experience_map/export_graph_named",
            std::bind(&ExportMapService::export_graph_named_callback,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2)
        );
        
        RCLCPP_INFO(this->get_logger(), "Export Map Service initialized");
        RCLCPP_INFO(this->get_logger(), "  - Call /experience_map/export_graph to export with default name");
        RCLCPP_INFO(this->get_logger(), "  - Call /experience_map/export_graph_named to export with custom name (set via parameter)");
    }

private:
    ExperienceMap* em_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr export_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr export_with_name_service_;

    // Callback para exportação padrão
    void export_graph_callback(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response)
    {
        (void)request;
        (void)response;
        
        std::string default_filename = "topological_map_" + 
                                       std::to_string(this->now().seconds()) + 
                                       ".json";
        export_graph_to_json(default_filename);
    }

    // Callback para exportação com nome customizado via parâmetro
    void export_graph_named_callback(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response)
    {
        (void)request;
        (void)response;
        
        std::string filename;
        // Declare e obtenha o parâmetro
        this->declare_parameter("export_filename", "topological_map.json");
        this->get_parameter("export_filename", filename);
        
        export_graph_to_json(filename);
    }

    // Função principal de exportação
    void export_graph_to_json(const std::string& filename)
    {
        if (em_ == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "ExperienceMap is null!");
            return;
        }

        Json::Value root;
        
        // Metadados do mapa
        root["metadata"]["frame_id"] = "map";
        root["metadata"]["timestamp"] = this->now().seconds();
        root["metadata"]["num_experiences"] = em_->get_num_experiences();
        root["metadata"]["num_links"] = em_->get_num_links();
        root["metadata"]["current_experience_id"] = em_->get_current_id();
        
        // Exportar nós (experiências)
        Json::Value nodes_array(Json::arrayValue);
        for (int i = 0; i < em_->get_num_experiences(); i++) {
            Experience* exp = em_->get_experience(i);
            if (exp == nullptr) continue;
            
            Json::Value node;
            node["id"] = exp->id;
            node["x_m"] = exp->x_m;
            node["y_m"] = exp->y_m;
            node["th_rad"] = exp->th_rad;
            node["seconds"] = exp->seconds;
            node["nanoseconds"] = exp->nanoseconds;
            
            // Lista de links que saem deste nó
            Json::Value links_from(Json::arrayValue);
            for (unsigned int link_id : exp->links_from) {
                links_from.append(link_id);
            }
            node["links_from"] = links_from;
            
            // Lista de links que chegam neste nó
            Json::Value links_to(Json::arrayValue);
            for (unsigned int link_id : exp->links_to) {
                links_to.append(link_id);
            }
            node["links_to"] = links_to;
            
            nodes_array.append(node);
        }
        root["nodes"] = nodes_array;
        
        // Exportar arestas (links)
        Json::Value edges_array(Json::arrayValue);
        for (int i = 0; i < em_->get_num_links(); i++) {
            Link* link = em_->get_link(i);
            if (link == nullptr) continue;
            
            Json::Value edge;
            edge["id"] = i;
            edge["exp_from_id"] = link->exp_from_id;
            edge["exp_to_id"] = link->exp_to_id;
            edge["d"] = link->d;
            edge["heading_rad"] = link->heading_rad;
            edge["facing_rad"] = link->facing_rad;
            edge["delta_time_s"] = link->delta_time_s;
            edges_array.append(edge);
        }
        root["edges"] = edges_array;
        
        // Exportar lista de objetivos (se houver)
        Json::Value goals_array(Json::arrayValue);
        const std::deque<int>& goals = em_->get_goals();
        for (int goal_id : goals) {
            goals_array.append(goal_id);
        }
        root["goals"] = goals_array;
        
        // Escrever arquivo
        std::ofstream file(filename);
        if (file.is_open()) {
            file << root.toStyledString();
            file.close();
            RCLCPP_INFO(this->get_logger(), "Map exported successfully to: %s", filename.c_str());
            RCLCPP_INFO(this->get_logger(), "  - Nodes: %d", em_->get_num_experiences());
            RCLCPP_INFO(this->get_logger(), "  - Edges: %d", em_->get_num_links());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", filename.c_str());
        }
    }
};

} // namespace neoslam

#endif