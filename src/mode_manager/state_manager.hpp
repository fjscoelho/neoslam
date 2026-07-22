#ifndef STATE_MANAGER_HPP
#define STATE_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <json/json.h>
#include <fstream>
#include <ctime>
#include "experience_map.h"
#include "posecell_network.h"
#include "local_view_match_with_intervals.hpp"

class StateManager {
public:
    StateManager(ExperienceMap* em, PosecellNetwork* pc, 
                 LocalViewMatchWithIntervals* lv)
        : em_(em), pc_(pc), lv_(lv) {}

    bool export_state(const std::string& filename) {
        Json::Value root;
        root["timestamp"] = static_cast<double>(std::time(nullptr));
        root["system_state_version"] = "1.0";
        
        // ExperienceMap
        if (em_) {
            Json::Value exp_map_json(Json::objectValue);
            exp_map_json["num_experiences"] = static_cast<int>(em_->get_num_experiences());
            exp_map_json["num_links"] = static_cast<int>(em_->get_num_links());
            exp_map_json["current_exp_id"] = em_->get_current_id();

            Json::Value experiences_json(Json::arrayValue);
            for (int i = 0; i < em_->get_num_experiences(); ++i) {
                const Experience* exp = em_->get_experience(i);
                Json::Value exp_json(Json::objectValue);
                exp_json["id"] = exp->id;
                exp_json["x_m"] = exp->x_m;
                exp_json["y_m"] = exp->y_m;
                exp_json["th_rad"] = exp->th_rad;
                exp_json["vt_id"] = exp->vt_id;
                exp_json["seconds"] = static_cast<int>(exp->seconds);
                exp_json["nanoseconds"] = static_cast<int>(exp->nanoseconds);
                exp_json["links_from"] = Json::Value(Json::arrayValue);
                for (const auto& link_id : exp->links_from) {
                    exp_json["links_from"].append(static_cast<int>(link_id));
                }
                exp_json["links_to"] = Json::Value(Json::arrayValue);
                for (const auto& link_id : exp->links_to) {
                    exp_json["links_to"].append(static_cast<int>(link_id));
                }
                experiences_json.append(exp_json);
            }
            exp_map_json["experiences"] = experiences_json;

            Json::Value links_json(Json::arrayValue);
            for (int i = 0; i < em_->get_num_links(); ++i) {
                const Link* link = em_->get_link(i);
                Json::Value link_json(Json::objectValue);
                link_json["exp_from_id"] = link->exp_from_id;
                link_json["exp_to_id"] = link->exp_to_id;
                link_json["d"] = link->d;
                link_json["heading_rad"] = link->heading_rad;
                link_json["facing_rad"] = link->facing_rad;
                link_json["delta_time_s"] = link->delta_time_s;
                links_json.append(link_json);
            }
            exp_map_json["links"] = links_json;
            root["experience_map"] = exp_map_json;
        }
        
        // PoseCellNetwork
        if (pc_) {
            root["pose_cells"] = pc_->serialize_to_json();
        }
        
        // SpatialViewCells
        if (lv_) {
            root["spatial_view"] = lv_->serialize_to_json();
        }
        
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Failed to open state file for writing: " << filename << std::endl;
            return false;
        }
        file << root.toStyledString();
        std::cout << "State exported to: " << filename << std::endl;
        return true;
    }

    bool import_state(const std::string& filename) {
        Json::Value root;
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Failed to open state file: " << filename << std::endl;
            return false;
        }
        file >> root;
        
        bool success = true;
        
        // ExperienceMap - você já tem o load_from_json do MapManager, então não precisa aqui
        // Apenas o PoseCell e SpatialView
        
        // PoseCellNetwork
        if (root.isMember("pose_cells") && pc_) {
            if (!pc_->deserialize_from_json(root["pose_cells"])) {
                std::cerr << "Failed to deserialize PoseCellNetwork" << std::endl;
                success = false;
            } else {
                std::cout << "PoseCellNetwork state restored" << std::endl;
            }
        }
        
        // SpatialViewCells
        if (root.isMember("spatial_view") && lv_) {
            if (!lv_->deserialize_from_json(root["spatial_view"])) {
                std::cerr << "Failed to deserialize SpatialViewCells" << std::endl;
                success = false;
            } else {
                std::cout << "SpatialViewCells state restored" << std::endl;
            }
        }
        
        return success;
    }

private:
    ExperienceMap* em_;
    PosecellNetwork* pc_;
    LocalViewMatchWithIntervals* lv_;
};

#endif