#pragma once
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/Dense>
#include <roaring/roaring.hh>
#include <vector>
#include <memory>
#include <chrono>
#include <iostream>
#include <algorithm>
#include <json/json.h>

/**
 * @brief TemplateInterval groups temporally close and similar visual templates
 * 
 * Reduces comparisons by grouping similar images into contiguous intervals.
 */
class TemplateInterval {
public:
    std::pair<int, int> init_end;          // [start, end] image indices
    Roaring accumulated_features;           // Union of all features in the interval
    
    TemplateInterval() : init_end{0, 0} {}
};


class LocalViewMatchWithIntervals {
public:
    /**
     * @brief Constructor - Spatial View Matching with Temporal Intervals
     * 
     * @param theta_alpha_ Similarity threshold to extend intervals
     * @param theta_rho_ Maximum duration of an interval (in images)
     * @param score_interval_ Score threshold for loop closure detection
     * @param exclude_recent_intervals_ Number of recent intervals to exclude from loop closure detection
     */
    LocalViewMatchWithIntervals(int theta_alpha_, int theta_rho_, int score_interval_, 
                                int exclude_recent_intervals_ = 3)
        : theta_alpha(theta_alpha_), 
          theta_rho(theta_rho_), 
          score_interval(score_interval_),
          exclude_recent_intervals(exclude_recent_intervals_),
          is_initialized_(false),  // Has not yet processed first image
          n_interval(0), 
          prev_interval(0),
          current_template_id(-1),
          next_template_id(0)  // Counter to generate new IDs
    {
        // 
    }

    /**
     * @brief Create a new Visual Template ID (unique place)
     */
    int create_template_id() {
        int new_id = next_template_id++;
        visual_template_ids.push_back(new_id);
        return new_id;
    }

    /**
     * @brief Main function - processes a new image and detects loop closures
     * 
     * SIMPLIFIED INTERFACE: Returns int ID directly!
     * 
     * PIPELINE:
     * 1. Interval management (create/extend)
     * 2. Incremental construction of the interval map
     * 3. Loop closure detection (comparison with all intervals)
     * 4. Decision: create new visual template or reuse existing one
     * 
     * @param feature Roaring Bitmap with active features (512 bits)
     * @param n_image Current image index (counter)
     * @return std::pair<Eigen::MatrixXd, int> - (scores, template_id)
     *         - scores: vector with similarities for each interval
     *         - template_id: ID of the recognized or created visual template (int)
     */
    std::pair<Eigen::MatrixXd, int> on_image_map(const Roaring& feature, int n_image) {
        auto overall_start = std::chrono::high_resolution_clock::now();
        
        // ==============================================================
        // INITIALIZATION: First image (executed only once)
        // ==============================================================
        if (!is_initialized_) {
            initialize_first_template(feature, n_image);
            is_initialized_ = true;
        }
        
        // ==============================================================
        // STEP 1: Information about received feature
        // ==============================================================
        int template_id;  // ID do visual template (simplificado!)
        // std::cout << "n_image: " << n_image << std::endl;
        // std::cout << "  [Roaring] Feature has " << feature.cardinality() << " active bits" << std::endl;
        
        // ==============================================================
        // STEP 2: Temporal Interval Management
        // Groups similar images to reduce comparisons
        // ==============================================================
        
        // Calculate similarity with current interval
        Roaring& anchor = interval_list[n_interval]->accumulated_features;
        int alpha = (anchor & feature).cardinality();  // Bitwise intersection
        
        // std::cout << "  [Similarity] alpha=" << alpha 
        //           << " (threshold=" << theta_alpha << ")" << std::endl;
        
        // Calculate current interval duration
        int interval_duration = interval_list[n_interval]->init_end.second 
                              - interval_list[n_interval]->init_end.first;
        
        // Decision: extend or create new?
        if ((alpha >= theta_alpha) && (interval_duration < theta_rho)) {
            // ==========================================
            // Extends current interval (similar image)
            // ==========================================
            interval_list[n_interval]->init_end.second += 1;
            interval_list[n_interval]->accumulated_features |= feature;  // Union (OR)
            
            // std::cout << "  [Interval] EXTENDED interval " << n_interval 
            //           << " (duration=" << (interval_duration + 1) << ")" << std::endl;
        } else {
            // ==========================================
            // Creates new interval (dissimilar or too long)
            // ==========================================
            current_interval = std::make_shared<TemplateInterval>();
            current_interval->init_end = {n_image, n_image};
            current_interval->accumulated_features = feature;
            interval_list.push_back(current_interval);
            n_interval += 1;
            
            std::string reason = (alpha < theta_alpha) ? "low_similarity" : "max_duration";
            // std::cout << "  [Interval] Created NEW interval " << n_interval 
            //           << " (reason=" << reason << ")" << std::endl;
        }
        
        // std::cout << "  [State] n_interval=" << n_interval 
        //           << ", total_intervals=" << interval_list.size() << std::endl;
        
        // ==============================================================
        // STEP 3: Incremental Interval Map Construction
        // Uses vector of Roaring Bitmaps (extremely efficient!)
        // ==============================================================
        auto map_build_start = std::chrono::high_resolution_clock::now();
        
        // std::cout << "  [Map] intervals_feature_map.size=" << intervals_feature_map.size() << std::endl;
        
        if (prev_interval != n_interval) {
            // New interval: add new row to map
            intervals_feature_map.push_back(interval_list[n_interval]->accumulated_features);
            
            // std::cout << "  [Map] NEW interval row added (n_interval=" << n_interval 
            //           << ", total_rows=" << intervals_feature_map.size() 
            //           << ", active_bits=" << intervals_feature_map.back().cardinality() << ")" << std::endl;
        } else {
            // Same interval: update last row with accumulated features
            if (!intervals_feature_map.empty()) {
                intervals_feature_map.back() = interval_list[n_interval]->accumulated_features;
                
                // std::cout << "  [Map] SAME interval updated (n_interval=" << n_interval 
                //           << ", active_bits=" << intervals_feature_map.back().cardinality() << ")" << std::endl;
            }
        }
        
        auto map_build_end = std::chrono::high_resolution_clock::now();
        auto map_build_duration = std::chrono::duration_cast<std::chrono::microseconds>(
            map_build_end - map_build_start);
        // std::cout << "  [TIMING] Map rebuild: " << map_build_duration.count() / 1000.0 << " ms" << std::endl;
        
        // ==============================================================
        // STEP 4: Loop Closure Detection
        // Calculate similarity with ALL past intervals
        // ==============================================================
        auto loop_closure_start = std::chrono::high_resolution_clock::now();
        
        // std::cout << "  [Loop Closure] Computing similarity with " 
        //           << intervals_feature_map.size() << " intervals..." << std::endl;
        
        // Calculate similarity scores for each interval
        int num_intervals = intervals_feature_map.size();
        Eigen::VectorXi similarity_scores(num_intervals);
        
        for (int i = 0; i < num_intervals; ++i) {
            similarity_scores(i) = (intervals_feature_map[i] & feature).cardinality();
        }
        
        auto loop_closure_end = std::chrono::high_resolution_clock::now();
        auto loop_closure_duration = std::chrono::duration_cast<std::chrono::microseconds>(
            loop_closure_end - loop_closure_start);
        // std::cout << "  [TIMING] Loop closure computation: " 
        //           << loop_closure_duration.count() / 1000.0 << " ms" << std::endl;
        
        // ==============================================================
        // STEP 5: Convert scores to return format (MatrixXd)
        // ==============================================================
        Eigen::MatrixXd interval_map = similarity_scores.cast<double>();
        
        // ==============================================================
        // STEP 6: Loop Closure Candidate Filtering
        // Exclude last N intervals (too recent, avoids false positives)
        // ==============================================================
        Eigen::Index n = similarity_scores.size();
        Eigen::Index block_rows = n > exclude_recent_intervals ? n - exclude_recent_intervals : 0;
        Eigen::VectorXi scores_for_matching;
        
        if (block_rows > 0) {
            scores_for_matching = similarity_scores.head(block_rows);
        } else {
            scores_for_matching = Eigen::VectorXi();  // Vazio
        }
        
        // ==============================================================
        // STEP 7: Decision - Create New Template or Reuse Existing
        // ==============================================================
        
        // ==========================================
        // Busca por loop closures
        // ==========================================
        std::vector<int> matched_intervals;
        for (int i = 0; i < scores_for_matching.size(); ++i) {
            if (scores_for_matching(i) > score_interval) {
                matched_intervals.push_back(i);
            }
        }
        
        if (matched_intervals.empty()) {
            // ==========================================
            // No match: new place
            // ==========================================
            if (prev_interval != n_interval) {
                // New interval: create new template
                template_id = create_template_id();
                std::cout << "  [Visual Template] Created NEW template (id=" 
                          << template_id << ", no_match)" << std::endl;
            } else {
                // Mesmo intervalo: reutiliza template anterior
                template_id = current_template_id;
                std::cout << "  [Visual Template] Reusing previous template (id=" 
                          << template_id << ", same_interval)" << std::endl;
            }
        } else {
            // ==========================================
            // LOOP CLOSURE DETECTED!
            // Reuse template from most similar interval
            // ==========================================
            
            // Find interval with highest score
            int best_interval_idx = std::distance(
                scores_for_matching.data(), 
                std::max_element(scores_for_matching.data(), 
                               scores_for_matching.data() + scores_for_matching.size())
            );
            
            template_id = interval_to_template_map[best_interval_idx];
            
            std::cout << "  [Visual Template] 🧭 LOOP CLOSURE! Reusing template " 
                      << template_id 
                      << " (interval=" << best_interval_idx 
                      << ", similarity=" << scores_for_matching(best_interval_idx) 
                      << ")" << std::endl;
        }
        
        // ==============================================================
        // STEP 8: State Update for Next Iteration
        // ==============================================================
        if (prev_interval != n_interval) {
            interval_to_template_map.push_back(template_id);
        }
        current_template_id = template_id;
        prev_interval = n_interval;
        
        // ==============================================================
        // STEP 9: Finalization and Metrics
        // ==============================================================
        auto overall_end = std::chrono::high_resolution_clock::now();
        auto overall_duration = std::chrono::duration_cast<std::chrono::microseconds>(
            overall_end - overall_start);
        // std::cout << "  [TIMING] Total on_image_map: " 
        //           << overall_duration.count() / 1000.0 << " ms" << std::endl;
        
        // Return: (similarity_scores, template_id)
        return std::make_pair(interval_map, template_id);
    }

    /**
     * @brief Calculate similarity scores for a specific feature
        * @param feature Roaring Bitmap with active features
        * @return Eigen::VectorXi with similarity scores for each interval
     */
    Eigen::VectorXi get_interval_similarity_scores(const Roaring& feature) const {
        int num_intervals = intervals_feature_map.size();
        Eigen::VectorXi scores(num_intervals);
        for (int i = 0; i < num_intervals; ++i) {
            scores(i) = (intervals_feature_map[i] & feature).cardinality();
        }
        return scores;
    }

    Json::Value serialize_to_json() const;
    bool deserialize_from_json(const Json::Value& root);

private:
    // ==============================================================
    // Configuration Parameters
    // ==============================================================
    int theta_alpha;       // Similarity threshold for intervals
    int theta_rho;         // Maximum interval duration
    int score_interval;    // Threshold for loop closure detection
    int exclude_recent_intervals;  // Number of recent intervals to exclude
    
    // ==============================================================
    // Internal State
    // ==============================================================
    bool is_initialized_;      // Initialization flag (first image processed?)
    int n_interval;            // Current interval index
    int prev_interval;         // Previous interval index
    int current_template_id;   // Current visual template ID
    int next_template_id;      // Counter to generate new IDs
    
    // ==============================================================
    // Data Structures
    // ==============================================================
    
    // List of visual template IDs (unique places)
    std::vector<int> visual_template_ids;
    
    // List of temporal intervals
    std::vector<std::shared_ptr<TemplateInterval>> interval_list;
    
    // Mapping: interval_id → visual_template_id
    std::vector<int> interval_to_template_map;
    
    // Pointer to current interval
    std::shared_ptr<TemplateInterval> current_interval;
    
    // Each element represents accumulated features of an interval
    std::vector<Roaring> intervals_feature_map;
    
    /**
     * @brief Initialize structures with the first image
     * Called automatically on the first execution of on_image_map
     */
    void initialize_first_template(const Roaring& feature, int n_image) {
        // Create first interval
        current_interval = std::make_shared<TemplateInterval>();
        current_interval->init_end = {n_image, n_image};
        current_interval->accumulated_features = feature;
        interval_list.push_back(current_interval);
        
        // Create first visual template
        int first_template_id = create_template_id();
        interval_to_template_map.push_back(first_template_id);
        current_template_id = first_template_id;
    }
};

Json::Value LocalViewMatchWithIntervals::serialize_to_json() const {
    Json::Value root;
    
    // Parâmetros
    root["theta_alpha"] = theta_alpha;
    root["theta_rho"] = theta_rho;
    root["score_interval"] = score_interval;
    root["exclude_recent_intervals"] = exclude_recent_intervals;
    
    // Visual template IDs
    Json::Value vt_ids(Json::arrayValue);
    for (int id : visual_template_ids) {
        vt_ids.append(id);
    }
    root["visual_template_ids"] = vt_ids;
    
    // Intervalos
    Json::Value intervals(Json::arrayValue);
    for (const auto& interval_ptr : interval_list) {
        Json::Value int_json;
        int_json["start"] = interval_ptr->init_end.first;
        int_json["end"] = interval_ptr->init_end.second;
        // Serializar Roaring bitmap (como array de uint32)
        const size_t cardinality = interval_ptr->accumulated_features.cardinality();
        std::vector<uint32_t> words(cardinality);
        interval_ptr->accumulated_features.toUint32Array(words.data());
        Json::Value words_json(Json::arrayValue);
        for (uint32_t w : words) {
            words_json.append(w);
        }
        int_json["features"] = words_json;
        intervals.append(int_json);
    }
    root["intervals"] = intervals;
    
    // Mapeamento intervalo -> template
    Json::Value map(Json::arrayValue);
    for (int id : interval_to_template_map) {
        map.append(id);
    }
    root["interval_to_template_map"] = map;
    
    // Estado
    root["is_initialized"] = is_initialized_;
    root["n_interval"] = n_interval;
    root["prev_interval"] = prev_interval;
    root["current_template_id"] = current_template_id;
    root["next_template_id"] = next_template_id;
    
    return root;
}

bool LocalViewMatchWithIntervals::deserialize_from_json(const Json::Value& root) {
    // Verificar parâmetros (opcional, mas bom para consistência)
    if (root["theta_alpha"].asInt() != theta_alpha ||
        root["theta_rho"].asInt() != theta_rho ||
        root["score_interval"].asInt() != score_interval ||
        root["exclude_recent_intervals"].asInt() != exclude_recent_intervals) {
        std::cerr << "[SpatialViewCells] Parameter mismatch in saved state!" << std::endl;
        return false;
    }
    
    // Restaurar visual template IDs
    visual_template_ids.clear();
    for (const auto& id_json : root["visual_template_ids"]) {
        visual_template_ids.push_back(id_json.asInt());
    }
    
    // Restaurar intervalos
    interval_list.clear();
    const auto& intervals_json = root["intervals"];
    for (const auto& int_json : intervals_json) {
        auto interval = std::make_shared<TemplateInterval>();
        interval->init_end.first = int_json["start"].asInt();
        interval->init_end.second = int_json["end"].asInt();
        // Restaurar Roaring bitmap
        const auto& words_json = int_json["features"];
        std::vector<uint32_t> words;
        for (const auto& w : words_json) {
            words.push_back(w.asUInt());
        }
        interval->accumulated_features = Roaring(words.size(), words.data());
        interval_list.push_back(interval);
    }

    // Reconstruir intervals_feature_map a partir dos intervalos carregados
    intervals_feature_map.clear();
    for (const auto& interval_ptr : interval_list) {
        intervals_feature_map.push_back(interval_ptr->accumulated_features);
    }

    // Restaurar current_interval apontando para o intervalo atual (índice n_interval)
    if (!interval_list.empty() && n_interval >= 0 && n_interval < (int)interval_list.size()) {
        current_interval = interval_list[n_interval];
    } else {
        current_interval.reset();
        RCLCPP_WARN(rclcpp::get_logger("SpatialViewCells"), 
                    "n_interval (%d) fora dos limites dos intervalos carregados", n_interval);
    }
    
    // Restaurar mapeamento intervalo -> template
    interval_to_template_map.clear();
    for (const auto& id_json : root["interval_to_template_map"]) {
        interval_to_template_map.push_back(id_json.asInt());
    }
    
    // Restaurar estado
    is_initialized_ = root["is_initialized"].asBool();
    n_interval = root["n_interval"].asInt();
    prev_interval = root["prev_interval"].asInt();
    current_template_id = root["current_template_id"].asInt();
    next_template_id = root["next_template_id"].asInt();
    
    std::cout << "[SpatialViewCells] Deserialized: " 
              << interval_list.size() << " intervals, "
              << visual_template_ids.size() << " template IDs" << std::endl;
    
    return true;
}