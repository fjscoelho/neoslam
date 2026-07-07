#include "utils.h"

#include <rclcpp/rclcpp.hpp>

#include "experience_map.h"
#include <topological_msgs/msg/topological_action.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "experience_map_scene.h"
#include <topological_msgs/msg/topological_map.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>  // <-- ESSE É O QUE FALTAVA!

// ============================================
// INCLUDES PARA OS SERVIÇOS E EXPORTAÇÃO
// ============================================
#include <std_srvs/srv/empty.hpp>
#include <json/json.h>
#include <fstream>
#include <memory>
// #include <ament_index_cpp/get_package_share_directory.hpp>  // <-- Este é o correto

// #include "export_map_service.hpp"
// #include "map_visualization_exporter.hpp"

#ifdef HAVE_IRRLICHT
#include "experience_map_scene.h"
#endif

/**
 * @brief ExperienceMapNode - Topological map building and path planning
 * 
 * This ROS2 node maintains a topological map (experience map) of the robot's trajectory
 * as a graph of experiences (nodes) connected by links (edges). It receives actions from
 * the pose cell network to create nodes, create edges, or relocalize. The map is
 * continuously refined using an iterative relaxation algorithm to correct accumulated
 * odometry drift. The node also supports goal-directed navigation by computing shortest
 * paths through the experience graph.
 * 
 * Subscriptions:
 *   - <topic_root>/odom: Receives odometry for dead reckoning between experiences
 *   - <topic_root>/PoseCell/TopologicalAction: Receives actions to update the experience map
 *   - <topic_root>/ExperienceMap/SetGoalPose: Receives goal poses for navigation
 * 
 * Publications:
 *   - <topic_root>/ExperienceMap/Map: Publishes complete topological map (periodic)
 *   - <topic_root>/ExperienceMap/MapMarker: Publishes visualization markers for RViz
 *   - <topic_root>/ExperienceMap/RobotPose: Publishes current robot pose in map frame
 *   - <topic_root>/ExperienceMap/PathToGoal: Publishes planned path to goal
 * 
 * Parameters:
 *   - topic_root: Base topic namespace for all subscriptions and publications
 *   - exp_correction: Correction rate for map relaxation algorithm (default: 0.5)
 *   - exp_loops: Number of relaxation iterations per update (default: 10)
 *   - exp_initial_em_deg: Initial heading in degrees for first experience (default: 90.0)
 *   - enable: Enable Irrlicht graphics visualization (default: true)
 *   - exp_map_size: Size of visualization window (default: 500)
 *   - media_path: Path to media resources for visualization
 *   - image_file: Image file for visualization background
 */
class ExperienceMapNode : public rclcpp::Node
{
public:
  ExperienceMapNode()
    : Node("experience_map_node")
  {
    // Declare and get parameters
    this->declare_parameter("topic_root", "");
    this->declare_parameter("exp_correction", 0.5);
    this->declare_parameter("exp_loops", 10);
    this->declare_parameter("exp_initial_em_deg", 90.0);
    this->declare_parameter("enable", true);
    this->declare_parameter("exp_map_size", 500);
    this->declare_parameter("media_path", "");
    this->declare_parameter("image_file", "");
    this->declare_parameter("export_filename", "topological_map.json");

    std::string topic_root = this->get_parameter("topic_root").as_string();
    
    // Log all parameters for debugging
    RCLCPP_INFO(this->get_logger(), "ExperienceMap Parameters:");
    RCLCPP_INFO(this->get_logger(), "  topic_root: %s", topic_root.c_str());
    RCLCPP_INFO(this->get_logger(), "  exp_correction: %f", this->get_parameter("exp_correction").as_double());
    RCLCPP_INFO(this->get_logger(), "  exp_loops: %ld", this->get_parameter("exp_loops").as_int());
    RCLCPP_INFO(this->get_logger(), "  exp_initial_em_deg: %f", this->get_parameter("exp_initial_em_deg").as_double());
    RCLCPP_INFO(this->get_logger(), "  export_filename: %s", this->get_parameter("export_filename").as_string().c_str());

    // Create ExperienceMap with parameters
    em = new ExperienceMap(
      this->get_parameter("exp_correction").as_double(),
      this->get_parameter("exp_loops").as_int(),
      this->get_parameter("exp_initial_em_deg").as_double()
    );

    // Initialize publishers
    pub_em = this->create_publisher<topological_msgs::msg::TopologicalMap>(
      topic_root + "/ExperienceMap/Map", 10);
    pub_em_markers = this->create_publisher<visualization_msgs::msg::Marker>(
      topic_root + "/ExperienceMap/MapMarker", 10);
    pub_pose = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      topic_root + "/ExperienceMap/RobotPose", 10);
    pub_goal_path = this->create_publisher<nav_msgs::msg::Path>(
      topic_root + "/ExperienceMap/PathToGoal", 10);

    // Publisher para MarkerArray (RViz)
    pub_rviz_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      topic_root + "/ExperienceMap/RVizMarkers", 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    RCLCPP_INFO(this->get_logger(), "TF broadcaster initialized");
    
    // Initialize subscribers
    sub_odometry = this->create_subscription<nav_msgs::msg::Odometry>(
      topic_root + "/odom", 10,
      std::bind(&ExperienceMapNode::odo_callback, this, std::placeholders::_1));
    
    sub_action = this->create_subscription<topological_msgs::msg::TopologicalAction>(
      topic_root + "/PoseCell/TopologicalAction", 10,
      std::bind(&ExperienceMapNode::action_callback, this, std::placeholders::_1));
    
    sub_goal = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      topic_root + "/ExperienceMap/SetGoalPose", 10,
      std::bind(&ExperienceMapNode::set_goal_pose_callback, this, std::placeholders::_1));

    // ============================================
    // CRIAR SERVIÇOS DE EXPORTAÇÃO DIRETAMENTE
    // ============================================
    
    // Serviço para exportar o mapa
    export_graph_service_ = this->create_service<std_srvs::srv::Empty>(
      "/experience_map/export_graph",
      std::bind(&ExperienceMapNode::export_graph_callback, 
                this, 
                std::placeholders::_1, 
                std::placeholders::_2)
    );
    
    // Serviço para exportar com nome customizado
    export_graph_named_service_ = this->create_service<std_srvs::srv::Empty>(
      "/experience_map/export_graph_named",
      std::bind(&ExperienceMapNode::export_graph_named_callback,
                this,
                std::placeholders::_1,
                std::placeholders::_2)
    );
    
    // Serviço para salvar marcadores RViz
    save_rviz_service_ = this->create_service<std_srvs::srv::Empty>(
      "/experience_map/save_rviz_markers",
      std::bind(&ExperienceMapNode::save_rviz_markers_callback,
                this,
                std::placeholders::_1,
                std::placeholders::_2)
    );
    
    RCLCPP_INFO(this->get_logger(), "Export services initialized");
    RCLCPP_INFO(this->get_logger(), "ExperienceMap node initialized");
    
#ifdef HAVE_IRRLICHT
    use_graphics = this->get_parameter("enable").as_bool();
    if (use_graphics)
    {
      ems = new ExperienceMapScene(
        this->get_parameter("exp_map_size").as_int(),
        this->get_parameter("media_path").as_string(),
        this->get_parameter("image_file").as_string(),
        em
      );
    }
#endif
  }
  
  ~ExperienceMapNode()
  {
    if (em != nullptr)
      delete em;
#ifdef HAVE_IRRLICHT
    if (ems != nullptr)
      delete ems; 
#endif
  }

private:
  void odo_callback(const nav_msgs::msg::Odometry::SharedPtr odo)
  {
    RCLCPP_DEBUG(this->get_logger(), "EM:odo_callback v=%f r=%f",
                 odo->twist.twist.linear.x, odo->twist.twist.angular.z);

    if (prev_time.seconds() > 0)
    {
      double time_diff = (rclcpp::Time(odo->header.stamp) - prev_time).seconds();
      em->on_odo(odo->twist.twist.linear.x, odo->twist.twist.angular.z, time_diff);
    }
    
    if (em->get_current_goal_id() >= 0)
    {
      prev_goal_update = rclcpp::Time(odo->header.stamp);
      em->calculate_path_to_goal(rclcpp::Time(odo->header.stamp).seconds());

      nav_msgs::msg::Path path;
      if (em->get_current_goal_id() >= 0)
      {
        em->get_goal_waypoint();

        geometry_msgs::msg::PoseStamped pose;
        path.header.stamp = this->now();
        path.header.frame_id = "map";

        path.poses.clear();
        unsigned int trace_exp_id = em->get_goals()[0];
        while (trace_exp_id != em->get_goal_path_final_exp())
        {
          pose.header.stamp = this->now();
          pose.header.frame_id = "map";  // <-- ADICIONAR ESTA LINHA
          pose.pose.position.x = em->get_experience(trace_exp_id)->x_m;
          pose.pose.position.y = em->get_experience(trace_exp_id)->y_m;
          path.poses.push_back(pose);

          trace_exp_id = em->get_experience(trace_exp_id)->goal_to_current;
        }

        pub_goal_path->publish(path);
      }
      else
      {
        path.header.stamp = this->now();
        path.header.frame_id = "map";
        path.poses.clear();
        pub_goal_path->publish(path);
      }
    }

    prev_time = rclcpp::Time(odo->header.stamp);
    
    std::cout << "Current Exp: " << em->get_current_id() << " | Total Exps: " << em->get_num_experiences() << " | Total actions: " << action_counter << std::endl;
    std::cout.flush();
  }

  void action_callback(const topological_msgs::msg::TopologicalAction::SharedPtr action)
  {
    action_counter++;
        
    RCLCPP_DEBUG(this->get_logger(), "EM:action_callback action=%d src=%d dst=%d",
                 action->action, action->src_id, action->dest_id);

    switch (action->action)
    {
      case topological_msgs::msg::TopologicalAction::CREATE_NODE:
        em->on_create_experience(action->dest_id, action->header.stamp.sec, action->header.stamp.nanosec);
        em->on_set_experience(action->dest_id, 0);
        break;

      case topological_msgs::msg::TopologicalAction::CREATE_EDGE:
        em->on_create_link(action->src_id, action->dest_id, action->relative_rad);
        em->on_set_experience(action->dest_id, action->relative_rad);
        break;

      case topological_msgs::msg::TopologicalAction::SET_NODE:
        em->on_set_experience(action->dest_id, action->relative_rad);
        break;
    }

    em->iterate();

    // Publicar pose do robô no frame "map"
    geometry_msgs::msg::PoseStamped pose_output;
    pose_output.header.stamp = action->header.stamp;
    pose_output.header.frame_id = "map";
    pose_output.pose.position.x = em->get_experience(em->get_current_id())->x_m;
    pose_output.pose.position.y = em->get_experience(em->get_current_id())->y_m;
    pose_output.pose.position.z = 0;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, em->get_experience(em->get_current_id())->th_rad);
    pose_output.pose.orientation = tf2::toMsg(q);
    
    pub_pose->publish(pose_output);

    // ============================================
    // Publicar transformação TF do robô no frame "map"
    // ============================================
    geometry_msgs::msg::TransformStamped tf_transform;
    tf_transform.header.stamp = action->header.stamp;
    tf_transform.header.frame_id = "map";
    tf_transform.child_frame_id = "base_link";

    Experience* current_exp = em->get_experience(em->get_current_id());
    if (current_exp != nullptr) {
        tf_transform.transform.translation.x = current_exp->x_m;
        tf_transform.transform.translation.y = current_exp->y_m;
        tf_transform.transform.translation.z = 0.0;
        
        tf2::Quaternion q_tf;
        q_tf.setRPY(0, 0, current_exp->th_rad);
        tf_transform.transform.rotation = tf2::toMsg(q_tf);
        
        tf_broadcaster_->sendTransform(tf_transform);
    }

    if ((rclcpp::Time(action->header.stamp) - prev_pub_time).seconds() > 30.0)
    {
      prev_pub_time = rclcpp::Time(action->header.stamp);

      topological_msgs::msg::TopologicalMap em_map;
      em_map.header.stamp = action->header.stamp;
      em_map.header.frame_id = "map";
      em_map.node_count = em->get_num_experiences();
      em_map.node.resize(em->get_num_experiences());
      
      for (int i = 0; i < em->get_num_experiences(); i++)
      {
        em_map.node[i].id = em->get_experience(i)->id;
        em_map.node[i].pose.pose.position.x = em->get_experience(i)->x_m;
        em_map.node[i].pose.pose.position.y = em->get_experience(i)->y_m;
        em_map.node[i].pose.header.stamp.sec = em->get_experience(i)->seconds;
        em_map.node[i].pose.header.stamp.nanosec = em->get_experience(i)->nanoseconds;
        em_map.node[i].pose.header.frame_id = "map";
        tf2::Quaternion q;
        q.setRPY(0, 0, em->get_experience(i)->th_rad);
        em_map.node[i].pose.pose.orientation = tf2::toMsg(q);
      }

      em_map.edge_count = em->get_num_links();
      em_map.edge.resize(em->get_num_links());
      
      for (int i = 0; i < em->get_num_links(); i++)
      {
        em_map.edge[i].id = i;
        em_map.edge[i].source_id = em->get_link(i)->exp_from_id;
        em_map.edge[i].destination_id = em->get_link(i)->exp_to_id;
        em_map.edge[i].duration = rclcpp::Duration::from_seconds(em->get_link(i)->delta_time_s);
        em_map.edge[i].transform.translation.x = em->get_link(i)->d * cos(em->get_link(i)->heading_rad);
        em_map.edge[i].transform.translation.y = em->get_link(i)->d * sin(em->get_link(i)->heading_rad);
        
        tf2::Quaternion q;
        q.setRPY(0, 0, em->get_link(i)->facing_rad);
        em_map.edge[i].transform.rotation = tf2::toMsg(q);
      }
      
      pub_em->publish(em_map);
    }

    visualization_msgs::msg::Marker em_marker;
    em_marker.header.stamp = this->now();
    em_marker.header.frame_id = "map";
    em_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    em_marker.points.resize(em->get_num_links() * 2);
    em_marker.action = visualization_msgs::msg::Marker::ADD;
    em_marker.scale.x = 0.01;
    em_marker.color.a = 1;
    em_marker.ns = "em";
    em_marker.id = 0;
    em_marker.pose.orientation.w = 1;
    
    for (int i = 0; i < em->get_num_links(); i++)
    {
      em_marker.points[i * 2].x = em->get_experience(em->get_link(i)->exp_from_id)->x_m;
      em_marker.points[i * 2].y = em->get_experience(em->get_link(i)->exp_from_id)->y_m;
      em_marker.points[i * 2].z = 0;
      em_marker.points[i * 2 + 1].x = em->get_experience(em->get_link(i)->exp_to_id)->x_m;
      em_marker.points[i * 2 + 1].y = em->get_experience(em->get_link(i)->exp_to_id)->y_m;
      em_marker.points[i * 2 + 1].z = 0;
    }

    pub_em_markers->publish(em_marker);

    // Publicar marcadores no formato MarkerArray para RViz
    visualization_msgs::msg::MarkerArray rviz_marker_array;
    
    // 1. Nós como esferas verdes
    visualization_msgs::msg::Marker nodes_marker;
    nodes_marker.header.frame_id = "map";
    nodes_marker.header.stamp = this->now();
    nodes_marker.ns = "nodes";
    nodes_marker.id = 0;
    nodes_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    nodes_marker.action = visualization_msgs::msg::Marker::ADD;
    nodes_marker.scale.x = 0.05;
    nodes_marker.scale.y = 0.05;
    nodes_marker.scale.z = 0.05;
    nodes_marker.color.r = 0.0;
    nodes_marker.color.g = 1.0;
    nodes_marker.color.b = 0.0;
    nodes_marker.color.a = 1.0;
    
    for (int i = 0; i < em->get_num_experiences(); i++) {
        Experience* exp = em->get_experience(i);
        if (exp == nullptr) continue;
        
        geometry_msgs::msg::Point p;
        p.x = exp->x_m;
        p.y = exp->y_m;
        p.z = 0.0;
        nodes_marker.points.push_back(p);
    }
    rviz_marker_array.markers.push_back(nodes_marker);
    
    // 2. Arestas como linhas brancas
    visualization_msgs::msg::Marker edges_marker;
    edges_marker.header.frame_id = "map";
    edges_marker.header.stamp = this->now();
    edges_marker.ns = "edges";
    edges_marker.id = 1;
    edges_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    edges_marker.action = visualization_msgs::msg::Marker::ADD;
    edges_marker.scale.x = 0.01;
    edges_marker.color.r = 1.0;
    edges_marker.color.g = 1.0;
    edges_marker.color.b = 1.0;
    edges_marker.color.a = 1.0;
    
    for (int i = 0; i < em->get_num_links(); i++) {
        Link* link = em->get_link(i);
        if (link == nullptr) continue;
        
        Experience* from = em->get_experience(link->exp_from_id);
        Experience* to = em->get_experience(link->exp_to_id);
        if (from == nullptr || to == nullptr) continue;
        
        geometry_msgs::msg::Point p1, p2;
        p1.x = from->x_m;
        p1.y = from->y_m;
        p1.z = 0.0;
        p2.x = to->x_m;
        p2.y = to->y_m;
        p2.z = 0.0;
        
        edges_marker.points.push_back(p1);
        edges_marker.points.push_back(p2);
    }
    rviz_marker_array.markers.push_back(edges_marker);
    
    // 3. Nó atual destacado em vermelho
    visualization_msgs::msg::Marker current_marker;
    current_marker.header.frame_id = "map";
    current_marker.header.stamp = this->now();
    current_marker.ns = "current";
    current_marker.id = 2;
    current_marker.type = visualization_msgs::msg::Marker::SPHERE;
    current_marker.action = visualization_msgs::msg::Marker::ADD;
    current_marker.scale.x = 0.15;
    current_marker.scale.y = 0.15;
    current_marker.scale.z = 0.15;
    current_marker.color.r = 1.0;
    current_marker.color.g = 0.0;
    current_marker.color.b = 0.0;
    current_marker.color.a = 1.0;
    
    Experience* current = em->get_experience(em->get_current_id());
    if (current != nullptr) {
        current_marker.pose.position.x = current->x_m;
        current_marker.pose.position.y = current->y_m;
        current_marker.pose.position.z = 0.0;
        rviz_marker_array.markers.push_back(current_marker);
    }
    
    // Publica o MarkerArray
    pub_rviz_markers_->publish(rviz_marker_array);

#ifdef HAVE_IRRLICHT
    if (use_graphics)
    {
      ems->update_scene();
      ems->draw_all();
    }
#endif
  }

  void set_goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
  {
    if (pose->header.frame_id == "map") {
      em->add_goal(pose->pose.position.x, pose->pose.position.y);
    } else {
      RCLCPP_WARN(this->get_logger(), "Goal pose frame_id is '%s', expected 'map'", 
                  pose->header.frame_id.c_str());
      em->add_goal(pose->pose.position.x, pose->pose.position.y);
    }
  }

  // ============================================
  // CALLBACKS DOS SERVIÇOS DE EXPORTAÇÃO
  // ============================================
  
  void export_graph_callback(
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response)
  {
    (void)request;
    (void)response;
    
    if (em == nullptr) {
      RCLCPP_ERROR(this->get_logger(), "ExperienceMap is null!");
      return;
    }
    
    std::string filename = "topological_map_" + 
                           std::to_string(this->now().seconds()) + 
                           ".json";
    export_graph_to_json(filename);
  }
  
  void export_graph_named_callback(
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response)
  {
    (void)request;
    (void)response;
    
    if (em == nullptr) {
      RCLCPP_ERROR(this->get_logger(), "ExperienceMap is null!");
      return;
    }
    
    std::string filename = this->get_parameter("export_filename").as_string();
    export_graph_to_json(filename);
  }
  
  void save_rviz_markers_callback(
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response)
  {
    (void)request;
    (void)response;
    
    if (em == nullptr) {
      RCLCPP_ERROR(this->get_logger(), "ExperienceMap is null!");
      return;
    }
    
    std::string filename = "topological_map_rviz_" + 
                           std::to_string(this->now().seconds()) + 
                           ".json";
    save_rviz_markers_to_json(filename);
  }
  
  void export_graph_to_json(const std::string& filename)
  {
    if (em == nullptr) {
      RCLCPP_ERROR(this->get_logger(), "ExperienceMap is null!");
      return;
    }

    Json::Value root;
    
    root["metadata"]["timestamp"] = this->now().seconds();
    root["metadata"]["num_experiences"] = em->get_num_experiences();
    root["metadata"]["num_links"] = em->get_num_links();
    root["metadata"]["current_experience_id"] = em->get_current_id();
    
    Json::Value nodes_array(Json::arrayValue);
    for (int i = 0; i < em->get_num_experiences(); i++) {
      Experience* exp = em->get_experience(i);
      if (exp == nullptr) continue;
      
      Json::Value node;
      node["id"] = exp->id;
      node["x_m"] = exp->x_m;
      node["y_m"] = exp->y_m;
      node["th_rad"] = exp->th_rad;
      node["seconds"] = exp->seconds;
      node["nanoseconds"] = exp->nanoseconds;
      nodes_array.append(node);
    }
    root["nodes"] = nodes_array;
    
    Json::Value edges_array(Json::arrayValue);
    for (int i = 0; i < em->get_num_links(); i++) {
      Link* link = em->get_link(i);
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
    
    std::ofstream file(filename);
    if (file.is_open()) {
      file << root.toStyledString();
      file.close();
      RCLCPP_INFO(this->get_logger(), "Map exported successfully to: %s", filename.c_str());
      RCLCPP_INFO(this->get_logger(), "  - Nodes: %d", em->get_num_experiences());
      RCLCPP_INFO(this->get_logger(), "  - Edges: %d", em->get_num_links());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", filename.c_str());
    }
  }
  
  void save_rviz_markers_to_json(const std::string& filename)
  {
    if (em == nullptr) {
      RCLCPP_ERROR(this->get_logger(), "ExperienceMap is null!");
      return;
    }

    Json::Value root;
    root["type"] = "topological_map";
    root["timestamp"] = this->now().seconds();
    
    Json::Value nodes_array(Json::arrayValue);
    Json::Value edges_array(Json::arrayValue);
    
    for (int i = 0; i < em->get_num_experiences(); i++) {
      Experience* exp = em->get_experience(i);
      if (exp == nullptr) continue;
      
      Json::Value node;
      node["id"] = exp->id;
      node["x"] = exp->x_m;
      node["y"] = exp->y_m;
      node["theta"] = exp->th_rad;
      nodes_array.append(node);
    }
    root["nodes"] = nodes_array;
    
    for (int i = 0; i < em->get_num_links(); i++) {
      Link* link = em->get_link(i);
      if (link == nullptr) continue;
      
      Json::Value edge;
      edge["from"] = link->exp_from_id;
      edge["to"] = link->exp_to_id;
      edge["distance"] = link->d;
      edges_array.append(edge);
    }
    root["edges"] = edges_array;
    
    std::ofstream file(filename);
    if (file.is_open()) {
      file << root.toStyledString();
      file.close();
      RCLCPP_INFO(this->get_logger(), "RViz markers saved to: %s", filename.c_str());
      RCLCPP_INFO(this->get_logger(), "  - Nodes: %d", em->get_num_experiences());
      RCLCPP_INFO(this->get_logger(), "  - Edges: %d", em->get_num_links());
      
      // Gerar arquivo de configuração RViz
      generate_rviz_config();
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to save RViz markers to: %s", filename.c_str());
    }
  }
  
  void generate_rviz_config()
  {
    std::string config_filename = "topological_map_viewer.rviz";
    std::ofstream config(config_filename);
    if (config.is_open()) {
      config << "Panels:\n"
             << "  - Class: rviz_common/Displays\n"
             << "    Name: Displays\n"
             << "    Position:\n"
             << "      x: 50\n"
             << "      y: 50\n"
             << "    Width: 400\n"
             << "    Height: 600\n"
             << "Visualization Manager:\n"
             << "  Class: \"\"\n"
             << "  Displays:\n"
             << "    - Alpha: 0.5\n"
             << "      Cell Size: 1\n"
             << "      Class: rviz_default_plugins/Grid\n"
             << "      Color: 160; 160; 164\n"
             << "      Enabled: true\n"
             << "      Name: Grid\n"
             << "      Normal Cell Count: 0\n"
             << "      Offset:\n"
             << "        x: 0\n"
             << "        y: 0\n"
             << "        z: 0\n"
             << "      Plane: XY\n"
             << "      Plane Cell Count: 10\n"
             << "      Reference Frame: map\n"
             << "      Value: true\n"
             << "    - Alpha: 1\n"
             << "      Class: rviz_default_plugins/MarkerArray\n"
             << "      Enabled: true\n"
             << "      Name: Topological Map\n"
             << "      Namespaces:\n"
             << "        nodes: true\n"
             << "        edges: true\n"
             << "        current: true\n"
             << "      Queue Size: 10\n"
             << "      Topic: /robotarium/ExperienceMap/RVizMarkers\n"
             << "      Value: true\n"
             << "  Enabled: true\n"
             << "  Global Options:\n"
             << "    Background Color: 48; 48; 48\n"
             << "    Fixed Frame: map\n"
             << "    Frame Rate: 30\n"
             << "  Name: root\n"
             << "  Tools:\n"
             << "    - Class: rviz_default_plugins/Interact\n"
             << "      Hide Inactive Objects: true\n"
             << "    - Class: rviz_default_plugins/MoveCamera\n"
             << "    - Class: rviz_default_plugins/Select\n"
             << "    - Class: rviz_default_plugins/FocusCamera\n"
             << "    - Class: rviz_default_plugins/Measure\n"
             << "  Value: true\n"
             << "  Views:\n"
             << "    Current:\n"
             << "      Class: rviz_default_plugins/Orbit\n"
             << "      Distance: 10\n"
             << "      Enable Stereo Rendering: false\n"
             << "      Stereo Eye Separation: 0.059999999999999998\n"
             << "      Stereo Focal Distance: 1\n"
             << "      Swap Stereo Eyes: false\n"
             << "      Value: Orbit (rviz)\n"
             << "      Focal Point:\n"
             << "        x: 0\n"
             << "        y: 0\n"
             << "        z: 0\n"
             << "      Focal Shape Fixed: true\n"
             << "      Focal Shape Size: 0.050000000000000003\n"
             << "      Invert Z Axis: false\n"
             << "      Name: Current View\n"
             << "      Near Clip Distance: 0.0099999999999999998\n"
             << "      Pitch: 0.78539816339744828\n"
             << "      Target Frame: map\n"
             << "      Value: Orbit (rviz)\n"
             << "      Yaw: 0.78539816339744828\n"
             << "    Saved: ~\n";
      config.close();
      RCLCPP_INFO(this->get_logger(), "RViz config generated: %s", config_filename.c_str());
      RCLCPP_INFO(this->get_logger(), "Run: rviz2 -d %s", config_filename.c_str());
    }
  }

  // ============================================================
  // MEMBROS PRIVADOS
  // ============================================================
  
  // Experience Map principal
  ExperienceMap* em = nullptr;  // <-- APENAS UMA VEZ!
  
  // Publicadores
  rclcpp::Publisher<topological_msgs::msg::TopologicalMap>::SharedPtr pub_em;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_em_markers;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_goal_path;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_rviz_markers_;
  
  // Subscritores
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry;
  rclcpp::Subscription<topological_msgs::msg::TopologicalAction>::SharedPtr sub_action;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal;
  
  // Timers e contadores
  rclcpp::Time prev_time{0, 0, RCL_ROS_TIME};
  rclcpp::Time prev_goal_update{0, 0, RCL_ROS_TIME};
  rclcpp::Time prev_pub_time{0, 0, RCL_ROS_TIME};
  int action_counter = 0;

  // ============================================
  // SERVIÇOS DE EXPORTAÇÃO
  // ============================================
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr export_graph_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr export_graph_named_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_rviz_service_;
  
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
#ifdef HAVE_IRRLICHT
  ExperienceMapScene *ems = nullptr;
  bool use_graphics;
#endif
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExperienceMapNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}