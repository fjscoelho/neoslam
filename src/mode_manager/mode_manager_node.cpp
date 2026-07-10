#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/string.hpp>
#include "mode_manager/mode_globals.h"  // <-- NOVO INCLUDE

class ModeManager : public rclcpp::Node
{
public:
  ModeManager()
    : Node("mode_manager")
  {
    RCLCPP_INFO(this->get_logger(), "🚀 Mode Manager node starting...");
    
    // Parâmetro para modo inicial
    this->declare_parameter("initial_mode", "mapping");
    std::string initial_mode = this->get_parameter("initial_mode").as_string();
    
    // Atualiza o global com o modo inicial
    ModeGlobals::getInstance().setMode(initial_mode);
    
    RCLCPP_INFO(this->get_logger(), "📝 Initial mode: %s", initial_mode.c_str());
    
    // Publisher para tópico
    mode_publisher_ = this->create_publisher<std_msgs::msg::String>(
      "/system_mode", 10);
    
    RCLCPP_INFO(this->get_logger(), "📡 Publisher created for /system_mode");
    
    // Serviço para mudar modo
    change_service_ = this->create_service<std_srvs::srv::SetBool>(
      "change_mode",
      [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
             std_srvs::srv::SetBool::Response::SharedPtr res) {
        
        std::string new_mode = req->data ? "navigation" : "mapping";
        
        // Atualiza o global
        ModeGlobals::getInstance().setMode(new_mode);
        
        // Publica no tópico
        auto msg = std_msgs::msg::String();
        msg.data = new_mode;
        mode_publisher_->publish(msg);
        
        res->success = true;
        res->message = "Mode changed to: " + new_mode;
        
        RCLCPP_INFO(this->get_logger(), "🔀 Mode changed to: %s", new_mode.c_str());
      });
    
    RCLCPP_INFO(this->get_logger(), "📡 Service created: /change_mode");
    
    // Publica o modo inicial
    auto msg = std_msgs::msg::String();
    msg.data = initial_mode;
    mode_publisher_->publish(msg);
    
    RCLCPP_INFO(this->get_logger(), "✅ Mode Manager fully initialized");
    RCLCPP_INFO(this->get_logger(), "📡 Service: /change_mode");
    RCLCPP_INFO(this->get_logger(), "📡 Topic: /system_mode");
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_publisher_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr change_service_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ModeManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}