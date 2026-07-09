#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/string.hpp>

class ModeManager : public rclcpp::Node
{
public:
  ModeManager()
    : Node("mode_manager")
  {
    // Parâmetro para modo inicial
    this->declare_parameter("initial_mode", "mapping");
    std::string initial_mode = this->get_parameter("initial_mode").as_string();
    current_mode_ = initial_mode;
    
    // Publisher para tópico
    mode_publisher_ = this->create_publisher<std_msgs::msg::String>(
      "/system_mode", 10);
    
    // Serviço para mudar modo
    change_service_ = this->create_service<std_srvs::srv::SetBool>(
      "change_mode",
      [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
             std_srvs::srv::SetBool::Response::SharedPtr res) {
        if (req->data) {
          current_mode_ = "navigation";
        } else {
          current_mode_ = "mapping";
        }
        
        auto msg = std_msgs::msg::String();
        msg.data = current_mode_;
        mode_publisher_->publish(msg);
        
        res->success = true;
        res->message = "Mode changed to: " + current_mode_;
        
        RCLCPP_INFO(this->get_logger(), "🔀 Mode changed to: %s", current_mode_.c_str());
      });
    
    // Publica o modo inicial
    auto msg = std_msgs::msg::String();
    msg.data = current_mode_;
    mode_publisher_->publish(msg);
    
    RCLCPP_INFO(this->get_logger(), "🚀 Mode Manager started in mode: %s", current_mode_.c_str());
    RCLCPP_INFO(this->get_logger(), "📡 Service: /mode_manager/change_mode");
    RCLCPP_INFO(this->get_logger(), "📡 Topic: /system_mode");
  }

private:
  std::string current_mode_;
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