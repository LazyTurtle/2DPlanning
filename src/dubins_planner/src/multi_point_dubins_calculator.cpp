#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "dubins_planner_msgs/srv/multi_point_dubins_planning.hpp"
#include "dubins_planner_msgs/srv/dubins_planning.hpp"

using namespace std::chrono_literals;

class MultiPointDubinsCalculator : public rclcpp::Node
{
  public:
    MultiPointDubinsCalculator()
    : Node("multi_point_dubins_calculator")
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initializing multi point dubins calculator...");
      init();
    }

    void init(){

      dubins_calculator = this->create_client<dubins_planner_msgs::srv::DubinsPlanning>("dubins_calculator");
      while (!dubins_calculator->wait_for_service(2s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
      }

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Dubins calculator available.");

      service = this->create_service<dubins_planner_msgs::srv::MultiPointDubinsPlanning>
        ("multi_point_dubins_calculator", 
        std::bind(
        &MultiPointDubinsCalculator::calculate,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Multi point dubins calculator ready.");
    }

    void calculate(
      const std::shared_ptr<dubins_planner_msgs::srv::MultiPointDubinsPlanning::Request> request,
      const std::shared_ptr<dubins_planner_msgs::srv::MultiPointDubinsPlanning::Response> response){
        

    }

  
  private:
    std::shared_ptr<rclcpp::Service<dubins_planner_msgs::srv::MultiPointDubinsPlanning>> service;
    std::shared_ptr<rclcpp::Client<dubins_planner_msgs::srv::DubinsPlanning>> dubins_calculator;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<MultiPointDubinsCalculator> calculator = std::make_shared<MultiPointDubinsCalculator>();
  rclcpp::spin(calculator);
  rclcpp::shutdown();
  
  return 0;
}