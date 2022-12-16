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
      log_info("Initializing...");
      init();
    }

    void init(){

      dubins_calculator = this->create_client<dubins_planner_msgs::srv::DubinsPlanning>("dubins_calculator");
      log_info("Waiting for dubins calculator...");

      while (!dubins_calculator->wait_for_service(3s)) {
        if (!rclcpp::ok()) {
          log_err("Interrupted while waiting for the service. Exiting.");
        }
        log_info("Service not available, waiting again...");
      }

      service = this->create_service<dubins_planner_msgs::srv::MultiPointDubinsPlanning>
        ("multi_point_dubins_calculator", 
        std::bind(
        &MultiPointDubinsCalculator::calculate,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

      log_info("Multi point dubins calculator ready.");
    }
  
    void calculate(
      const std::shared_ptr<dubins_planner_msgs::srv::MultiPointDubinsPlanning::Request> request,
      const std::shared_ptr<dubins_planner_msgs::srv::MultiPointDubinsPlanning::Response> response){

        std::vector<geometry_msgs::msg::Point> points = request->points;
        int npoints=points.size();
        int omega = request->komega;

        std::vector<std::vector<int>> L(npoints , std::vector<int>(omega, std::numeric_limits<int>::max()));
        for(int j=0; j < L[0].size(); j++){
          L[L.size()-1][j] = 0;
        }

        float minLenght = std::numeric_limits<float>::max();
        float minAngleAlpha, minAngleBeta;
        
        for(int i=0; i<omega; i++){
          for(int j=0; j<omega; j++){
            
          }
        }
    }

  
  private:
    std::shared_ptr<rclcpp::Service<dubins_planner_msgs::srv::MultiPointDubinsPlanning>> service;
    std::shared_ptr<rclcpp::Client<dubins_planner_msgs::srv::DubinsPlanning>> dubins_calculator;



    inline void log_info(const std::string log){
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Multi point dubins calculator: "<<log);
    }

    inline void log_err(const std::string log){
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Multi point dubins calculator: "<<log);
    }

    inline void log_warn(const std::string log){
      RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "Multi point dubins calculator: "<<log);
    }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<MultiPointDubinsCalculator> calculator = std::make_shared<MultiPointDubinsCalculator>();
  rclcpp::spin(calculator);
  rclcpp::shutdown();
  
  return 0;
}