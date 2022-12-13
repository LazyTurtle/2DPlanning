#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"

class MultiPointDubinsCalculator : public rclcpp::Node
{
  public:
    MultiPointDubinsCalculator()
    : Node("multi_point_dubins_calculator")
    {
      
    }
  void init(){

  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<MultiPointDubinsCalculator> calculator = std::make_shared<MultiPointDubinsCalculator>();
  calculator->init();
  rclcpp::spin(calculator);
  rclcpp::shutdown();
  
  return 0;
}