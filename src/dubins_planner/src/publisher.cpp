#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */
using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      
      publisher_ = this->create_publisher<nav_msgs::msg::Path>("plan", 10);
      timer_ = this->create_wall_timer
      (500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    size_t count_;
    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback()
        {
          nav_msgs::msg::Path path_msg;
          path_msg.header.stamp = this->get_clock()->now();
          path_msg.header.frame_id = "map";
          geometry_msgs::msg::PoseStamped temp;
          temp.header.stamp = this->get_clock()->now();
          temp.header.frame_id = "";
          temp.pose.position.x = 0;
          temp.pose.position.y = 0;
          temp.pose.position.z = 0 ;
          temp.pose.orientation.x = 0;
          temp.pose.orientation.y = 0;
          temp.pose.orientation.z = 0;
          temp.pose.orientation.w = 1;
          path_msg.poses.push_back(temp);
          temp.pose.position.x = 4;
          temp.pose.position.y = 6;
          temp.pose.position.z = 0 ;
          temp.pose.orientation.x = 0;
          temp.pose.orientation.y = 0;
          temp.pose.orientation.z = 0;
          temp.pose.orientation.w = 1;
          path_msg.poses.push_back(temp);
          publisher_->publish(path_msg);
        }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  
  return 0;
}