#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <tuple>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "dubins_planner_msgs/srv/dubins_planning.hpp"

#define PI 3.14159265

using namespace std::chrono_literals;

////////////////////////////////////////
///// Structures and Classes

struct DubinsArc{
  float 
    x0, y0, th0,
    xf, yf, thf,
    k, L;
} typedef DubinsArc;

struct DubinsCurve{
  DubinsArc a1, a2, a3;
  float L;
} typedef DubinsCurve;


////////////////////////////////////////
///// Node Classes

class DubinsCalculator : public rclcpp::Node
{
  public:
    DubinsCalculator():Node("server"){
      // rcutils_logging_set_logger_level(
      //   this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

      service = this->create_service<dubins_planner_msgs::srv::DubinsPlanning>
        ("dubins_calculator", 
        std::bind(
        &DubinsCalculator::calculate,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
    }
  
    void calculate(
      const std::shared_ptr<dubins_planner_msgs::srv::DubinsPlanning::Request> request,
      const std::shared_ptr<dubins_planner_msgs::srv::DubinsPlanning::Response> response){
      
      
      float x0 = request->start.point.x;
      float y0 = request->start.point.y;
      float th0 = PI*request->start.angle;

      float xf = request->end.point.x;
      float yf = request->end.point.y;
      float thf = PI*request->end.angle;

      float Kmax = request->kmax;

      DubinsCurve curve;
      int curve_id;
      std::tie(curve_id, curve) = dubins_shortest_path(x0, y0, th0, xf, yf, thf, Kmax);

      std::vector<std::tuple<float, float>> points = getPlotPoints(curve);

      response->path = convert_to_path(points);
    }

  private:
    std::shared_ptr<rclcpp::Service<dubins_planner_msgs::srv::DubinsPlanning>> service;

    nav_msgs::msg::Path convert_to_path(
      const std::vector<std::tuple<float, float>>& points){
      
      nav_msgs::msg::Path path;
      path.header.stamp = this->get_clock()->now();
      path.header.frame_id = "map";

      for(auto point : points){
        geometry_msgs::msg::PoseStamped temp;
        temp.header.stamp = this->get_clock()->now();
        temp.header.frame_id = "";
        temp.pose.position.x = std::get<0>(point);
        temp.pose.position.y = std::get<1>(point);
        temp.pose.position.z = 0 ;
        temp.pose.orientation.x = 0;
        temp.pose.orientation.y = 0;
        temp.pose.orientation.z = 0;
        temp.pose.orientation.w = 1;
        path.poses.push_back(temp);
      }
      return path;

    }

    ////////////////////////////////////////
    ///// Helper functions

    static float sinc(const float t){
      float s= (abs(t)<0.002) ? (1 - pow(t,2)/6 * (1 - pow(t,2)/20)) : sin(t)/t;
      return s;
    }

    // Normalize an angle in range [0,2*pi)
    static float mod2pi(const float angle){
      float moduled_angle = angle;
      while(moduled_angle<0){
        moduled_angle += 2*PI;
      }
      while(moduled_angle>=2*PI){
        moduled_angle -= 2*PI;
      }
      return moduled_angle;
    }

    // Normalize an angle in range [-pi,pi)
    static float rangeSymm(const float angle){
      float moduled_angle = angle;
      while(moduled_angle<-PI){
        moduled_angle += 2*PI;
      }
      while(moduled_angle>=PI){
        moduled_angle -= 2*PI;
      }
      return moduled_angle;
    }

    static bool check_dubins(
      const float s1, const float k0,
      const float s2, const float k1,
      const float s3, const float k2,
      const float th0,const float thf){

        const float x0 = -1.0;
        const float y0 = 0.0;
        const float xf = 1.0;
        const float yf = 0.0;

        const float eq1 = 
          x0 + 
          s1 * sinc((1./2.) * k0 * s1) * cos(th0 + (1./2.) * k0 * s1) + 
          s2 * sinc((1./2.) * k1 * s2) * cos(th0 + k0 * s1 + (1./2.) * k1 * s2) +
          s3 * sinc((1./2.) * k2 * s3) * cos(th0 + k0 * s1 + k1 * s2 + (1./2.) * k2 * s3) - xf;

        const float eq2 = 
          y0 + 
          s1 * sinc((1/2.) * k0 * s1) * sin(th0 + (1/2.) * k0 * s1) + 
          s2 * sinc((1/2.) * k1 * s2) * sin(th0 + k0 * s1 + (1/2.) * k1 * s2) + 
          s3 * sinc((1/2.) * k2 * s3) * sin(th0 + k0 * s1 + k1 * s2 + (1/2.) * k2 * s3) - yf;

        const float eq3 = rangeSymm(k0 * s1 + k1 * s2 + k2 * s3 + th0 - thf);

        const bool Lpos = (s1 > 0) || (s2 > 0) || (s3 > 0);
        const float epsilon = sqrt(eq1*eq1+eq2*eq2+eq3*eq3);
        const bool result = (epsilon < 1.e-6) && Lpos;
        return result;
      }

    // Scale the input problem to standard form (x0: -1, y0: 0, xf: 1, yf: 0)
    static std::tuple<float, float, float, float> scaleToStandard(
      const float x0, const float y0, const float th0, 
      const float xf, const float yf, const float thf, 
      const float Kmax){
        const float dx = xf - x0;
        const float dy = yf - y0;
        const float phi = atan2(dy,dx);
        const float lambda = hypot(dx,dy)/2.0;

        const float scaled_th0 = mod2pi(th0 - phi);
        const float scaled_th1 = mod2pi(thf - phi);
        const float scaled_Kmax = Kmax*lambda;
        auto result = std::make_tuple(scaled_th0, scaled_th1, scaled_Kmax, lambda);
        return result;
      }

    static std::tuple<float, float, float> scaleFromStandard(
      const float lambda, const float sc_s1, const float sc_s2, const float sc_s3){

        const float s1 = sc_s1 * lambda;
        const float s2 = sc_s2 * lambda;
        const float s3 = sc_s3 * lambda;
        auto result = std::make_tuple(s1,s2,s3);
        return result;
      }

    static std::tuple<float,float,float> circline(
      const float s, const float x0, const float y0, const float th0, const float k){

        const float x = x0 + s * sinc(k * s / 2.0) * cos(th0 + k * s / 2);
        const float y = y0 + s * sinc(k * s / 2.0) * sin(th0 + k * s / 2);
        const float th = mod2pi(th0 + k * s);
        return std::make_tuple(x,y,th);
    }

    const DubinsArc getArc(
      const float x0, const float y0, const float th0, const float k, const float L){

        DubinsArc arc;
        arc.x0 = x0;
        arc.y0 = y0;
        arc.th0 = th0;
        arc.k = k;
        arc.L = L;
        std::tie(arc.xf, arc.yf, arc.thf) = circline(L,x0,y0,th0,k);
        return arc;
      }

    const DubinsCurve getCurve(
      const float x0, const float y0, const float th0,
      const float s1, const float s2, const float s3,
      const float k0, const float k1, const float k2){

        DubinsCurve curve;
        curve.a1 = getArc(x0, y0, th0, k0, s1);
        curve.a2 = getArc(curve.a1.xf, curve.a1.yf, curve.a1.thf, k1, s2);
        curve.a3 = getArc(curve.a2.xf, curve.a2.yf, curve.a2.thf, k2, s3);
        curve.L = curve.a1.L + curve.a2.L + curve.a3.L;
        return curve;
    }

    std::vector<std::tuple<float,float>> getPlotPoints(
      const DubinsArc arc, const int nPoints=10){

      std::vector<std::tuple<float,float>> points;
      for(int i = 0; i<nPoints; i++){
        float s = arc.L/nPoints * i;
        auto point = circline(s,arc.x0,arc.y0,arc.th0,arc.k);
        points.push_back(std::make_tuple(std::get<0>(point),std::get<1>(point)));
      }
      return points;
    }

    std::vector<std::tuple<float,float>> getPlotPoints(
      const DubinsCurve curve, const int nPoints=10){

        std::vector<std::tuple<float,float>> points;
        auto arc1Points = getPlotPoints(curve.a1, nPoints);
        auto arc2Points = getPlotPoints(curve.a2, nPoints);
        auto arc3Points = getPlotPoints(curve.a3, nPoints);

        points.insert(points.end(),arc1Points.begin(),arc1Points.end());
        points.insert(points.end(),arc2Points.begin(),arc2Points.end());
        points.insert(points.end(),arc3Points.begin(),arc3Points.end());

        return points;
    }

    ////////////////////////////////////////
    ///// DUBINS SOLUTIONS

    static std::tuple<bool,float,float,float> LSL(
      const float sc_th0, const float sc_thf, const float sc_Kmax){
        float sc_s1, sc_s2, sc_s3;
        bool ok = false;

        const float invK = 1 / sc_Kmax;
        const float C = cos(sc_thf) - cos(sc_th0);
        const float S = 2 * sc_Kmax + sin(sc_th0) - sin(sc_thf);
        const float temp1 = atan2(C, S);
        const float temp2 = 2 + 4 * pow(sc_Kmax,2) - 2 * cos(sc_th0 - sc_thf) + 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf));

        sc_s1 = invK * mod2pi(temp1 - sc_th0);
        if(temp2 < 0){
          ok = false;
          sc_s1 = 0; sc_s2 = 0; sc_s3 = 0;
        }else{
          ok = true;
          sc_s2 = invK * sqrt(temp2);
          sc_s3 = invK * mod2pi(sc_thf - temp1);
        }

        auto result = std::make_tuple(ok, sc_s1, sc_s2, sc_s3);
        return result;
      }

    static std::tuple<bool,float,float,float> RSR(
      const float sc_th0, const float sc_thf, const float sc_Kmax){
        float sc_s1, sc_s2, sc_s3;
        bool ok = false;

        const float invK = 1 / sc_Kmax;
        const float C = cos(sc_th0) - cos(sc_thf);
        const float S = 2 * sc_Kmax - sin(sc_th0) + sin(sc_thf);
        const float temp1 = atan2(C, S);
        const float temp2 = 2 + 4 * pow(sc_Kmax,2) - 2 * cos(sc_th0 - sc_thf) - 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf));

        sc_s1 = invK * mod2pi(sc_th0 - temp1);
        if(temp2 < 0){
          ok = false;
          sc_s1 = 0; sc_s2 = 0; sc_s3 = 0;
        }else{
          ok = true;
          sc_s2 = invK * sqrt(temp2);
          sc_s3 = invK * mod2pi(temp1 - sc_thf);
        }

        auto result = std::make_tuple(ok, sc_s1, sc_s2, sc_s3);
        return result;
      }

    static std::tuple<bool,float,float,float> LSR(
      const float sc_th0, const float sc_thf, const float sc_Kmax){
        float sc_s1, sc_s2, sc_s3;
        bool ok = false;

        const float invK = 1 / sc_Kmax;
        const float C = cos(sc_th0) + cos(sc_thf);
        const float S = 2 * sc_Kmax + sin(sc_th0) + sin(sc_thf);
        const float temp1 = atan2(-C, S);
        const float temp3 = 4 * pow(sc_Kmax,2) - 2 + 2 * cos(sc_th0 - sc_thf) + 4 * sc_Kmax * (sin(sc_th0) + sin(sc_thf));

        if(temp3 < 0){
          ok = false;
          sc_s1 = 0; sc_s2 = 0; sc_s3 = 0;
        }else{
          ok = true;
          sc_s2 = invK * sqrt(temp3);
          const float temp2 = -atan2(-2, sc_s2 * sc_Kmax);
          sc_s1 = invK * mod2pi(temp1 + temp2 - sc_th0);
          sc_s3 = invK * mod2pi(temp1 + temp2 - sc_thf);
        }

        auto result = std::make_tuple(ok, sc_s1, sc_s2, sc_s3);
        return result;
      }

    static std::tuple<bool,float,float,float> RSL(
      const float sc_th0, const float sc_thf, const float sc_Kmax){
        float sc_s1, sc_s2, sc_s3;
        bool ok = false;

        const float invK = 1 / sc_Kmax;
        const float C = cos(sc_th0) + cos(sc_thf);
        const float S = 2 * sc_Kmax - sin(sc_th0) - sin(sc_thf);
        const float temp1 = atan2(C, S);
        const float temp3 = 4 * pow(sc_Kmax,2) - 2 + 2 * cos(sc_th0 - sc_thf) - 4 * sc_Kmax * (sin(sc_th0) + sin(sc_thf));

        if(temp3 < 0){
          ok = false;
          sc_s1 = 0; sc_s2 = 0; sc_s3 = 0;
        }else{
          ok = true;
          sc_s2 = invK * sqrt(temp3);
          const float temp2 = atan2(2, sc_s2 * sc_Kmax);
          sc_s1 = invK * mod2pi(sc_th0 - temp1 + temp2);
          sc_s3 = invK * mod2pi(sc_thf - temp1 + temp2);
        }

        auto result = std::make_tuple(ok, sc_s1, sc_s2, sc_s3);
        return result;
      }

    static std::tuple<bool,float,float,float> RLR(
      const float sc_th0, const float sc_thf, const float sc_Kmax){
        float sc_s1, sc_s2, sc_s3;
        bool ok = false;

        const float invK = 1 / sc_Kmax;
        const float C = cos(sc_th0) - cos(sc_thf);
        const float S = 2 * sc_Kmax - sin(sc_th0) + sin(sc_thf);
        const float temp1 = atan2(C, S);
        const float temp2 = 0.125 * (6 - 4 * pow(sc_Kmax,2) + 2 * cos(sc_th0 - sc_thf) + 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf)));

        if(abs(temp2) > 1){
          ok = false;
          sc_s1 = 0; sc_s2 = 0; sc_s3 = 0;
        }else{
          ok = true;
          sc_s2 = invK * mod2pi(2 * PI - acos(temp2));
          sc_s1 = invK * mod2pi(sc_th0 - temp1 + 0.5 * sc_s2 * sc_Kmax);
          sc_s3 = invK * mod2pi(sc_th0 - sc_thf + sc_Kmax * (sc_s2 - sc_s1));
        }

        auto result = std::make_tuple(ok, sc_s1, sc_s2, sc_s3);
        return result;
      }

    static std::tuple<bool,float,float,float> LRL(
      const float sc_th0, const float sc_thf, const float sc_Kmax){
        float sc_s1, sc_s2, sc_s3;
        bool ok = false;

        const float invK = 1 / sc_Kmax;
        const float C = cos(sc_thf) - cos(sc_th0);
        const float S = 2 * sc_Kmax + sin(sc_th0) - sin(sc_thf);
        const float temp1 = atan2(C, S);
        const float temp2 = 0.125 * (6 - 4 * pow(sc_Kmax,2) + 2 * cos(sc_th0 - sc_thf) - 4 * sc_Kmax * (sin(sc_th0) - sin(sc_thf)));

        if(abs(temp2) > 1){
          ok = false;
          sc_s1 = 0; sc_s2 = 0; sc_s3 = 0;
        }else{
          ok = true;
          sc_s2 = invK * mod2pi(2 * PI - acos(temp2));
          sc_s1 = invK * mod2pi(temp1 - sc_th0 + 0.5 * sc_s2 * sc_Kmax);
          sc_s3 = invK * mod2pi(sc_thf - sc_th0 + sc_Kmax * (sc_s2 - sc_s1));
        }

        auto result = std::make_tuple(ok, sc_s1, sc_s2, sc_s3);
        return result;
      }

    ////////////////////////////////////////
    ///// Solver

    std::tuple<int, DubinsCurve> dubins_shortest_path(
      const float x0, const float y0, const float th0,
      const float xf, const float yf, const float thf, const float Kmax){

        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Calculating path...");

        float sc_th0, sc_thf, sc_Kmax, lambda;
        std::tie(sc_th0, sc_thf, sc_Kmax, lambda) = scaleToStandard(x0, y0, th0, xf, yf, thf, Kmax);
        
        int ksigns[6][3] = {{1,0,1},{-1,0,-1},{1,0,-1},{-1,0,1},{-1,1,-1},{1,-1,1}};
        std::vector<std::function<std::tuple<bool,float,float,float>(float,float,float)>> primitives({
          &LSL, &RSR, &LSR, &RSL, &RLR, &LRL
        });

        int pidx = -1;
        float L = INFINITY, sc_s1, sc_s2, sc_s3;

        for(int i=0; i<(int)primitives.size(); i++){
          float ok, sc_s1_c, sc_s2_c, sc_s3_c;
          std::tie(ok, sc_s1_c, sc_s2_c, sc_s3_c) = primitives[i](sc_th0, sc_thf, sc_Kmax);
          float Lcur = sc_s1_c + sc_s2_c + sc_s3_c;
          if (ok && Lcur<L){
            L = Lcur;
            sc_s1 = sc_s1_c;
            sc_s2 = sc_s2_c;
            sc_s3 = sc_s3_c;
            pidx = i;        
          }
        }
        
        DubinsCurve curve;
        if(pidx>-1){
          float s1, s2, s3;
          std::tie(s1, s2, s3) = scaleFromStandard(lambda, sc_s1, sc_s2, sc_s3);
          curve = getCurve(
            x0, y0, th0,
            s1, s2, s3,
            ksigns[pidx][0]*Kmax, ksigns[pidx][1]*Kmax, ksigns[pidx][2]*Kmax);
          
          bool bCorrectDubins = check_dubins(
              sc_s1, ksigns[pidx][0]*sc_Kmax,
              sc_s2, ksigns[pidx][1]*sc_Kmax,
              sc_s3, ksigns[pidx][2]*sc_Kmax,
              sc_th0, sc_thf);
          
          assert(bCorrectDubins);
            
        }
        return std::make_tuple(pidx, curve);
    }
    
};

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  auto node=std::make_shared<DubinsCalculator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
  return 0;
}