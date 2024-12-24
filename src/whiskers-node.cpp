#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "braitenbug_msgs/msg/whiskers.hpp"

#define constrain(min, val, max) val<min?min:val>max?max:val

class WhiskersNode: public rclcpp::Node
{
public:
  WhiskersNode();
  void declare_parameters();
  void update_parameters();
  void print_parameters();

private:
  constexpr static const char* PARAM_DIST_MAX = "dist_max";
  constexpr static const char* PARAM_DIST_MIN = "dist_min";
  constexpr static double DEFAULT_DIST_MIN = 0.0;
  constexpr static double DEFAULT_DIST_MAX = 0.0;
  constexpr static const char* DESC_DIST_MAX = "dist_max";
  constexpr static const char* DESC_DIST_MIN = "dist_min";


private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _scan_subscription;
  rclcpp::Publisher<braitenbug_msgs::msg::Whiskers>::SharedPtr _whiskers_publisher;
  double _dist_max;
  double _dist_min;

private:
  void _scan_to_whiskers(std::shared_ptr<sensor_msgs::msg::LaserScan> scan);
  bool _validate_scan(std::shared_ptr<sensor_msgs::msg::LaserScan> scan);
  double _get_avarage_distance_in_cone(int cone_deviation,
                                       double cone_angle,
                                       std::shared_ptr<sensor_msgs::msg::LaserScan> scan);

};

void WhiskersNode::declare_parameters()
{
  auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

  descriptor.description = DESC_DIST_MAX;
  this->declare_parameter(PARAM_DIST_MAX, DEFAULT_DIST_MAX, descriptor);
  descriptor.description = DESC_DIST_MIN;
  this->declare_parameter(PARAM_DIST_MIN, DEFAULT_DIST_MIN, descriptor);
}

void WhiskersNode::update_parameters()
{
  this->_dist_min = this->get_parameter(PARAM_DIST_MIN).as_double();
  this->_dist_max = this->get_parameter(PARAM_DIST_MAX).as_double();
}

void WhiskersNode::print_parameters()
{
  RCLCPP_INFO(this->get_logger(), "Parameter %s = %f", PARAM_DIST_MIN, this->_dist_min);
  RCLCPP_INFO(this->get_logger(), "Parameter %s = %f", PARAM_DIST_MAX, this->_dist_max);
}

WhiskersNode::WhiskersNode(): Node("whiskers")
{
  declare_parameters();
  update_parameters();
  print_parameters();
  _scan_subscription = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan",
    rclcpp::SensorDataQoS(),
    std::bind(&WhiskersNode::_scan_to_whiskers, this, std::placeholders::_1)
  );
  _whiskers_publisher = this->create_publisher<braitenbug_msgs::msg::Whiskers>(
    "whiskers",
    rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable)
  );
  _dist_min = DEFAULT_DIST_MIN;
  _dist_max = DEFAULT_DIST_MAX;
}

void WhiskersNode::_scan_to_whiskers(std::shared_ptr<sensor_msgs::msg::LaserScan> scan)
{
  if(!_validate_scan(scan))
  {
    return;
  }
  int cone_deviation = (int)floor(22.0/scan->angle_increment);
  auto whiskers = braitenbug_msgs::msg::Whiskers();
  whiskers.side_left = _get_avarage_distance_in_cone(cone_deviation, -M_PI/2, scan);
  whiskers.front_left = _get_avarage_distance_in_cone(cone_deviation, -M_PI/4, scan);
  whiskers.center = _get_avarage_distance_in_cone(cone_deviation, 0, scan);
  whiskers.front_right = _get_avarage_distance_in_cone(cone_deviation, M_PI/4, scan);
  whiskers.side_right = _get_avarage_distance_in_cone(cone_deviation, M_PI/2, scan);
  _whiskers_publisher->publish(whiskers);
}

bool WhiskersNode::_validate_scan(std::shared_ptr<sensor_msgs::msg::LaserScan> scan)
{
  if(scan->angle_min > -M_PI || scan->angle_max < M_PI)
  {
    RCLCPP_ERROR(this->get_logger(), "Whiskers must use a 360 lidar as input.");
    return false;
  }
  if(scan->angle_increment<1.0)
  {
    RCLCPP_ERROR(this->get_logger(), 
                 "Whiskers must use lidar with scan increment not greater than 1.");
    return false;
  }
  return true;
}

double WhiskersNode::_get_avarage_distance_in_cone(int cone_deviation,
                                                  double cone_angle,
                                                  std::shared_ptr<sensor_msgs::msg::LaserScan> scan)
{
  int base_idx = (int)floor(cone_angle/scan->angle_increment);
  float distance = 0.0;
  for(int idx=base_idx-cone_deviation; idx<=base_idx+cone_deviation; idx++)
  {
    if(idx<0)
    {
      idx += scan->ranges.size();
    }
    else if (idx>scan->ranges.size())
    {
      idx -= scan->ranges.size();
    }
    distance += scan->ranges.at(idx);
  }
  distance /= cone_deviation*2;
  return distance;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WhiskersNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
