#include <stdexcept>
#include <sstream>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "braitenbug_msgs/msg/whiskers.hpp"

#define VALUE_IN_RANGE(v,min,max) (v>=min && v<=max) 

struct PolarCoordinates
{
    float angle;
    float radius;
};

//--------------------------------------------------------------------------------------------------
//                                   NORMALIZED LASER SCAN
//--------------------------------------------------------------------------------------------------
class NormalizedLaserScan
{
public:
    NormalizedLaserScan(std::shared_ptr<sensor_msgs::msg::LaserScan> scan);
    std::vector<PolarCoordinates> get_normalized_coordinates_from_range(std::pair<double, double> range);
private:
    std::vector<PolarCoordinates> _measurements;
};

NormalizedLaserScan::NormalizedLaserScan(std::shared_ptr<sensor_msgs::msg::LaserScan> scan)
{
    std::vector<float> ranges = scan->ranges;
    std::transform(ranges.begin(), ranges.end(), ranges.begin(),
                   [scan](double range) { return range / scan->angle_max; });
    _measurements.reserve(scan->ranges.size());
    float angle = scan->angle_min;
    for(auto range: ranges)
    {
        _measurements.push_back(PolarCoordinates{angle, range});
        angle += scan->angle_increment;
    }
}

std::vector<PolarCoordinates> NormalizedLaserScan::get_normalized_coordinates_from_range(std::pair<double, double> range)
{
    std::vector<PolarCoordinates>
}