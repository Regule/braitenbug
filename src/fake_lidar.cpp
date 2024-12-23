//=================================================================================================
//                                          INCLUDES
//=================================================================================================
#include <chrono>
#include <memory>
#include <cmath>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"


//=================================================================================================
//                                        LIDAR CONFIG 
//=================================================================================================
class LidarConfig
{
public:
    constexpr static const char* PARAM_MIN_RANGE = "min_range";
    constexpr static const char* PARAM_MAX_RANGE = "max_range";
    constexpr static const char* PARAM_MIN_ANGLE = "min_angle";
    constexpr static const char* PARAM_MAX_ANGLE = "max_angle";
    constexpr static const char* PARAM_SAMPLE_COUNT = "sample_count";
    constexpr static const char* PARAM_SAMPLING_FREQUENCY = "sampling_frequency";
    constexpr static const char* PARAM_SIGMA = "sigma";

    constexpr static const double DEFAULT_MIN_RANGE = 0.2;
    constexpr static const double DEFAULT_MAX_RANGE = 2.0;
    constexpr static const double DEFAULT_MIN_ANGLE = -M_PI;
    constexpr static const double DEFAULT_MAX_ANGLE = M_PI;
    constexpr static const int DEFAULT_SAMPLE_COUNT = 360;
    constexpr static const double DEFAULT_SAMPLING_FREQUENCY = 10.0;
    constexpr static const double DEFAULT_SIGMA = 1.0;

    constexpr static const char* DESCRIPTION_MIN_RANGE =
    "minimum range value [m]";
    constexpr static const char* DESCRIPTION_MAX_RANGE =
    "maximum range value [m]";
    constexpr static const char* DESCRIPTION_MIN_ANGLE =
    "start angle of the scan [rad]";
    constexpr static const char* DESCRIPTION_MAX_ANGLE =
    "end angle of the scan [rad]";
    constexpr static const char* DESCRIPTION_SAMPLE_COUNT =
    "Number of samples per full laser scan";
    constexpr static const char* DESCRIPTION_SAMPLING_FREQUENCY =
    "Number of full Scans per second.";
    constexpr static const char* DESCRIPTION_SIGMA =
    "Standard deviation of obstacle (it is essentialy normal distribution pushed into circle).";

    constexpr static const double epsilon = 0.0001;

public:
    std::pair<double,double> range;
    std::pair<double,double> angle;
    int sample_count;
    double sampling_frequency;
    double sigma;

public:
    void declare_parameters(rclcpp::Node *node);
    void update_parameters(rclcpp::Node *node);
    void print_config(rclcpp::Node *node);

    double get_scan_step() const;
    int get_scan_period_ms() const;
    double get_scaled_sample(double scale) const;


};
    
double LidarConfig::get_scaled_sample(double scale) const
{
    // Range is not inclusive we need substract small number (epsilon)
    // so that scale 1.0 will return valid sample.
    return range.first + (range.second - range.first - epsilon) * scale;
}

int LidarConfig::get_scan_period_ms() const
{
    return std::lround(1000/sampling_frequency);
}

double LidarConfig::get_scan_step() const
{
    return (angle.second-angle.first)/sample_count;
}    

void LidarConfig::declare_parameters(rclcpp::Node *node)
{
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.description = DESCRIPTION_MIN_RANGE;
    node->declare_parameter(PARAM_MIN_RANGE, DEFAULT_MIN_RANGE, descriptor);
    descriptor.description = DESCRIPTION_MAX_RANGE;
    node->declare_parameter(PARAM_MAX_RANGE, DEFAULT_MAX_RANGE, descriptor);
    descriptor.description = DESCRIPTION_MIN_ANGLE;
    node->declare_parameter(PARAM_MIN_ANGLE, DEFAULT_MIN_ANGLE, descriptor);
    descriptor.description = DESCRIPTION_MAX_ANGLE;
    node->declare_parameter(PARAM_MAX_ANGLE, DEFAULT_MAX_ANGLE, descriptor);
    descriptor.description = DESCRIPTION_SAMPLE_COUNT;
    node->declare_parameter(PARAM_SAMPLE_COUNT, DEFAULT_SAMPLE_COUNT, descriptor);
    descriptor.description = DESCRIPTION_SAMPLING_FREQUENCY;
    node->declare_parameter(PARAM_SAMPLING_FREQUENCY, DEFAULT_SAMPLING_FREQUENCY, descriptor);
    descriptor.description = DESCRIPTION_SIGMA;
    node->declare_parameter(PARAM_SIGMA, DEFAULT_SIGMA, descriptor);
}

void LidarConfig::update_parameters(rclcpp::Node *node)
{
    this->range.first = node->get_parameter(PARAM_MIN_RANGE).as_double();
    this->range.second = node->get_parameter(PARAM_MAX_RANGE).as_double();
    this->angle.first = node->get_parameter(PARAM_MIN_ANGLE).as_double();
    this->angle.second = node->get_parameter(PARAM_MAX_ANGLE).as_double();
    this->sample_count = node->get_parameter(PARAM_SAMPLE_COUNT).as_int();
    this->sampling_frequency = node->get_parameter(PARAM_SAMPLING_FREQUENCY).as_double();
    this->sigma = node->get_parameter(PARAM_SIGMA).as_double();
}

void LidarConfig::print_config(rclcpp::Node *node)
{
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_MIN_RANGE, this->range.first);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_MAX_RANGE, this->range.second);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_MIN_ANGLE, this->angle.first);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_MAX_ANGLE, this->angle.second);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %d", PARAM_SAMPLE_COUNT, this->sample_count);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_SAMPLING_FREQUENCY,
                this->sampling_frequency);
    RCLCPP_INFO(node->get_logger(), "Scan step = %f", this->get_scan_step());
    RCLCPP_INFO(node->get_logger(), "Sigma = %f", this->sigma);
}

//=================================================================================================
//                                         FAKE LIDAR NODE 
//=================================================================================================
class FakeLidar: public rclcpp::Node
{
public:
    FakeLidar();

private:
    LidarConfig _config;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr _scan_publisher;
    rclcpp::TimerBase::SharedPtr _scan_timer;
    std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
    int _scan_counter;
    double _scale_normalization;
    

private:
    void _publish_fake_scan();

};

FakeLidar::FakeLidar(): Node("fake_lidar")
{
    _config.declare_parameters(this);
    _config.update_parameters(this);
    _config.print_config(this);
    _scan_publisher = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan",10);
    _scan_timer = this->create_wall_timer(
        std::chrono::milliseconds(_config.get_scan_period_ms()),
        std::bind(&FakeLidar::_publish_fake_scan, this)
    );
    _tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    _scan_counter = 0;

    geometry_msgs::msg::TransformStamped transformStamped;

    transformStamped.header.stamp = this->get_clock()->now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "laser_frame";
    _tf_broadcaster->sendTransform(transformStamped);

    _scale_normalization = exp(0)/( _config.sigma * sqrt(2*M_PI));


}
    
void FakeLidar::_publish_fake_scan()
{
    auto msg = sensor_msgs::msg::LaserScan();
    
    msg.header.stamp = this->now();
    msg.header.frame_id = "laser_frame";

    msg.angle_min = _config.angle.first;
    msg.angle_max = _config.angle.second;
    msg.range_min = _config.range.first;
    msg.range_max = _config.range.second;
    msg.angle_increment = _config.get_scan_step();
    msg.time_increment = 0;
    msg.scan_time = _config.get_scan_period_ms()/1000.0;

    std::vector<float> ranges(_config.sample_count);
    for(int i=0; i<_config.sample_count; i++)
    {
        double x = i;
        if(i>=_scan_counter+_config.sample_count/2)
        {
            x =   2*_scan_counter + _config.sample_count - i;
        }
        else
        {
            x = i -_scan_counter;
        }
        double divider = _config.sigma * sqrt(2*M_PI);
        double exponent = 0.0-pow(x,2)/(2.0*pow(_config.sigma,2));
        double scale = 1 - exp(exponent)/(divider*_scale_normalization);
        ranges[i] = _config.get_scaled_sample(scale); 
    }
    _scan_counter++;
    if(_scan_counter>=_config.sample_count)
    {
        _scan_counter = 0;
    }
    msg.ranges = ranges;
    _scan_publisher->publish(msg);

}

//=================================================================================================
//                                          MAIN 
//=================================================================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<FakeLidar>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

