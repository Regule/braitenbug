#include <chrono>
#include <memory>
#include <cmath>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "fake_lidar_config.hpp"


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
            x = i - _config.sample_count - _scan_counter;
            //x =   2*_scan_counter + _config.sample_count - i;
        }
        else if(i<=_scan_counter-_config.sample_count/2)
        {
            x = i + _config.sample_count/2  - _scan_counter;
            RCLCPP_INFO(this->get_logger(), "X=%f", x);
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

