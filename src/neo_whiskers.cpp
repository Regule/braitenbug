#include <stdexcept>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "braitenbug_msgs/msg/whiskers.hpp"

#define VALUE_IN_RANGE(v,min,max) v>=min && v<=max 


class NormalizationError: public std::logic_error
{
public:
    double value;

public:
    NormalizationError(double value) throw();
    virtual char const* what() const throw();
};

NormalizationError::NormalizationError(double _value) throw(): value(_value)
{

}

char const* NormalizationError::what() const throw()
{

}

struct NormalizedPolarCoordinates
{
    double angle;
    double distance;

    NormalizedPolarCoordinates();
    NormalizedPolarCoordinates(double angle, double distance);
};

NormalizedPolarCoordinates::NormalizedPolarCoordinates(): angle(0), distance(0)
{
}

NormalizedPolarCoordinates::NormalizedPolarCoordinates(double angle,
                                                       double distance)
{
    if(!(VALUE_IN_RANGE(angle,0.0,1.0) || VALUE_IN_RANGE(distance,0.0,1.0)))
    {

    }
}

class NormalizedLaserScan
{

};