#include <stdexcept>
#include <sstream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "braitenbug_msgs/msg/whiskers.hpp"

#define VALUE_IN_RANGE(v,min,max) (v>=min && v<=max) 


//--------------------------------------------------------------------------------------------------
//                                     NORMALIZATION ERROR
//--------------------------------------------------------------------------------------------------
class NormalizationError
{
public:
    NormalizationError(double value) throw();
    virtual std::string get_msg() const throw();
    bool is_valid() const noexcept;
    double get_value() const noexcept;

private:
    double _value;
};

NormalizationError::NormalizationError(double value) throw(): _value(value)
{
}

std::string NormalizationError::get_msg() const throw()
{
    std::stringstream msg;
    if(this->is_valid())
    {
        msg << "Attempted to set a value outside of <0.0;1.0> range (" << _value << ") ";
        msg << "for object where normalized value is requred";
    }
    else
    {
        msg << "Returning normalization error while value is within <0.0;1.0> range (" << _value << "). ";
        msg << "This indicates error in way that expetion is thrown as this value is normalized correctly and ";
        msg << "no exception should be thrown.";
    }
    std::string msg_str = msg.str();
    return msg_str;
}

bool NormalizationError::is_valid() const noexcept
{
    return VALUE_IN_RANGE(_value, 0.0, 1.0);
}

double NormalizationError::get_value() const noexcept
{
    return _value;
}

//--------------------------------------------------------------------------------------------------
//                                   NORMALIZED POLAR COORDINATES
//--------------------------------------------------------------------------------------------------

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

NormalizedPolarCoordinates::NormalizedPolarCoordinates(double angle_val,
                                                       double distance_val)
{
    if(!VALUE_IN_RANGE(angle,0.0,1.0))
        throw NormalizationError(angle);
    if(!VALUE_IN_RANGE(distance,0.0,1.0))
        throw NormalizationError(distance);
    angle = angle_val;
    distance = distance_val;
}

//--------------------------------------------------------------------------------------------------
//                                   NORMALIZED LASER SCAN
//--------------------------------------------------------------------------------------------------
class NormalizedLaserScan
{

};