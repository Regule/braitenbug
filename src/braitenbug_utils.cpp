#include "braitenbug/braitenbug_utils.hpp"

namespace braitenbug
{
CyclicGaussian::CyclicGaussian(int center, int size, double sigma)
{
    _center = center;
    _size = size;
    _sigma = sigma;
    _normalization_factor = exp(0)/( _sigma * sqrt(2*M_PI));
}

double CyclicGaussian::operator[](int index)
{
    double x = index;
    if(index>=_center+_size/2)
    {
        x =   2*_center + _size - index;
    }
    else if(index<=_center- _size/2)
    {
        x = index + _size/2  - _center;
    }
    else
    {
        x = index -_center;
    }
    double divider = _sigma * sqrt(2*M_PI);
    double exponent = 0.0-pow(x,2)/(2.0*pow(_sigma,2));
    return exp(exponent)/(divider*_normalization_factor);
}

}