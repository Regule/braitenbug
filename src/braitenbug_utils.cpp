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
    double x = _get_offseted_index(index);
    double divider = _sigma * sqrt(2*M_PI);
    double exponent = 0.0-pow(x,2)/(2.0*pow(_sigma,2));
    return exp(exponent)/(divider*_normalization_factor);
}

int CyclicGaussian::_get_offseted_index(int index)
{
    if(index>_center+_size/2)
    {
        index =   index - _center - _size;
    }
    else if(index<_center-_size/2)
    {
        index = _size - _center + index + 1;
    }
    else
    {
        index = index -_center;
    }
    return index;
}

}