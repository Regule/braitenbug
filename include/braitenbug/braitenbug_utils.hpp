#ifndef BRAITENBUG_UTILS_HPP
#define BRAITENBUG_UTILS_HPP

#include <cmath>

namespace braitenbug
{

class CyclicGaussian
{
public:
    CyclicGaussian(int center, int size, double sigma);
    double operator[](int index);
private:
    int _center;
    int _size;
    double _sigma;
    double _normalization_factor;
};

} // namespace braitenbug

#endif // BRAITENBUG_UTILS_HPP