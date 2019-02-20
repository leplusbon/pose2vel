#ifndef __NORM_HPP__
#define __NORM_HPP__

#include <cmath>

namespace jik {
    inline double norm(double x, double y) {
        return sqrt(x * x + y * y);
    }
    
    inline double norm(double x, double y, double z) {
        return sqrt(x * x + y * y + z * z);
    }
}

#endif