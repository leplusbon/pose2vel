#ifndef __QUATERNION_HPP__
#define __QUATERNION_HPP__

#include <cmath>
#include "geometry_msgs/Quaternion.h"

namespace jik::quaternion {
    inline double quaternion_to_theta(const double w, const double x, const double y, const double z) {
        const double siny_cosp = 2.0 * (w * z + x * y);
        const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        const double theta = atan2(siny_cosp, cosy_cosp);
        return theta;
    }

    inline double quaternion_to_theta(const geometry_msgs::Quaternion& q) {
        return quaternion_to_theta(
            q.w, q.x, q.y, q.z
        );
    }
}

#endif
