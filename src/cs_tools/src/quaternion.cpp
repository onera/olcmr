#include <array>
#include <iostream>
#include <cmath>
#include "cs_tools/quaternion.hpp"

// Source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

namespace cs_tools
{

  //--------------------------------------------------------------------------
  std::string Quaternion::toString() const
  {
    std::ostringstream out;
    out << " - w: " << w_ << std::endl
        << " - x: " << x_ << std::endl
        << " - y: " << y_ << std::endl
        << " - z: " << z_;
    return out.str();
  }
  //--------------------------------------------------------------------------
  Quaternion::Quaternion(double angle, std::array<double, 3> axis)
  {
    double norm = std::sqrt(axis[0]*axis[0]+axis[1]*axis[1]+axis[2]*axis[2]);
    double sa = std::sin(angle/2.0f);
    w_ = std::cos(angle/2.0f);
    x_ = sa*axis[0]/norm;
    y_ = sa*axis[1]/norm;
    z_ = sa*axis[2]/norm;
  }

  //--------------------------------------------------------------------------
  Quaternion::Quaternion(const Euler& euler)
  {
    // Abbreviations for the various angular functions
    double cy = std::cos(euler.yaw * 0.5);
    double sy = std::sin(euler.yaw * 0.5);
    double cp = std::cos(euler.pitch * 0.5);
    double sp = std::sin(euler.pitch * 0.5);
    double cr = std::cos(euler.roll * 0.5);
    double sr = std::sin(euler.roll * 0.5);

    Quaternion q;
    w_ = cr * cp * cy + sr * sp * sy;
    x_ = sr * cp * cy - cr * sp * sy;
    y_ = cr * sp * cy + sr * cp * sy;
    z_ = cr * cp * sy - sr * sp * cy;
  }

  //--------------------------------------------------------------------------
  Euler Quaternion::euler() const
  {
    Euler euler;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (w_ * x_ + y_ * z_);
    double cosr_cosp = 1 - 2 * (x_ * x_ + y_ * y_);
    euler.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (w_ * y_ - z_ * x_);
    if (std::abs(sinp) >= 1)
        euler.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        euler.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w_ * z_ + x_ * y_);
    double cosy_cosp = 1 - 2 * (y_ * y_ + z_ * z_);
    euler.yaw = std::atan2(siny_cosp, cosy_cosp);

    return euler;
  }

  //--------------------------------------------------------------------------
  Quaternion Quaternion::mult(const Quaternion& q) const
  {
    Quaternion out;

    out.w_ = w_*q.w_ - x_*q.x_ - y_*q.y_ - z_*q.z_;
    out.x_ = w_*q.x_ + x_*q.w_ + y_*q.z_ - z_*q.y_;
    out.y_ = w_*q.y_ - x_*q.z_ + y_*q.w_ + z_*q.x_;
    out.z_ = w_*q.z_ + x_*q.y_ - y_*q.x_ + z_*q.w_;

    return out;
  }

  //--------------------------------------------------------------------------
  Quaternion Quaternion::conjugate() const
  {
    Quaternion out;
    out.w_=+w_;
    out.x_=-x_;
    out.y_=-y_;
    out.z_=-z_;
    return out;
  }

  //--------------------------------------------------------------------------
  Quaternion Quaternion::operator*(const Quaternion &q) const
  {
    return mult(q);
  }

  //--------------------------------------------------------------------------
  Quaternion Quaternion::operator+(const Quaternion &q) const
  {
    Quaternion out;
    out.w_=w_+q.w_;
    out.x_=x_+q.x_;
    out.y_=y_+q.y_;
    out.z_=z_+q.z_;
    return out;
  }

  //--------------------------------------------------------------------------
  Quaternion Quaternion::operator-() const
  {
    Quaternion out;
    out.w_=-w_;
    out.x_=-x_;
    out.y_=-y_;
    out.z_=-z_;
    return out;
  }
  //--------------------------------------------------------------------------
  Quaternion Quaternion::operator-(const Quaternion &q) const
  {
    Quaternion out;
    out.w_=w_-q.w_;
    out.x_=x_-q.x_;
    out.y_=y_-q.y_;
    out.z_=z_-q.z_;
    return out;
  }


  //--------------------------------------------------------------------------
  std::ostream& operator<<(std::ostream& o, const Quaternion& v)
  {
    o << v.toString();
    return o;
  }

}  // namespace cs_tools


