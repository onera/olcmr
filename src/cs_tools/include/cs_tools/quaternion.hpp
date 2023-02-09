#ifndef CS_TOOLS__QUATERNION_HPP_
#define CS_TOOLS__QUATERNION_HPP_

#include <sstream>
#include <array>
#include "cs_tools/common.hpp"

namespace cs_tools
{

/// \struct Euler
/// \brief Orientation angles (Yaw-Pitch-Roll)
struct Euler
{
  double yaw = 0.0f;
  double pitch = 0.0f;
  double roll = 0.0f;

  Euler(double yaw=0.0, double pitch=0.0, double roll=0.0)
  {
    this->yaw=yaw;
    this->pitch=pitch;
    this->roll=roll;
  }

  std::string toString() const
  {
    std::ostringstream out;
    out << " - yaw:   "<<yaw<<std::endl
        << " - pitch: "<<pitch<<std::endl
        << " - roll:  "<<roll;
    return out.str();
  }
};

inline Euler degree(const Euler &in)
{
  Euler out;
  out.yaw = degree(in.yaw);
  out.pitch = degree(in.pitch);
  out.roll = degree(in.roll);
  return out;
}

inline Euler radian(const Euler &in)
{
  Euler out;
  out.yaw = radian(in.yaw);
  out.pitch = radian(in.pitch);
  out.roll = radian(in.roll);
  return out;
}

/// \class Quaternion
/// \brief Orientation quaternion (w,x,y,z)
class Quaternion
{
public:
  /// \brief Constructor
  /// \param[in] w double Scalar part of quaternion
  /// \param[in] x double Vector part of quaternion (x)
  /// \param[in] y double Vector part of quaternion (y)
  /// \param[in] z double Vector part of quaternion (z)
  Quaternion(double w=1, double x=0, double y=0, double z=0)
  :w_(w), x_(x), y_(y), z_(z)
  {};

  double w() {return w_;}
  double x() {return x_;}
  double y() {return y_;}
  double z() {return z_;}

  /// \brief Constructor from Euler angles
  /// \param[in] angles Euler Euler angles [rad]
  Quaternion(const Euler& angles);

  /// \brief Constructor from Angle and Axis
  /// \param[in] angle double Rotation angle [rad]
  /// \param[in] axis std::array<double, 3> Unit axis
  Quaternion(double angle, std::array<double, 3> axis);

  std::string toString() const;
  //void from_euler(const Euler &in);
  //void from_axis(double angle, double vx, double vy, double vz);
  Euler euler() const;

  Quaternion conjugate() const;
  Quaternion operator-() const;
  Quaternion operator+(const Quaternion &q) const;
  Quaternion operator-(const Quaternion &q) const;
  Quaternion operator*(const Quaternion &q) const;

  //Quaternion transform(CoordinateType from, CoordinateType to) const;
  //Quaternion transform(const std::string& from, const std::string& to) const;

  private:
    double w_;
    double x_;
    double y_;
    double z_;

    Quaternion mult(const Quaternion &q) const;
  };

  std::ostream &operator<<(std::ostream &o, const Euler &v);
  std::ostream &operator<<(std::ostream &o, const Quaternion &v);

} // namespace cs_tools
#endif  // CS_TOOLS__QUATERNION_HPP_
