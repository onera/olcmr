#ifndef CS_TOOLS__COMMON_HPP_
#define CS_TOOLS__COMMON_HPP_

#include <cmath>
#include <map>

namespace cs_tools {

/// \enum CoordinateType
/// \brief Unique identifiers for coordinate system
enum CoordinateType {
  /// \brief Local tangent plane (East, North, Up)
  ENU = 1,
  /// \brief Local tangent plane (North, East, Down)
  NED = 2,
  /// \brief Local tangent plane (North, West, Up)
  NWU = 3
};

static const std::map<std::string, CoordinateType> CoordinateName = {
  {"ENU", ENU},
  {"NED", NED},
  {"NWU", NWU}
};

/// \brief Convert from degree to radian
/// \param[in] double angle in degree
/// \return double angle in radian
inline double radian(double val) { return val * M_PI / 180.0f; }

/// \brief Convert from radian degree
/// \param[in] double angle in radian
/// \return double angle in degree
inline double degree(double val) { return val * 180.0f / M_PI; }

} // namespace cs_tools

#endif // CS_TOOLS__COMMON_HPP_
