#ifndef CS_TOOLS__GEO_PROJ_HPP_
#define CS_TOOLS__GEO_PROJ_HPP_

#include <iostream>
#include <sstream>
#include "cs_tools/common.hpp"

namespace cs_tools {

/// \struct Point
/// \brief Position in cartesian coordinates
struct Point
{
  double x;
  double y;
  double z;

  Point(double x=0.0, double y=0.0, double z=0.0)
  {
    this->x=x;
    this->y=y;
    this->z=z;
  }

  std::string toString() const
  {
    std::ostringstream out;
    out << " - x: " << x << std::endl
        << " - y: " << y << std::endl
        << " - z: " << z << std::endl;
    return out.str();
  }
};

/// \struct GeoPoint
/// \brief Position in geographics coordinates
struct GeoPoint
{
  /// \brief latitude Latitude in degrees.
  double latitude;
  /// \brief longitude Longitude in degrees.
  double longitude;
  /// \brief altitude Altitude in meter (above ellipsoid).
  double altitude;

  GeoPoint(double lat=0.0, double lon=0.0, double alt=0.0)
  {
    latitude=lat;
    longitude=lon;
    altitude=alt;
  };

  std::string toString() const
  {
    std::ostringstream out;
    //out << " - latitude:  "<<latitude<<" deg" << std::endl
    //    << " - longitude: "<<longitude<<" deg" << std::endl
    //    << " - altitude:  "<<altitude<<" m"<<std::endl;
    out << "(" << latitude << ", " << longitude << ", " << altitude << ")";
    return out.str();
  }
};

std::ostream &operator<<(std::ostream &o, const Point &p);
std::ostream &operator<<(std::ostream &o, const GeoPoint &p);

/// \brief Convert GeoPoint from degree to radian
/// \param[in] GeoPoint geographic point in degree
/// \return GeoPoint geographic point in radian
inline GeoPoint radian(GeoPoint in) {
  GeoPoint out;
  out.latitude = radian(in.latitude);
  out.longitude = radian(in.longitude);
  out.altitude = in.altitude;
  return out;
}

/// \brief Convert GeoPoint from radian to degree
/// \param[in] GeoPoint geographic point in radian
/// \return GeoPoint geographic point in degree
inline GeoPoint degree(GeoPoint in) {
  GeoPoint out;
  out.latitude = degree(in.latitude);
  out.longitude = degree(in.longitude);
  out.altitude = in.altitude;
  return out;
}

/// \enum SurfaceType
/// \brief Unique identifier for geodetic models (DATUM)
enum SurfaceType {
  /// \brief Model of reference ellipsoid for earth, based on
  /// WGS 84 standard. see wikipedia: World_Geodetic_System
  WGS84 = 1
};

class GeoProj {
public:
  /// \brief Constructor with input paramters.
  /// \param[in] type SurfaceType specification.
  /// \param[in] reference GeoPoint origin for local coordinates.
  GeoProj(const SurfaceType type, const GeoPoint &reference);

  /// \brief Set reference in geodetic coordinates.
  /// \param[in] ref GeoPoint reference geodetic coordinates (angles in rad).
  void set_reference(GeoPoint ref);

  /// \brief Get reference geodetic coordinates.
  /// \return GeoPoint Reference geodetic coordinates (angle in rad).
  GeoPoint get_reference() const;

  /// \brief Convert geodetic to ECEF coordinates
  /// \param[in] in GeoPoint input position in geodetic system
  /// \return Point position in ECEF system
  Point spherical_to_ecef(const GeoPoint &in) const;

  /// \brief Convert ECEF to geodetic coordinates
  /// \param[in] in Point input position in ECEF system
  /// \return GeoPoint position in in geodetic system
  GeoPoint ecef_to_spherical(const Point &in) const;

  /// \brief Convert ECEF to local coordinates
  /// \param[in] in Point input position in ECEF system
  /// \param[in] type CoordinateType input local plan tangent (LPT) system type
  /// \return Point position in local plan tangent system
  Point ecef_to_local(const Point &in, const CoordinateType type) const;

  /// \brief Convert local to ECEF coordinates
  /// \param[in] in Point input position in local plan tangent (LPT) system
  /// \param[in] type CoordinateType input local plan tangent system type
  /// \return Point position in ECEF system
  Point local_to_ecef(const Point &in, const CoordinateType type) const;

  /// \brief Convert spherical to local coordinates
  /// \param[in] in GeoPoint input position in geodetic system
  /// \param[in] type CoordinateType input local plan tangent (LPT) system type
  /// \return Point position in local plan tangent system
  Point spherical_to_local(const GeoPoint &in, const CoordinateType type) const;

  /// \brief Convert local to spherical coordinates
  /// \param[in] in Point input position in local plan tangent (LPT) system
  /// \param[in] type CoordinateType input local plan tangent system type
  /// \return GeoPoint position in in geodetic system
  GeoPoint local_to_spherical(const Point &in, const CoordinateType type) const;

private:
  struct Ellipsoid {
    double a;  // Semi-major axis [m]
    double f;  // flattening
    double e2; // square value of excentricity
  };

  /// Surface type
  SurfaceType surface_type_;

  /// Definition of the ellipsoid
  Ellipsoid E_;

  /// Reference in gedesic coordinate system
  GeoPoint reference_;

  /// Reference in ECEF coordinate system
  Point ref_ecef_;

  /// Projection matrix (ECEF to LTP)
  double PM_[3][3];
};

/// \brief Compute euclidian distance between two point in local coordinate system
double distance(const Point& a, const Point& b);

/// \brief Compute euclidian distance between two point in spherical system
double distance(const GeoPoint& a, const GeoPoint& b);


} // namespace cs_tools

#endif // CS_TOOLS__GEO_PROJ_HPP_
