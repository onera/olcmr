// Copyright ONERA
//
//

#include "cs_tools/geo_proj.hpp"
#include "cs_tools/common.hpp"

// Sources:
// - https://geodesie.ign.fr/contenu/fichiers/documentation/pedagogiques/TransformationsCoordonneesGeodesiques.pdf
// - https://en.wikipedia.org/wiki/Geographic_coordinate_conversion

namespace cs_tools
{

  //--------------------------------------------------------------------------
  GeoProj::GeoProj(const SurfaceType type, const GeoPoint& reference)
  {
    if(type==WGS84)
    {
      E_.a = 6378137.0f;
      E_.f = (1.0f/298.257223563);
      E_.e2 = 1-(1-E_.f)*(1-E_.f);
    }

    set_reference(reference);
  }

  //--------------------------------------------------------------------------
  void GeoProj::set_reference(GeoPoint ref)
  {
    // Set origin
    reference_ = ref;

    // Set origin in ECEF coordinates system
    ref_ecef_ = spherical_to_ecef(ref);


    // Compute projection matrix PM_ used to project coordinates in LTP
    PM_[0][0] = -sin(radian(ref.longitude));
    PM_[0][1] = +cos(radian(ref.longitude));
    PM_[0][2] = 0.0f;

    PM_[1][0] = -sin(radian(ref.latitude))*cos(radian(ref.longitude));
    PM_[1][1] = -sin(radian(ref.latitude))*sin(radian(ref.longitude));
    PM_[1][2] = cos(radian(ref.latitude));

    PM_[2][0] = cos(radian(ref.latitude))*cos(radian(ref.longitude));
    PM_[2][1] = cos(radian(ref.latitude))*sin(radian(ref.longitude));
    PM_[2][2] = sin(radian(ref.latitude));
  }

  //--------------------------------------------------------------------------
  GeoPoint GeoProj::get_reference() const
  {
    return reference_;
  }

  //--------------------------------------------------------------------------
  Point GeoProj::spherical_to_ecef(const GeoPoint& in) const
  {
    double Sp = sin(radian(in.latitude));
    double Cp = cos(radian(in.latitude));
    double Sl = sin(radian(in.longitude));
    double Cl = cos(radian(in.longitude));
    double N=E_.a/sqrt(1-E_.e2*Sp*Sp);
    Point out;
    out.x = (N+in.altitude)*Cp*Cl;
    out.y = (N+in.altitude)*Cp*Sl;
    out.z = (N*(1-E_.e2)+in.altitude)*Sp;
    return out;
  }

  //--------------------------------------------------------------------------
  GeoPoint GeoProj::ecef_to_spherical(const Point& in) const
  {
    GeoPoint out;
    out.longitude=degree(atan2(in.y, in.x));

    double X2=in.x*in.x;
    double Y2=in.y*in.y;
    double Z2=in.z*in.z;
    double Rt=sqrt(X2+Y2);
    double R=sqrt(X2+Y2+Z2);
    double alpha=atan(in.z/Rt*((1-E_.f)+(E_.e2*E_.a/R)));
    double Sa=sin(alpha);
    double Ca=cos(alpha);
    double lat=atan((in.z*(1-E_.f)+E_.e2*E_.a*Sa*Sa*Sa)/((1-E_.f)*(Rt-E_.e2*E_.a*Ca*Ca*Ca)));
    out.latitude = degree(lat);

    double Cp=cos(lat);
    double Sp=sin(lat);
    out.altitude=Rt*Cp+in.z*Sp-E_.a*sqrt(1-E_.e2*Sp*Sp);
    return out;
  }

  //--------------------------------------------------------------------------
  Point GeoProj::ecef_to_local(const Point& in, const CoordinateType type) const
  {
    double dx = in.x - ref_ecef_.x;
    double dy = in.y - ref_ecef_.y;
    double dz = in.z - ref_ecef_.z;

    // PM_ is projection matrix depending on reference geographic position
    double east  = PM_[0][0]*dx + PM_[0][1]*dy;
    double north = PM_[1][0]*dx + PM_[1][1]*dy + PM_[1][2]*dz;
    double up    = PM_[2][0]*dx + PM_[2][1]*dy + PM_[2][2]*dz;

    Point out;
    switch(type)
    {
      case ENU:
        out.x = east;
        out.y = north;
        out.z = up;
        break;

      case NED:
        out.x = north;
        out.y = east;
        out.z = -up;
        break;

      case NWU:
        out.x = north;
        out.y = -east;
        out.z = up;
        break;
    }

    return out;
  }

  //--------------------------------------------------------------------------
  Point GeoProj::local_to_ecef(const Point& in, const CoordinateType type) const
  {
    double north, east, up;

    switch(type)
    {
      case ENU:
        east  = in.x;
        north = in.y;
        up    = in.z;
        break;

      case NED:
        north = in.x;
        east  = in.y;
        up    = -in.z;
        break;

      case NWU:
        north = in.x;
        east  = -in.y;
        up    = in.z;
        break;
    }

    // Use transpose of projection matrix PM_
    double dx = PM_[0][0]*east + PM_[1][0]*north + PM_[2][0]*up;
    double dy = PM_[0][1]*east + PM_[1][1]*north + PM_[2][1]*up;
    double dz =                  PM_[1][2]*north + PM_[2][2]*up;

    Point out;
    out.x = dx + ref_ecef_.x;
    out.y = dy + ref_ecef_.y;
    out.z = dz + ref_ecef_.z;

    return out;
  }

  //--------------------------------------------------------------------------
  Point GeoProj::spherical_to_local(const GeoPoint& in, const CoordinateType type) const
  {
    auto p_ecef = spherical_to_ecef(in);
    return ecef_to_local(p_ecef, type);
  }

  //--------------------------------------------------------------------------
  GeoPoint GeoProj::local_to_spherical(const Point& in, const CoordinateType type) const
  {
    auto p_ecef = local_to_ecef(in, type);
    return ecef_to_spherical(p_ecef);
  }

  //--------------------------------------------------------------------------
  std::ostream& operator<<(std::ostream& o, const Point& p)
  {
    o << p.toString();
    return o;
  }

  //--------------------------------------------------------------------------
  std::ostream& operator<<(std::ostream& o, const GeoPoint& p)
  {
    o << p.toString();
    return o;
  }

  //--------------------------------------------------------------------------
  double distance(const Point& a, const Point& b)
  {
    double dx = b.x-a.x;
    double dy = b.y-a.y;
    double dz = b.z-a.z;
    return std::sqrt(dx*dx + dy*dy +dz*dz);
  }

  //--------------------------------------------------------------------------
  double distance(const GeoPoint &a, const GeoPoint &b)
  {
    // Projection using a as origin
    auto proj = GeoProj(WGS84, a);
    // Projection of b in local ENU with a as origin
    auto c = proj.spherical_to_local(b, ENU);
    return std::sqrt(c.x * c.x + c.y * c.y + c.z * c.z);
  }

}  // namespace cs_tools
