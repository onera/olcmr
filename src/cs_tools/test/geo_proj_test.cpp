#include <iostream>
#include <iomanip>
#include <cs_tools/geo_proj.hpp>

int main(void)
{
  cs_tools::GeoPoint origin;

  // caylus camp
  origin.latitude = 44.2743;
  origin.longitude = 1.7285;
  origin.altitude = 361.0;
  std::cout<<std::fixed<<"Origin:"<<std::endl
           <<origin<<std::endl;

  std::cout<<"===================== ECEF =========================="<<std::endl;
  auto proj = cs_tools::GeoProj(cs_tools::WGS84, origin);
  auto o_ecef = proj.spherical_to_ecef(origin);
  std::cout<<"Origin in ECEF:"<<std::endl
           <<std::fixed<<o_ecef<<std::endl;

  auto o_lla = proj.ecef_to_spherical(o_ecef);
  std::cout<<"Origin in LLA:"<<std::endl<<std::fixed<<o_lla<<std::endl;

  auto p_ecef = o_ecef;
  p_ecef.x +=10;
  p_ecef.y +=10;
  p_ecef.z +=10;
  std::cout<<"add offset (10.0, 10.0, 10.0):"<<std::endl<<p_ecef<<std::endl;

  auto p_lla = proj.ecef_to_spherical(p_ecef);
  std::cout<<"Offset in LLA:"<<std::endl<<std::fixed<<p_lla<<std::endl;

  std::cout<<"===================== LPT =========================="<<std::endl;
  cs_tools::Point p = {10.0, 10.0, 2.0};
  std::cout<<"Local position:"<<std::endl<<p<<std::endl;

  p_ecef =  proj.local_to_ecef(p, cs_tools::ENU);
  std::cout<<"Local position in ecef:"<<std::endl<<p_ecef<<std::endl;

  p_lla =  proj.ecef_to_spherical(p_ecef);
  std::cout<<"Local position in LLA:"<<std::endl<<p_lla<<std::endl;

  // Direct
  p_lla = proj.local_to_spherical(p, cs_tools::ENU);
  std::cout<<"Local position in LLA (direct):"<<std::endl<<p_lla<<std::endl;

  auto p_local = proj.spherical_to_local(p_lla, cs_tools::ENU);
  std::cout<<"Back to local position (direct):"<<std::endl<<p_local<<std::endl;

  return 0;
}
