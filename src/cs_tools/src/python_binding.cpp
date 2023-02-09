#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>
#include "cs_tools/common.hpp"
#include "cs_tools/geo_proj.hpp"
#include "cs_tools/quaternion.hpp"

namespace py = pybind11;
namespace cs = cs_tools;


PYBIND11_MODULE(_cs_tools, m)
{
    py::enum_<cs::CoordinateType>(m, "CoordinateType")
        .value("ENU", cs::CoordinateType::ENU)
        .value("NED", cs::CoordinateType::NED)
        .value("NWU", cs::CoordinateType::NWU)
        .export_values();

    py::enum_<cs::SurfaceType>(m, "SurfaceType")
        .value("WGS84", cs::SurfaceType::WGS84)
        .export_values();

    m.def("radian", static_cast<double(*)(double)>(&cs::radian), "Convert from degree to radian");
    m.def("degree", static_cast<double(*)(double)>(&cs::degree), "Convert from radian to degree");

    py::class_<cs::GeoPoint>(m, "GeoPoint")
        .def(py::init())
        .def(py::init<double, double, double>())
        .def_readwrite("latitude", &cs::GeoPoint::latitude)
        .def_readwrite("longitude", &cs::GeoPoint::longitude)
        .def_readwrite("altitude", &cs::GeoPoint::altitude)
        .def("__str__", [](const cs::GeoPoint &p) {return p.toString();})
        .def("__repr__", [](const cs::GeoPoint &p) {return p.toString();});

    py::class_<cs::Point>(m, "Point")
        .def(py::init())
        .def(py::init<double, double, double>())
        .def_readwrite("x", &cs::Point::x)
        .def_readwrite("y", &cs::Point::y)
        .def_readwrite("z", &cs::Point::z)
        .def("__str__", [](const cs::Point &p) {return p.toString();})
        .def("__repr__", [](const cs::Point &p) {return p.toString();});

    m.def("distance", static_cast<double(*)(const cs::Point&, const cs::Point&)>(&cs::distance), "Compute distance between two points");
    m.def("distance", static_cast<double(*)(const cs::GeoPoint&, const cs::GeoPoint&)>(&cs::distance), "Compute distance between two geo points");

    py::class_<cs::GeoProj>(m, "GeoProj")
        .def(py::init<const cs::SurfaceType, const cs::GeoPoint &>())
        .def("set_reference", &cs::GeoProj::set_reference, "Set reference in geodetic coordinates")
        .def("get_reference", &cs::GeoProj::get_reference, "Get reference geodetic coordinates")
        .def("spherical_to_ecef", &cs::GeoProj::spherical_to_ecef, "Convert geodetic to ECEF coordinates")
        .def("ecef_to_spherical", &cs::GeoProj::ecef_to_spherical, "Convert ECEF to geodetic coordinates")
        .def("ecef_to_local", &cs::GeoProj::ecef_to_local, "Convert ECEF to local coordinates")
        .def("local_to_ecef", &cs::GeoProj::local_to_ecef, "Convert local to ECEF coordinates")
        .def("spherical_to_local", &cs::GeoProj::spherical_to_local, "Convert spherical to local coordinates")
        .def("local_to_spherical", &cs::GeoProj::local_to_spherical, "Convert local to spherical coordinates");

    py::class_<cs::Euler>(m, "Euler")
        .def(py::init())
        .def(py::init<double, double, double>())
        .def_readwrite("yaw", &cs::Euler::yaw)
        .def_readwrite("pitch", &cs::Euler::pitch)
        .def_readwrite("roll", &cs::Euler::roll)
        .def("__str__", [](const cs::Euler &e) {return e.toString();})
        .def("__repr__", [](const cs::Euler &e) {return e.toString();});

    m.def("radian", static_cast<cs::Euler(*)(const cs::Euler&)>(&cs::radian), "Convert from degree to radian");
    m.def("degree", static_cast<cs::Euler(*)(const cs::Euler&)>(&cs::degree), "Convert from radian to degree");


    py::class_<cs::Quaternion>(m, "Quaternion")
        .def(py::init(),"Basic constructor")
        .def(py::init<double, double, double, double>())
        .def(py::init<const cs::Euler &>())
        .def(py::init<double, std::array<double, 3>>())
        .def("w", &cs::Quaternion::w,"Accessor to w")
        .def("x", &cs::Quaternion::x,"Accessor to x")
        .def("y", &cs::Quaternion::y,"Accessor to y")
        .def("z", &cs::Quaternion::z,"Accessor to z")
        .def("euler", &cs::Quaternion::euler,"Convert Quaternion to Euler angles")
        .def(py::self + cs::Quaternion())
        .def(py::self - cs::Quaternion())
        .def(py::self * cs::Quaternion())
        .def(- py::self)
        .def("__str__", [](const cs::Quaternion &q) {return q.toString();})
        .def("__repr__", [](const cs::Quaternion &q) {return q.toString();});
}