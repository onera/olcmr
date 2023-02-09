# cs_tools: converts between coordinate systems

Simple C++ library for conversion between coordinate systems with Python3 binding.

Position conversion between:
- sperical geodetic system (latitude, longitude, altitude) using WGS84 Datum
- cartesian Earth Centred Earth Fixed (ECEF) system
- cartesian Local Tangent Plane (LTP) using following conventions:
     - ENU : East (X), North (Y), Up (Z)
     - NED : North (X), East (Y), Down (Z)
     - NWU : North (X), West (Y), Up (Z)

Orientation:
- conversion between Quaternion and Euler angles
- transforms Quaternion between ENU, NED and NWU LTP conventions

## Dependencies

For python binding

    sudo apt install python3-dev python3-pybind11

## Sources: 
- https://geodesie.ign.fr/contenu/fichiers/documentation/pedagogiques/TransformationsCoordonneesGeodesiques.pdf
- https://en.wikipedia.org/wiki/Geographic_coordinate_conversion
