cmake_minimum_required(VERSION 3.5)
project(cs_tools)

#=============================================================================
#  Common config
#=============================================================================

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Compiler flag (-fPIC need by pybind11)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

#=============================================================================
#  Specific Catkin or Ament config
#=============================================================================

if( CATKIN_DEVEL_PREFIX OR catkin_FOUND OR CATKIN_BUILD_BINARY_PACKAGE)
  set(COMPILING_WITH_CATKIN 1)

  message(STATUS "--------------------------------------")
  message(STATUS " cs_tools is being built using CATKIN ")
  message(STATUS "--------------------------------------")

  find_package(catkin REQUIRED)

  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES cs_tools
  )

  catkin_python_setup()

elseif( DEFINED ENV{AMENT_PREFIX_PATH})
  set(COMPILING_WITH_AMENT 1)

  message(STATUS "-------------------------------------")
  message(STATUS " cs_tools is being built using AMENT ")
  message(STATUS "-------------------------------------")

  # find dependencies
  find_package(ament_cmake REQUIRED)

endif()

#=============================================================================
# Common build
#=============================================================================

include_directories(include)

# C++ library
add_library(cs_tools
    src/geo_proj.cpp
    src/quaternion.cpp
    )

# C++ test
add_executable(geo_proj_test test/geo_proj_test)
target_link_libraries(geo_proj_test cs_tools)

# Python binding
find_package(pybind11 REQUIRED)
pybind11_add_module(_cs_tools MODULE src/python_binding.cpp)
target_link_libraries(_cs_tools PRIVATE cs_tools)

#=============================================================================
#  Specific Catkin or Ament install
#=============================================================================
if(COMPILING_WITH_CATKIN)

  # Install C++ library
  install(TARGETS cs_tools
    LIBRARY DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  # Install C++ test
  install(TARGETS
    geo_proj_test
    DESTINATION lib/${PROJECT_NAME}
  )

  # Install Python library
  set_target_properties(_cs_tools
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/${PROJECT_NAME}
  )

  # Install Python test scripts
  catkin_install_python(PROGRAMS
    scripts/geo_proj_test.py
    scripts/quaternion_test.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

elseif(COMPILING_WITH_AMENT)

  # Install library headers
  install(DIRECTORY include/${PROJECT_NAME}
    DESTINATION include
  )

  # Install C++ library
  install(TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    INCLUDES DESTINATION include
  )

  # Install C++ test
  install(TARGETS geo_proj_test
    DESTINATION lib/${PROJECT_NAME}
  )

  # Install python module
  ament_python_install_package(${PROJECT_NAME})
  install(TARGETS _cs_tools
    DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
  )

  # Install python test scripts
  install(PROGRAMS
    scripts/geo_proj_test.py
    scripts/quaternion_test.py
    DESTINATION lib/${PROJECT_NAME}
  )

  # ament export
  #ament_export_include_directories(include)
  #ament_export_libraries(cs_tools)
  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
  ament_package()
endif()
