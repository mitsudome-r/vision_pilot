cmake_minimum_required(VERSION 3.22.1)

find_package(PkgConfig REQUIRED)
pkg_check_modules(IPOPT REQUIRED ipopt)

# Prefer the system Eigen (libeigen3-dev >= 3.4, e.g. Ubuntu 22.04 / ROS 2 Humble) so that
# offline builds (colcon, CI without network) work. Fall back to fetching Eigen 5.0.0.
find_package(Eigen3 3.4 QUIET NO_MODULE)
if(NOT TARGET Eigen3::Eigen)
    include(FetchContent)

    FetchContent_Declare(
            eigen
            GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
            GIT_TAG 5.0.0
            GIT_SHALLOW TRUE
    )

    # Skip building tests/docs
    set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
    set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
    set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)

    # Install vendored Eigen headers to a private path so packaging doesn't
    # collide with the system libeigen3-dev package
    set(INCLUDE_INSTALL_DIR "include/visionpilot/eigen3" CACHE STRING "" FORCE)

    FetchContent_MakeAvailable(eigen)
endif()

add_library(planning
        src/planning.cpp
        src/longitudinal_planning.cpp
        src/lateral_planning.cpp

)

target_include_directories(planning
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        PRIVATE
        ${IPOPT_INCLUDE_DIRS}
)

target_compile_definitions(planning
        PRIVATE
        HAVE_CSTDDEF
)

target_link_libraries(planning
        PUBLIC
        common
        Eigen3::Eigen
        PRIVATE
        ${IPOPT_LIBRARIES}
)

# CppAD runtime library (libcppad-dev). The Docker build passes it through
# CMAKE_CXX_STANDARD_LIBRARIES; resolve it here so plain colcon builds link too.
find_library(CPPAD_LIB NAMES cppad_lib)
if(CPPAD_LIB)
    target_link_libraries(planning PRIVATE ${CPPAD_LIB})
endif()