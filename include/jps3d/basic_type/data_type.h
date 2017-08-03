/**
 * @file data_type.h
 * @brief Defines all data types used in jps3d lib

 * Mostly alias from Eigen Library.
 */

#ifndef BASIC_DATA_H
#define BASIC_DATA_H
#include <stdio.h>
#include <math.h>
#include <limits>
#include <vector>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

///Set red font in printf funtion 
#define ANSI_COLOR_RED "\x1b[31m"
///Set green font in printf funtion 
#define ANSI_COLOR_GREEN "\x1b[32m"
///Set yellow font in printf funtion 
#define ANSI_COLOR_YELLOW "\x1b[33m"
///Set blue font in printf funtion 
#define ANSI_COLOR_BLUE "\x1b[34m"
///Set magenta font in printf funtion 
#define ANSI_COLOR_MAGENTA "\x1b[35m"
///Set cyan font in printf funtion 
#define ANSI_COLOR_CYAN "\x1b[36m"
///Reset font color in printf funtion 
#define ANSI_COLOR_RESET "\x1b[0m"

///Pre-allocated std::vector for Eigen.
template <typename T> 
using vec_E = std::vector<T, Eigen::aligned_allocator<T>>;
/*! \brief Rename the float type used in lib 

    Default is set to be double, but user can change it to float.
*/
typedef double decimal_t;

///Column vector in float of size 2.
typedef Eigen::Matrix<decimal_t, 2, 1> Vec2f;
///Column vector in int of size 2.
typedef Eigen::Vector2i Vec2i;
///Column vector in float of size 3.
typedef Eigen::Matrix<decimal_t, 3, 1> Vec3f;
///Column vector in int of size 3.
typedef Eigen::Vector3i Vec3i;
///Vector of type Vec2f.
typedef vec_E<Vec2f> vec_Vec2f;
///Vector of type Vec2i.
typedef vec_E<Vec2i> vec_Vec2i;
///Vector of type Vec3f.
typedef vec_E<Vec3f> vec_Vec3f;
///Vector of type Vec3i.
typedef vec_E<Vec3i> vec_Vec3i;
#endif
