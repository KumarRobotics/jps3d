/**
 * @file data_type.h
 * @brief Defines all data types used in jps3d lib

 * Mostly alias from Eigen Library.
 */

#include <stdio.h>
#include <limits>
#include <vector>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#ifndef BASIC_COLOR_H
#define BASIC_COLOR_H
///Set red font in printf funtion 
#define ANSI_COLOR_RED "\x1b[1;31m"
///Set green font in printf funtion 
#define ANSI_COLOR_GREEN "\x1b[1;32m"
///Set yellow font in printf funtion 
#define ANSI_COLOR_YELLOW "\x1b[1;33m"
///Set blue font in printf funtion 
#define ANSI_COLOR_BLUE "\x1b[1;34m"
///Set magenta font in printf funtion 
#define ANSI_COLOR_MAGENTA "\x1b[1;35m"
///Set cyan font in printf funtion 
#define ANSI_COLOR_CYAN "\x1b[1;36m"
///Reset font color in printf funtion 
#define ANSI_COLOR_RESET "\x1b[0m"
#endif

#ifndef BASIC_DATA_H
#define BASIC_DATA_H
typedef double decimal_t;
///Pre-allocated std::vector for Eigen using vec_E
template <typename T> 
using vec_E = std::vector<T, Eigen::aligned_allocator<T>>;
///Eigen 1D float vector
template <int N> 
using Vecf = Eigen::Matrix<decimal_t, N, 1>;
///Eigen 1D int vector
template <int N> 
using Veci = Eigen::Matrix<int, N, 1>;
template <int N> 
using vec_Vecf = vec_E<Vecf<N>>;
///Vector of Eigen 1D int vector
template <int N> 
using vec_Veci = vec_E<Veci<N>>;

///Eigen 1D float vector of size 2
typedef Vecf<2> Vec2f;
///Eigen 1D int vector of size 2
typedef Veci<2> Vec2i;
///Eigen 1D float vector of size 3
typedef Vecf<3> Vec3f;
///Eigen 1D int vector of size 3
typedef Veci<3> Vec3i;

///Vector of type Vec2f.
typedef vec_E<Vec2f> vec_Vec2f;
///Vector of type Vec2i.
typedef vec_E<Vec2i> vec_Vec2i;
///Vector of type Vec3f.
typedef vec_E<Vec3f> vec_Vec3f;
///Vector of type Vec3i.
typedef vec_E<Vec3i> vec_Vec3i;
#endif

