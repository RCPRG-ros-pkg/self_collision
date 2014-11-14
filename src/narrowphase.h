/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Jia Pan */

/*
This code is from fcl library with some modifications.
Origin:
https://github.com/flexible-collision-library/fcl
*/

#ifndef FCL_NARROWPHASE_H
#define FCL_NARROWPHASE_H

#include "gjk.h"

namespace fcl_2
{

/// @brief collision and distance solver based on GJK algorithm implemented in fcl (rewritten the code from the GJK in bullet)
struct GJKSolver_indep
{  
  /// @brief distance computation between two shapes
  template<typename S1, typename S2>
  bool shapeDistance(const S1& s1, const Transform3f& tf1,
                     const S2& s2, const Transform3f& tf2,
                     FCL_REAL* distance, Vec3f* p1, Vec3f* p2) const
  {
    Vec3f guess(1, 0, 0);
    if(enable_cached_guess) guess = cached_guess;
    details::MinkowskiDiff shape;
    shape.shapes[0] = &s1;
    shape.shapes[1] = &s2;
    shape.toshape1 = tf2.getRotation().transposeTimes(tf1.getRotation());
    shape.toshape0 = tf1.inverseTimes(tf2);

    details::GJK gjk(gjk_max_iterations, gjk_tolerance);
    details::GJK::Status gjk_status = gjk.evaluate(shape, -guess);
    if(enable_cached_guess) cached_guess = gjk.getGuessFromSimplex();

    if(gjk_status == details::GJK::Valid)
    {
      Vec3f w0, w1;
      for(size_t i = 0; i < gjk.getSimplex()->rank; ++i)
      {
        FCL_REAL p = gjk.getSimplex()->p[i];
        w0 += shape.support(gjk.getSimplex()->c[i]->d, 0) * p;
        w1 += shape.support(-gjk.getSimplex()->c[i]->d, 1) * p;
      }

      if(distance) *distance = (w0 - w1).length();
      
      if(p1) *p1 = w0;
      if(p2) *p2 = shape.toshape0.transform(w1);
      
      return true;
    }
    else
    {
      if(distance) *distance = -1;
      return false;
    }
  }

  template<typename S1, typename S2>
  bool shapeDistance(const S1& s1, const Transform3f& tf1,
                     const S2& s2, const Transform3f& tf2,
                     FCL_REAL* distance) const
  {
    return shapeDistance(s1, tf1, s2, tf2, distance, NULL, NULL);
  }

  /// @brief distance computation between one shape and a triangle
  template<typename S>
  bool shapeTriangleDistance(const S& s, const Transform3f& tf,
                             const Vec3f& P1, const Vec3f& P2, const Vec3f& P3,
                             FCL_REAL* distance, Vec3f* p1, Vec3f* p2) const
  {
    TriangleP tri(P1, P2, P3);
    Vec3f guess(1, 0, 0);
    if(enable_cached_guess) guess = cached_guess;

    details::MinkowskiDiff shape;
    shape.shapes[0] = &s;
    shape.shapes[1] = &tri;
    shape.toshape1 = tf.getRotation();
    shape.toshape0 = inverse(tf);

    details::GJK gjk(gjk_max_iterations, gjk_tolerance);
    details::GJK::Status gjk_status = gjk.evaluate(shape, -guess);
    if(enable_cached_guess) cached_guess = gjk.getGuessFromSimplex();
    
    if(gjk_status == details::GJK::Valid)
    {
      Vec3f w0, w1;
      for(size_t i = 0; i < gjk.getSimplex()->rank; ++i)
      {
        FCL_REAL p = gjk.getSimplex()->p[i];
        w0 += shape.support(gjk.getSimplex()->c[i]->d, 0) * p;
        w1 += shape.support(-gjk.getSimplex()->c[i]->d, 1) * p;
      }

      if(distance) *distance = (w0 - w1).length();
      if(p1) *p1 = w0;
      if(p2) *p2 = shape.toshape0.transform(w1);
      return true;
    }
    else
    {
      if(distance) *distance = -1;
      return false;
    }
  }

  template<typename S>
  bool shapeTriangleDistance(const S& s, const Transform3f& tf,
                             const Vec3f& P1, const Vec3f& P2, const Vec3f& P3,
                             FCL_REAL* distance) const
  {
    return shapeTriangleDistance(s, tf, P1, P2, P3, distance, NULL, NULL);
  }
  
  /// @brief distance computation between one shape and a triangle with transformation
  template<typename S>
  bool shapeTriangleDistance(const S& s, const Transform3f& tf1,
                             const Vec3f& P1, const Vec3f& P2, const Vec3f& P3, const Transform3f& tf2,
                             FCL_REAL* distance, Vec3f* p1, Vec3f* p2) const
  {
    TriangleP tri(P1, P2, P3);
    Vec3f guess(1, 0, 0);
    if(enable_cached_guess) guess = cached_guess;
    
    details::MinkowskiDiff shape;
    shape.shapes[0] = &s;
    shape.shapes[1] = &tri;
    shape.toshape1 = tf2.getRotation().transposeTimes(tf1.getRotation());
    shape.toshape0 = tf1.inverseTimes(tf2);

    details::GJK gjk(gjk_max_iterations, gjk_tolerance);
    details::GJK::Status gjk_status = gjk.evaluate(shape, -guess);
    if(enable_cached_guess) cached_guess = gjk.getGuessFromSimplex();

    if(gjk_status == details::GJK::Valid)
    {
      Vec3f w0, w1;
      for(size_t i = 0; i < gjk.getSimplex()->rank; ++i)
      {
        FCL_REAL p = gjk.getSimplex()->p[i];
        w0 += shape.support(gjk.getSimplex()->c[i]->d, 0) * p;
        w1 += shape.support(-gjk.getSimplex()->c[i]->d, 1) * p;
      }

      if(distance) *distance = (w0 - w1).length();
      if(p1) *p1 = w0;
      if(p2) *p2 = shape.toshape0.transform(w1);
      return true;
    }
    else
    {
      if(distance) *distance = -1;
      return false;
    }
  }

  template<typename S>
  bool shapeTriangleDistance(const S& s, const Transform3f& tf1,
                             const Vec3f& P1, const Vec3f& P2, const Vec3f& P3, const Transform3f& tf2,
                             FCL_REAL* distance) const
  {
    return shapeTriangleDistance(s, tf1, P1, P2, P3, tf2, distance, NULL, NULL);
  }
  
  /// @brief default setting for GJK algorithm
  GJKSolver_indep()
  {
    gjk_max_iterations = 128;
    gjk_tolerance = 1e-6;
    epa_max_face_num = 128;
    epa_max_vertex_num = 64;
    epa_max_iterations = 255;
    epa_tolerance = 1e-6;
    enable_cached_guess = false;
    cached_guess = Vec3f(1, 0, 0);
  }

  void enableCachedGuess(bool if_enable) const
  {
    enable_cached_guess = if_enable;
  }

  void setCachedGuess(const Vec3f& guess) const
  {
    cached_guess = guess;
  }

  Vec3f getCachedGuess() const
  {
    return cached_guess;
  }

  /// @brief maximum number of simplex face used in EPA algorithm
  unsigned int epa_max_face_num;

  /// @brief maximum number of simplex vertex used in EPA algorithm
  unsigned int epa_max_vertex_num;

  /// @brief maximum number of iterations used for EPA iterations
  unsigned int epa_max_iterations;

  /// @brief the threshold used in EPA to stop iteration
  FCL_REAL epa_tolerance;

  /// @brief the threshold used in GJK to stop iteration
  FCL_REAL gjk_tolerance;

  /// @brief maximum number of iterations used for GJK iterations
  FCL_REAL gjk_max_iterations;

  /// @brief Whether smart guess can be provided
  mutable bool enable_cached_guess;

  /// @brief smart guess
  mutable Vec3f cached_guess;
};


template<>
bool GJKSolver_indep::shapeDistance<Sphere, Capsule>(const Sphere& s1, const Transform3f& tf1,
                                                     const Capsule& s2, const Transform3f& tf2,
                                                     FCL_REAL* dist, Vec3f* p1, Vec3f* p2) const;

// @brief Computation of the distance result for capsule capsule. Closest points are based on two line-segments.
 template<>
   bool GJKSolver_indep::shapeDistance<Capsule, Capsule>(const Capsule& s1, const Transform3f& tf1,
							 const Capsule& s2, const Transform3f& tf2,
							 FCL_REAL* dist, Vec3f* p1, Vec3f* p2) const;


/// @brief Fast implementation for sphere-sphere distance
template<>
bool GJKSolver_indep::shapeDistance<Sphere, Sphere>(const Sphere& s1, const Transform3f& tf1,
                                                    const Sphere& s2, const Transform3f& tf2,
                                                    FCL_REAL* dist, Vec3f* p1, Vec3f* p2) const;

/// @brief Fast implementation for sphere-triangle distance
template<>
bool GJKSolver_indep::shapeTriangleDistance<Sphere>(const Sphere& s, const Transform3f& tf,
                                                    const Vec3f& P1, const Vec3f& P2, const Vec3f& P3, 
                                                    FCL_REAL* dist, Vec3f* p1, Vec3f* p2) const;

/// @brief Fast implementation for sphere-triangle distance
template<> 
bool GJKSolver_indep::shapeTriangleDistance<Sphere>(const Sphere& s, const Transform3f& tf1, 
                                                    const Vec3f& P1, const Vec3f& P2, const Vec3f& P3, const Transform3f& tf2,
                                                    FCL_REAL* dist, Vec3f* p1, Vec3f* p2) const;


}

#endif
