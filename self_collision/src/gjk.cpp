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

#include "gjk.h"
#include "intersect.h"

namespace fcl_2
{

namespace details
{

Vec3f getSupport(const ShapeBase* shape, const Vec3f& dir)
{
  switch(shape->getNodeType())
  {
  case GEOM_TRIANGLE:
    {
      const TriangleP* triangle = static_cast<const TriangleP*>(shape);
      FCL_REAL dota = dir.dot(triangle->a);
      FCL_REAL dotb = dir.dot(triangle->b);
      FCL_REAL dotc = dir.dot(triangle->c);
      if(dota > dotb)
      {
        if(dotc > dota)
          return triangle->c;
        else
          return triangle->a;
      }
      else
      {
        if(dotc > dotb)
          return triangle->c;
        else
          return triangle->b;
      }
    }
    break;
  case GEOM_BOX:
    {
      const Box* box = static_cast<const Box*>(shape);
      return Vec3f((dir[0]>0)?(box->side[0]/2):(-box->side[0]/2),
                   (dir[1]>0)?(box->side[1]/2):(-box->side[1]/2),
                   (dir[2]>0)?(box->side[2]/2):(-box->side[2]/2));
    }
    break;
  case GEOM_SPHERE:
    {
      const Sphere* sphere = static_cast<const Sphere*>(shape);
      return dir * sphere->radius;
    }
    break;
  case GEOM_CAPSULE:
    {
      const Capsule* capsule = static_cast<const Capsule*>(shape);
      FCL_REAL half_h = capsule->lz * 0.5;
      Vec3f pos1(0, 0, half_h);
      Vec3f pos2(0, 0, -half_h);
      Vec3f v = dir * capsule->radius;
      pos1 += v;
      pos2 += v;
      if(dir.dot(pos1) > dir.dot(pos2))
        return pos1;
      else return pos2;
    }
    break;
  case GEOM_CONE:
    {
      const Cone* cone = static_cast<const Cone*>(shape);
      FCL_REAL zdist = dir[0] * dir[0] + dir[1] * dir[1];
      FCL_REAL len = zdist + dir[2] * dir[2];
      zdist = std::sqrt(zdist);
      len = std::sqrt(len);
      FCL_REAL half_h = cone->lz * 0.5;
      FCL_REAL radius = cone->radius;

      FCL_REAL sin_a = radius / std::sqrt(radius * radius + 4 * half_h * half_h);

      if(dir[2] > len * sin_a)
        return Vec3f(0, 0, half_h);
      else if(zdist > 0)
      {
        FCL_REAL rad = radius / zdist;
        return Vec3f(rad * dir[0], rad * dir[1], -half_h);
      }
      else
        return Vec3f(0, 0, -half_h);
    }
    break;
  case GEOM_CYLINDER:
    {
      const Cylinder* cylinder = static_cast<const Cylinder*>(shape);
      FCL_REAL zdist = std::sqrt(dir[0] * dir[0] + dir[1] * dir[1]);
      FCL_REAL half_h = cylinder->lz * 0.5;
      if(zdist == 0.0)
      {
        return Vec3f(0, 0, (dir[2]>0)? half_h:-half_h);
      }
      else
      {
        FCL_REAL d = cylinder->radius / zdist;
        return Vec3f(d * dir[0], d * dir[1], (dir[2]>0)?half_h:-half_h);
      }
    }
    break;
  case GEOM_CONVEX:
    {
      const Convex* convex = static_cast<const Convex*>(shape);
      FCL_REAL maxdot = - std::numeric_limits<FCL_REAL>::max();
      Vec3f* curp = convex->points;
      Vec3f bestv;
      for(int i = 0; i < convex->num_points; ++i, curp+=1)
      {
        FCL_REAL dot = dir.dot(*curp);
        if(dot > maxdot)
        {
          bestv = *curp;
          maxdot = dot;
        }
      }
      return bestv;
    }
    break;
  case GEOM_PLANE:
	break;
  default:
    ; // nothing
  }

  return Vec3f(0, 0, 0);
}

void GJK::initialize()
{
  ray = Vec3f();
  nfree = 0;
  status = Failed;
  current = 0;
  distance = 0.0;
  simplex = NULL;
}


Vec3f GJK::getGuessFromSimplex() const
{
  return ray;
}


GJK::Status GJK::evaluate(const MinkowskiDiff& shape_, const Vec3f& guess)
{
  size_t iterations = 0;
  FCL_REAL alpha = 0;
  Vec3f lastw[4];
  size_t clastw = 0;
    
  free_v[0] = &store_v[0];
  free_v[1] = &store_v[1];
  free_v[2] = &store_v[2];
  free_v[3] = &store_v[3];
    
  nfree = 4;
  current = 0;
  status = Valid;
  shape = shape_;
  distance = 0.0;
  simplices[0].rank = 0;
  ray = guess;

  appendVertex(simplices[0], (ray.sqrLength() > 0) ? -ray : Vec3f(1, 0, 0));
  simplices[0].p[0] = 1;
  ray = simplices[0].c[0]->w;
  lastw[0] = lastw[1] = lastw[2] = lastw[3] = ray; // cache previous support points, the new support point will compare with it to avoid too close support points

  do
  {
    size_t next = 1 - current;
    Simplex& curr_simplex = simplices[current];
    Simplex& next_simplex = simplices[next];

    // check A: when origin is near the existing simplex, stop
    FCL_REAL rl = ray.length();
    if(rl < tolerance) // mean origin is near the face of original simplex, return touch
    {
      status = Inside;
      break;
    }

    appendVertex(curr_simplex, -ray); // see below, ray points away from origin

    // check B: when the new support point is close to previous support points, stop (as the new simplex is degenerated)
    Vec3f& w = curr_simplex.c[curr_simplex.rank - 1]->w;
    bool found = false;
    for(size_t i = 0; i < 4; ++i)
    {
      if((w - lastw[i]).sqrLength() < tolerance)
      {
        found = true; break;
      }
    }

    if(found)
    {
      removeVertex(simplices[current]);
      break; 
    }
    else
    {
      lastw[clastw = (clastw+1)&3] = w;
    }

    // check C: when the new support point is close to the sub-simplex where the ray point lies, stop (as the new simplex again is degenerated)
    FCL_REAL omega = ray.dot(w) / rl;
    alpha = std::max(alpha, omega);
    if((rl - alpha) - tolerance * rl <= 0)
    {
      removeVertex(simplices[current]);
      break;
    }

    Project::ProjectResult project_res;
    switch(curr_simplex.rank)
    {
    case 2:
      project_res = Project::projectLineOrigin(curr_simplex.c[0]->w, curr_simplex.c[1]->w); break;
    case 3:
      project_res = Project::projectTriangleOrigin(curr_simplex.c[0]->w, curr_simplex.c[1]->w, curr_simplex.c[2]->w); break;
    case 4:
      project_res = Project::projectTetrahedraOrigin(curr_simplex.c[0]->w, curr_simplex.c[1]->w, curr_simplex.c[2]->w, curr_simplex.c[3]->w); break;
    }
      
    if(project_res.sqr_distance >= 0)
    {
      next_simplex.rank = 0;
      ray = Vec3f();
      current = next;
      for(size_t i = 0; i < curr_simplex.rank; ++i)
      {
        if(project_res.encode & (1 << i))
        {
          next_simplex.c[next_simplex.rank] = curr_simplex.c[i];
          next_simplex.p[next_simplex.rank++] = project_res.parameterization[i]; // weights[i];
          ray += curr_simplex.c[i]->w * project_res.parameterization[i]; // weights[i];
        }
        else
          free_v[nfree++] = curr_simplex.c[i];
      }
      if(project_res.encode == 15) status = Inside; // the origin is within the 4-simplex, collision
    }
    else
    {
      removeVertex(simplices[current]);
      break;
    }

    status = ((++iterations) < max_iterations) ? status : Failed;
      
  } while(status == Valid);

  simplex = &simplices[current];
  switch(status)
  {
  case Valid: distance = ray.length(); break;
  case Inside: distance = 0; break;
  default: break;
  }
  return status;
}

void GJK::getSupport(const Vec3f& d, SimplexV& sv) const
{
  sv.d = normalize(d);
  sv.w = shape.support(sv.d);
}

void GJK::getSupport(const Vec3f& d, const Vec3f& v, SimplexV& sv) const
{
  sv.d = normalize(d);
  sv.w = shape.support(sv.d, v);
}

void GJK::removeVertex(Simplex& simplex)
{
  free_v[nfree++] = simplex.c[--simplex.rank];
}

void GJK::appendVertex(Simplex& simplex, const Vec3f& v)
{
  simplex.p[simplex.rank] = 0; // initial weight 0
  simplex.c[simplex.rank] = free_v[--nfree]; // set the memory
  getSupport(v, *simplex.c[simplex.rank++]);
}

bool GJK::encloseOrigin()
{
  switch(simplex->rank)
  {
  case 1:
    {
      for(size_t i = 0; i < 3; ++i)
      {
        Vec3f axis;
        axis[i] = 1;
        appendVertex(*simplex, axis);
        if(encloseOrigin()) return true;
        removeVertex(*simplex);
        appendVertex(*simplex, -axis);
        if(encloseOrigin()) return true;
        removeVertex(*simplex);
      }
    }
    break;
  case 2:
    {
      Vec3f d = simplex->c[1]->w - simplex->c[0]->w;
      for(size_t i = 0; i < 3; ++i)
      {
        Vec3f axis;
        axis[i] = 1;
        Vec3f p = d.cross(axis);
        if(p.sqrLength() > 0)
        {
          appendVertex(*simplex, p);
          if(encloseOrigin()) return true;
          removeVertex(*simplex);
          appendVertex(*simplex, -p);
          if(encloseOrigin()) return true;
          removeVertex(*simplex);
        }
      }
    }
    break;
  case 3:
    {
      Vec3f n = (simplex->c[1]->w - simplex->c[0]->w).cross(simplex->c[2]->w - simplex->c[0]->w);
      if(n.sqrLength() > 0)
      {
        appendVertex(*simplex, n);
        if(encloseOrigin()) return true;
        removeVertex(*simplex);
        appendVertex(*simplex, -n);
        if(encloseOrigin()) return true;
        removeVertex(*simplex);
      }
    }
    break;
  case 4:
    {
      if(std::abs(triple(simplex->c[0]->w - simplex->c[3]->w, simplex->c[1]->w - simplex->c[3]->w, simplex->c[2]->w - simplex->c[3]->w)) > 0)
        return true;
    }
    break;
  }

  return false;
}

} // details

} // fcl
