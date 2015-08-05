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

/** \author Dawid Seredynski */

#ifndef URDF_COLLISION_PARSER_H
#define URDF_COLLISION_PARSER_H

#include "ros/ros.h"
#include <kdl/frames.hpp>
#include <kdl/tree.hpp>
#include <tinyxml.h>
#include "visualization_msgs/MarkerArray.h"

namespace fcl_2
{
class ShapeBase;
class GJKSolver_indep;
}

namespace self_collision
{

class Geometry
{
public:
	enum {UNDEFINED, CAPSULE, CONVEX, SPHERE};
	int type;
	boost::shared_ptr<fcl_2::ShapeBase> shape;
	Geometry(int type);
	virtual void clear() = 0;
	virtual void addMarkers(visualization_msgs::MarkerArray &marker_array) = 0;
	virtual void updateMarkers(visualization_msgs::MarkerArray &marker_array, const KDL::Frame &fr) = 0;
	int marker_id_;
private:
};

class Capsule : public Geometry
{
public:
	Capsule();
	double radius;
	double length;
	virtual void clear();
	virtual void addMarkers(visualization_msgs::MarkerArray &marker_array);
	virtual void updateMarkers(visualization_msgs::MarkerArray &marker_array, const KDL::Frame &fr);
private:
};

class Sphere : public Geometry
{
public:
	Sphere();
	double radius;
	virtual void clear();
	virtual void addMarkers(visualization_msgs::MarkerArray &marker_array);
	virtual void updateMarkers(visualization_msgs::MarkerArray &marker_array, const KDL::Frame &fr);
private:
};

class Convex : public Geometry
{
public:
	Convex();
	~Convex();
	void updateConvex(int num_points, const std::vector<geometry_msgs::Point> &points, int num_planes, const std::vector<int> &polygons);

	typedef std::vector<std::pair<std::string, KDL::Vector> > ConvexPointsStrVector;
	typedef std::vector<std::pair<int, KDL::Vector> > ConvexPointsIdVector;
	ConvexPointsStrVector points_str_;
	ConvexPointsIdVector points_id_;
	KDL::Vector center_;
	double radius_;
	virtual void clear();
	virtual void addMarkers(visualization_msgs::MarkerArray &marker_array);
	virtual void updateMarkers(visualization_msgs::MarkerArray &marker_array, const KDL::Frame &fr);
private:
};

class Link;

class Collision
{
public:
	void clear();
	boost::shared_ptr< Geometry > geometry;
	KDL::Frame origin;
	boost::shared_ptr<Link> parent_;
private:
};

class Link
{
public:
	void clear();
	std::string name;
	int index_;
	int parent_index_;
	const KDL::TreeElement *kdl_segment_;
	typedef std::vector< boost::shared_ptr< Collision > > VecPtrCollision;
	VecPtrCollision collision_array;
private:
};

class CollisionModel
{
public:
	static boost::shared_ptr<CollisionModel> parseURDF(const std::string &xml_string);
	void parseSRDF(const std::string &xml_string);

	std::string name_;

	typedef std::vector<std::pair<int, int> > CollisionPairs;
	CollisionPairs disabled_collisions;
	CollisionPairs enabled_collisions;

	boost::shared_ptr< const Link > getLink(int id);
	int getLinkIndex(const std::string &name);
	void generateCollisionPairs();
	static double getDistance(const Geometry &geom1, const KDL::Frame &tf1, const Geometry &geom2, const KDL::Frame &tf2, KDL::Vector &d1_out, KDL::Vector &d2_out, double d0);

	boost::shared_ptr<Link> *links_;
	int link_count_;
	int root_index_;

private:
	CollisionModel();

	bool parseDisableCollision(std::string &link1, std::string &link2, TiXmlElement *c);

	static bool parsePose(KDL::Frame &pose, TiXmlElement* xml);
	static bool parseSphere(Sphere &s, TiXmlElement *c);
	static bool parseCapsule(Capsule &s, TiXmlElement *c);
	static bool parsePoint(std::string &frame, KDL::Vector &p, TiXmlElement *c);
	static bool parseConvex(Convex &s, TiXmlElement *c);
	static boost::shared_ptr<Geometry> parseGeometry(TiXmlElement *g);
	static bool parseCollision(Collision &col, TiXmlElement* config);
	static bool parseLink(Link &link, TiXmlElement* config);

	static fcl_2::GJKSolver_indep gjk_solver;
};

}	// namespace self_collision

#endif	// URDF_COLLISION_PARSER_H

