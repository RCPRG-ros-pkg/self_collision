#ifndef URDF_COLLISION_PARSER_H
#define URDF_COLLISION_PARSER_H

#include "ros/ros.h"
//#include "std_msgs/String.h"
//#include "visualization_msgs/MarkerArray.h"
//#include "urdf/model.h"

//#include <iostream>
#include <kdl_parser/kdl_parser.hpp>
#include <tinyxml.h>

namespace self_collision
{

class Geometry
{
public:
	enum {CAPSULE, CONVEX};
	int type;
	virtual void clear() = 0;
private:
};

class Capsule : public Geometry
{
public:
	double radius;
	double length;
	virtual void clear();
private:
};

class Convex : public Geometry
{
public:
	typedef std::vector<std::pair<std::string, KDL::Vector> > ConvexPointsVector;
	ConvexPointsVector points;
	virtual void clear();
private:
};

class Collision
{
public:
	void clear();
	boost::shared_ptr< Geometry > geometry;
	KDL::Frame origin;
private:
};

class Link
{
public:
	void clear();
	std::string name;
	typedef std::vector< boost::shared_ptr< Collision > > VecPtrCollision;
	VecPtrCollision collision_array;
private:
};

class CollisionModel
{
public:
	static boost::shared_ptr<CollisionModel> parseURDF(const std::string &xml_string);
	std::string name_;

	boost::shared_ptr< const Link > getLink(const std::string &name);
private:
	static bool parsePose(KDL::Frame &pose, TiXmlElement* xml);
	static bool parseCapsule(Capsule &s, TiXmlElement *c);
	static bool parsePoint(std::string &frame, KDL::Vector &p, TiXmlElement *c);
	static bool parseConvex(Convex &s, TiXmlElement *c);
	static boost::shared_ptr<Geometry> parseGeometry(TiXmlElement *g);
	static bool parseCollision(Collision &col, TiXmlElement* config);
	static bool parseLink(Link &link, TiXmlElement* config);

	typedef std::map<std::string, boost::shared_ptr<Link> > LinkMap;
	LinkMap links_;
};

}	// namespace self_collision

#endif	// URDF_COLLISION_PARSER_H

