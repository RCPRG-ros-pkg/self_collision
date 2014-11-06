#ifndef URDF_COLLISION_PARSER_H
#define URDF_COLLISION_PARSER_H

#include "ros/ros.h"
#include <kdl/frames.hpp>
#include <kdl/tree.hpp>
#include <tinyxml.h>
#include "narrowphase.h"
#include "qhull_interface.h"
#include "marker_publisher.h"

namespace self_collision
{

class Geometry
{
public:
	enum {UNDEFINED, CAPSULE, CONVEX};
	int type;
	boost::shared_ptr<fcl_2::ShapeBase> shape;
	Geometry(int type);
	virtual void clear() = 0;
	virtual int publishMarker(ros::Publisher &pub, int m_id, const KDL::Frame tf) = 0;
private:
};

class Capsule : public Geometry
{
public:
	Capsule();
	double radius;
	double length;
	virtual void clear();
	virtual int publishMarker(ros::Publisher &pub, int m_id, const KDL::Frame tf);
private:
};

class Convex : public Geometry
{
public:
	Convex();
	~Convex();
	void updateConvex(const std::vector<KDL::Vector> &v, const std::vector<Face> &f);

	typedef std::vector<std::pair<std::string, KDL::Vector> > ConvexPointsStrVector;
	typedef std::vector<std::pair<int, KDL::Vector> > ConvexPointsIdVector;
	ConvexPointsStrVector points_str_;
	ConvexPointsIdVector points_id_;
	KDL::Vector center_;
	double radius_;
	virtual void clear();
	virtual int publishMarker(ros::Publisher &pub, int m_id, const KDL::Frame tf);
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
	int id_;
	int parent_id_;
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
	int getLinkId(const std::string &name);
	void generateCollisionPairs();
	static double getDistance(const Geometry &geom1, const KDL::Frame &tf1, const Geometry &geom2, const KDL::Frame &tf2, KDL::Vector &d1_out, KDL::Vector &d2_out, double d0);

	boost::shared_ptr<Link> *links_;
	int link_count_;

private:
	CollisionModel();

	bool parseDisableCollision(std::string &link1, std::string &link2, TiXmlElement *c);

	static bool parsePose(KDL::Frame &pose, TiXmlElement* xml);
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

