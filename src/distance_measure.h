#ifndef DISTANCE_MEASURE_H
#define DISTANCE_MEASURE_H

#include "ros/ros.h"
#include <kdl/frames.hpp>
#include "narrowphase.h"
#include "marker_publisher.h"
#include "qhull_interface.h"

class DistanceMeasure
{
private:
	enum ObjectType {CAPSULE, CONVEX};
	typedef std::map<std::string, std::pair<ObjectType, fcl_2::ShapeBase*> > ObjectsMap;
public:
	DistanceMeasure();
	void addCapsule(const std::string &name, double radius, double length);
	void addConvex(std::string name);
	void updateConvex(std::string name, const std::vector<KDL::Vector> &v, const std::vector<Face> &f);
	double getDistance(const std::string &name1, const KDL::Frame &tf1, const std::string &name2, const KDL::Frame &tf2, KDL::Vector &d1_out, KDL::Vector &d2_out);
	int publishMarker(ros::Publisher &pub, int m_id, const std::string &name, const KDL::Frame tf);
private:
	fcl_2::GJKSolver_indep gjk_solver;
	ObjectsMap map_;
};

#endif	// DISTANCE_MEASURE_H


