#ifndef MARKER_PUBLISHER_H
#define MARKER_PUBLISHER_H

#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"
#include <kdl/frames.hpp>
#include "narrowphase.h"

int publishSinglePointMarker(ros::Publisher &pub, int m_id, const KDL::Vector &pos, double r, double g, double b, double size);
int publishLineMarker(ros::Publisher &pub, int m_id, const KDL::Vector &pos1, const KDL::Vector &pos2, double r, double g, double b);
int publishCapsule(ros::Publisher &pub, int m_id, KDL::Frame fr, double length, double radius);
int publishCylinder(ros::Publisher &pub, int m_id, KDL::Frame fr, double length, double radius);
int publishMeshMarker(ros::Publisher &pub, int m_id, const KDL::Frame &tf, const fcl_2::Vec3f *points, int num_planes, const int *polygons, double r, double g, double b);
void clearMarkers(ros::Publisher &pub, int from, int to);

#endif	// MARKER_PUBLISHER_H


