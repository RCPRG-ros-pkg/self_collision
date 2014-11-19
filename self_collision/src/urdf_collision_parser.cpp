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

#include "ros/ros.h"
#include "self_collision/urdf_collision_parser.h"
#include "urdf/model.h"
#include <kdl/frames.hpp>
#include <tinyxml.h>
#include "narrowphase.h"

namespace self_collision
{

fcl_2::GJKSolver_indep CollisionModel::gjk_solver;

Geometry::Geometry(int type) :
	type(type)
{
}

Capsule::Capsule() :
	Geometry(CAPSULE)
{
}

void Capsule::clear()
{
	radius = 0.0;
	length = 0.0;
}

void Capsule::addMarkers(visualization_msgs::MarkerArray &marker_array)
{
	const fcl_2::Capsule* ob = static_cast<const fcl_2::Capsule*>(shape.get());

	marker_id_ = 0;
	if (marker_array.markers.size() > 0)
	{
		marker_id_ = marker_array.markers.back().id + 1;
	}

	visualization_msgs::Marker marker;
	marker.header.frame_id = "world";
	marker.ns = "default";
	marker.id = marker_id_;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.scale.x = ob->radius * 2.0;
	marker.scale.y = ob->radius * 2.0;
	marker.scale.z = ob->radius * 2.0;
	marker.color.a = 0.5;
	marker.color.r = 0.0;
	marker.color.g = 0.0;
	marker.color.b = 1.0;
	marker_array.markers.push_back(marker);

	visualization_msgs::Marker marker2;
	marker2.header.frame_id = "world";
	marker2.ns = "default";
	marker2.id = marker_id_ + 1;
	marker2.type = visualization_msgs::Marker::SPHERE;
	marker2.action = visualization_msgs::Marker::ADD;
	marker2.scale.x = ob->radius * 2.0;
	marker2.scale.y = ob->radius * 2.0;
	marker2.scale.z = ob->radius * 2.0;
	marker2.color.a = 0.5;
	marker2.color.r = 0.0;
	marker2.color.g = 0.0;
	marker2.color.b = 1.0;
	marker_array.markers.push_back(marker2);

	visualization_msgs::Marker marker3;
	marker3.header.frame_id = "world";
	marker3.ns = "default";
	marker3.id = marker_id_+2;
	marker3.type = visualization_msgs::Marker::CYLINDER;
	marker3.action = visualization_msgs::Marker::ADD;
	marker3.scale.x = ob->radius * 2.0;
	marker3.scale.y = ob->radius * 2.0;
	marker3.scale.z = ob->lz;
	marker3.color.a = 0.5;
	marker3.color.r = 0.0;
	marker3.color.g = 0.0;
	marker3.color.b = 1.0;
	marker_array.markers.push_back(marker3);

/*	visualization_msgs::Marker marker4;
	marker4.header.frame_id = "world";
	marker4.ns = "default";
	marker4.id = marker_id_+3;
	marker4.type = visualization_msgs::Marker::SPHERE;
	marker4.action = visualization_msgs::Marker::ADD;
	marker4.scale.x = 0.01;
	marker4.scale.y = 0.01;
	marker4.scale.z = 0.01;
	marker4.color.a = 1.0;
	marker4.color.r = 0.0;
	marker4.color.g = 1.0;
	marker4.color.b = 0.0;
	marker_array.markers.push_back(marker4);
*/
}

void Capsule::updateMarkers(visualization_msgs::MarkerArray &marker_array, const KDL::Frame &fr)
{
	const fcl_2::Capsule* ob = static_cast<const fcl_2::Capsule*>(shape.get());

	double qx, qy, qz, qw;
	fr.M.GetQuaternion(qx, qy, qz, qw);
	KDL::Vector v(0,0,ob->lz);
	KDL::Vector v2 = (fr * v) - (fr * KDL::Vector());

	if (marker_id_+3 >= marker_array.markers.size())
	{
		std::cout<<"Capsule::updateMarkers ERROR!"<<std::endl;
		*((int*)NULL) = 1;
	}

	if (marker_array.markers[marker_id_].id != marker_id_)
	{
		std::cout<<"Capsule::updateMarkers ERROR!"<<std::endl;
		*((int*)NULL) = 1;
	}
	marker_array.markers[marker_id_].header.stamp = ros::Time();
	marker_array.markers[marker_id_].pose.position.x = fr.p.x() - v2.x()/2.0;
	marker_array.markers[marker_id_].pose.position.y = fr.p.y() - v2.y()/2.0;
	marker_array.markers[marker_id_].pose.position.z = fr.p.z() - v2.z()/2.0;
	marker_array.markers[marker_id_].pose.orientation.x = qx;
	marker_array.markers[marker_id_].pose.orientation.y = qy;
	marker_array.markers[marker_id_].pose.orientation.z = qz;
	marker_array.markers[marker_id_].pose.orientation.w = qw;

	if (marker_array.markers[marker_id_+1].id != marker_id_+1)
	{
		std::cout<<"Capsule::updateMarkers ERROR!"<<std::endl;
		*((int*)NULL) = 1;
	}
	if (marker_array.markers[marker_id_+2].id != marker_id_+2)
	{
		std::cout<<"Capsule::updateMarkers ERROR!"<<std::endl;
	}
	if (marker_array.markers[marker_id_+3].id != marker_id_+3)
	{
		std::cout<<"Capsule::updateMarkers ERROR!"<<std::endl;
		*((int*)NULL) = 1;
	}
	marker_array.markers[marker_id_+1].header.stamp = ros::Time();
	marker_array.markers[marker_id_+1].pose.position.x = fr.p.x() + v2.x()/2.0;
	marker_array.markers[marker_id_+1].pose.position.y = fr.p.y() + v2.y()/2.0;
	marker_array.markers[marker_id_+1].pose.position.z = fr.p.z() + v2.z()/2.0;
	marker_array.markers[marker_id_+1].pose.orientation.x = qx;
	marker_array.markers[marker_id_+1].pose.orientation.y = qy;
	marker_array.markers[marker_id_+1].pose.orientation.z = qz;
	marker_array.markers[marker_id_+1].pose.orientation.w = qw;

	marker_array.markers[marker_id_+2].header.stamp = ros::Time();
	marker_array.markers[marker_id_+2].pose.position.x = fr.p.x();
	marker_array.markers[marker_id_+2].pose.position.y = fr.p.y();
	marker_array.markers[marker_id_+2].pose.position.z = fr.p.z();
	marker_array.markers[marker_id_+2].pose.orientation.x = qx;
	marker_array.markers[marker_id_+2].pose.orientation.y = qy;
	marker_array.markers[marker_id_+2].pose.orientation.z = qz;
	marker_array.markers[marker_id_+2].pose.orientation.w = qw;

/*	marker_array.markers[marker_id_+3].header.stamp = ros::Time();
	marker_array.markers[marker_id_+3].pose.position.x = fr.p.x();
	marker_array.markers[marker_id_+3].pose.position.y = fr.p.y();
	marker_array.markers[marker_id_+3].pose.position.z = fr.p.z();
*/
}

Convex::Convex() :
	Geometry(CONVEX)
{
	// allocate a lot of memory and initialize very simple convex hull mesh (4 points and 4 polygons)
	int num_points = 4;
	fcl_2::Vec3f* points = new fcl_2::Vec3f[5000];
	int num_planes = 4;
	int *polygons = new int[20000];
	points[0] = fcl_2::Vec3f(0.0, 0.0, 0.0);
	points[1] = fcl_2::Vec3f(0.1, 0.0, 0.0);
	points[2] = fcl_2::Vec3f(0.0, 0.1, 0.0);
	points[3] = fcl_2::Vec3f(0.0, 0.0, 0.1);
	polygons[0] = 3;	polygons[1] = 2;	polygons[2] = 1;	polygons[3] = 0;
	polygons[4] = 3;	polygons[5] = 3;	polygons[6] = 1;	polygons[7] = 0;
	polygons[8] = 3;	polygons[9] = 3;	polygons[10] = 2;	polygons[11] = 0;
	polygons[12] = 3;	polygons[13] = 3;	polygons[14] = 2;	polygons[15] = 1;
	shape.reset( static_cast<fcl_2::ShapeBase*>(new fcl_2::Convex(NULL, NULL, num_planes, points, num_points, polygons)) );
}

Convex::~Convex()
{
	fcl_2::Convex *conv = static_cast<fcl_2::Convex*>(shape.get());
	delete[] conv->polygons;
	conv->polygons = NULL;
	delete[] conv->points;
	conv->points = NULL;
}

void Convex::updateConvex(int num_points, const std::vector<geometry_msgs::Point> &points, int num_planes, const std::vector<int> &polygons)
{
	fcl_2::Convex *conv = static_cast<fcl_2::Convex*>(shape.get());
	int polygons_idx = 0;
	for (int f_idx=0; f_idx<num_planes; f_idx++)
	{
		polygons_idx += polygons[polygons_idx] + 1;
	}
	for (int i=0; i<polygons_idx; ++i)
	{
		conv->polygons[i] = polygons[i];
	}

	conv->num_planes = num_planes;

	for (int p_idx=0; p_idx<num_points; p_idx++)
	{
		conv->points[p_idx] = fcl_2::Vec3f(points[p_idx].x, points[p_idx].y, points[p_idx].z);
	}

	conv->num_points = num_points;
	conv->fillEdges();
}

void Convex::clear()
{
	points_str_.clear();
	points_id_.clear();
}

void Convex::addMarkers(visualization_msgs::MarkerArray &marker_array)
{
	const fcl_2::Convex* ob = static_cast<const fcl_2::Convex*>(shape.get());

	marker_id_ = 0;
	if (marker_array.markers.size() > 0)
	{
		marker_id_ = marker_array.markers.back().id + 1;
	}

	visualization_msgs::Marker marker;
	marker.header.frame_id = "world";
	marker.ns = "default";
	marker.id = marker_id_;
	marker.type = visualization_msgs::Marker::LINE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.points.resize(500);
	marker.scale.x = 0.005;
	marker.color.a = 1.0;
	marker.color.r = 0.0;
	marker.color.g = 0.0;
	marker.color.b = 1.0;
	marker_array.markers.push_back(marker);
}

void Convex::updateMarkers(visualization_msgs::MarkerArray &marker_array, const KDL::Frame &fr)
{
	const fcl_2::Convex* ob = static_cast<const fcl_2::Convex*>(shape.get());

	double qx, qy, qz, qw;
	fr.M.GetQuaternion(qx, qy, qz, qw);

	const fcl_2::Vec3f *points = ob->points;
	int num_planes = ob->num_planes;
	const int *polygons = ob->polygons;

	// edges
	marker_array.markers[marker_id_].header.stamp = ros::Time();
	marker_array.markers[marker_id_].pose.position.x = fr.p.x();
	marker_array.markers[marker_id_].pose.position.y = fr.p.y();
	marker_array.markers[marker_id_].pose.position.z = fr.p.z();
	marker_array.markers[marker_id_].pose.orientation.x = qx;
	marker_array.markers[marker_id_].pose.orientation.y = qy;
	marker_array.markers[marker_id_].pose.orientation.z = qz;
	marker_array.markers[marker_id_].pose.orientation.w = qw;
	int pt_size = marker_array.markers[marker_id_].points.size();
	int pt_idx = 0;
	int poly_idx = 0;
	for (int i=0; i<num_planes; i++)
	{
		int points_in_poly = polygons[poly_idx];
		for (int j=0; j<points_in_poly; j++)
		{
			geometry_msgs::Point pt1;
			pt1.x = points[polygons[poly_idx + 1 + j]][0];
			pt1.y = points[polygons[poly_idx + 1 + j]][1];
			pt1.z = points[polygons[poly_idx + 1 + j]][2];
			marker_array.markers[marker_id_].points[pt_idx++] = pt1;
			if (pt_idx >= pt_size)
			{
				break;
			}

			geometry_msgs::Point pt2;
			pt2.x = points[polygons[poly_idx + 1 + (j+1)%points_in_poly]][0];
			pt2.y = points[polygons[poly_idx + 1 + (j+1)%points_in_poly]][1];
			pt2.z = points[polygons[poly_idx + 1 + (j+1)%points_in_poly]][2];
			marker_array.markers[marker_id_].points[pt_idx++] = pt2;
			if (pt_idx >= pt_size)
			{
				break;
			}
		}
		if (pt_idx >= pt_size)
		{
			break;
		}
		poly_idx += points_in_poly+1;
	}

	for (; pt_idx<pt_size; pt_idx++)
	{
		marker_array.markers[marker_id_].points[pt_idx].x = 0;
		marker_array.markers[marker_id_].points[pt_idx].y = 0;
		marker_array.markers[marker_id_].points[pt_idx].z = 0;
	}
}

void Collision::clear()
{
	// TODO
}

void Link::clear()
{
	// TODO
}

boost::shared_ptr< const Link > CollisionModel::getLink(int id)
{
	if (id < 0 || id >= link_count_)
	{
		ROS_ERROR("CollisionModel::getLink: id out of range: 0 <= %d < %d", id, link_count_);
		return boost::shared_ptr< const Link >();
	}
	return links_[id];
}

int CollisionModel::getLinkIndex(const std::string &name)
{
	for (int l_i = 0; l_i < link_count_; l_i++)
	{
		if (links_[l_i]->name == name)
		{
			return l_i;
		}
	}
	return -1;
}

KDL::Vector initVectorFromString(const std::string &vector_str)
{
	std::vector<std::string> pieces;
	std::vector<double> xyz;
	boost::split( pieces, vector_str, boost::is_any_of(" "));
	for (unsigned int i = 0; i < pieces.size(); ++i){
		if (pieces[i] != "")
		{
			try
			{
				xyz.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
			}
			catch (boost::bad_lexical_cast &e)
			{
				ROS_ERROR("Vector3 xyz element (%s) is not a valid float",pieces[i].c_str());
				return KDL::Vector();
			}
		}
	}

	if (xyz.size() != 3)
	{
		ROS_ERROR("Vector contains %i elements instead of 3 elements", (int)xyz.size()); 
		return KDL::Vector();
	}
	return KDL::Vector(xyz[0], xyz[1], xyz[2]);
};

CollisionModel::CollisionModel() :
	links_(NULL),
	link_count_(0)
{
}

bool CollisionModel::parsePose(KDL::Frame &pose, TiXmlElement* xml)
{
	KDL::Vector pos;
	KDL::Vector rpy;
	if (xml)
	{
		const char* xyz_str = xml->Attribute("xyz");
		if (xyz_str != NULL)
		{
			try
			{
				pos = initVectorFromString(xyz_str);
			}
			catch (urdf::ParseError &e)
			{
				ROS_ERROR("%s", e.what());
				return false;
			}
		}
		const char* rpy_str = xml->Attribute("rpy");
		if (rpy_str != NULL)
		{
			try
			{
				rpy = initVectorFromString(rpy_str);
			}
			catch (urdf::ParseError &e)
			{
				ROS_ERROR("%s", e.what());
				return false;
			}
		}
	}
	KDL::Frame pose_ret( KDL::Rotation::RPY(rpy.x(), rpy.y(), rpy.z()), pos );
	pose = pose_ret;
	return true;
}

bool CollisionModel::parseCapsule(Capsule &s, TiXmlElement *c)
{
	s.clear();
	s.type = Geometry::CAPSULE;
	if (!c->Attribute("radius"))
	{
		ROS_ERROR("Capsule shape must have a radius attribute");
		return false;
	}
	try
	{
		s.radius = boost::lexical_cast<double>(c->Attribute("radius"));
	}
	catch (boost::bad_lexical_cast &e)
	{
		std::stringstream stm;
		stm << "radius [" << c->Attribute("radius") << "] is not a valid float: " << e.what();
		ROS_ERROR("%s", stm.str().c_str());
		return false;
	}

	if (!c->Attribute("length"))
	{
		ROS_ERROR("Capsule shape must have a length attribute");
		return false;
	}
	try
	{
		s.length = boost::lexical_cast<double>(c->Attribute("length"));
	}
	catch (boost::bad_lexical_cast &e)
	{
		std::stringstream stm;
		stm << "length [" << c->Attribute("length") << "] is not a valid float: " << e.what();
		ROS_ERROR("%s", stm.str().c_str());
		return false;
	}
	return true;
}

bool CollisionModel::parsePoint(std::string &frame, KDL::Vector &p, TiXmlElement *c)
{
	if (c)
	{
		const char* xyz_str = c->Attribute("xyz");
		if (xyz_str != NULL)
		{
			try
			{
				p = initVectorFromString(xyz_str);
			}
			catch (urdf::ParseError &e)
			{
				ROS_ERROR("%s", e.what());
				return false;
			}
		}
		const char* frame_str = c->Attribute("frame");
		if (frame_str != NULL)
		{
			try
			{
				frame = frame_str;
			}
			catch (urdf::ParseError &e)
			{
				ROS_ERROR("%s", e.what());
				return false;
			}
		}
	}
	return true;
}

bool CollisionModel::parseConvex(Convex &s, TiXmlElement *c)
{
	s.clear();
	s.type = Geometry::CONVEX;

	for (TiXmlElement* col_xml = c->FirstChildElement("point"); col_xml; col_xml = col_xml->NextSiblingElement("point"))
	{
		std::string frame;
		KDL::Vector point;
		if (parsePoint(frame, point, col_xml))
		{
			
			s.points_str_.push_back(std::make_pair<std::string, KDL::Vector>(frame, point));
		}
		else
		{
			ROS_ERROR("Could not parse point element for Convex");
			return false;
		}
	}

	return true;
}

boost::shared_ptr<Geometry> CollisionModel::parseGeometry(TiXmlElement *g)
{
	boost::shared_ptr<Geometry> geom;
	if (!g) return geom;
	TiXmlElement *shape_xml = g->FirstChildElement();
	if (!shape_xml)
	{
		ROS_ERROR("Geometry tag contains no child element.");
		return geom;
	}
	std::string type_name = shape_xml->ValueStr();
	if (type_name == "capsule")
	{
		Capsule *s = new Capsule();
		geom.reset(s);
		if (parseCapsule(*s, shape_xml))
		{
			s->shape.reset( static_cast<fcl_2::ShapeBase*>(new fcl_2::Capsule(s->radius, s->length)) );
			return geom;
		}
	}
	else if (type_name == "convex")
	{
		Convex *s = new Convex();
		geom.reset(s);
		if (parseConvex(*s, shape_xml))
		{
			return geom;
		}
	}
	else
	{
		ROS_ERROR("Unknown geometry type '%s'", type_name.c_str());
		return geom;
	}
	return boost::shared_ptr<Geometry>();
}

bool CollisionModel::parseCollision(Collision &col, TiXmlElement* config)
{
	col.clear();
	// Origin
	TiXmlElement *o = config->FirstChildElement("origin");
	if (o) {
		if (!parsePose(col.origin, o))
			return false;
	}
	// Geometry
	TiXmlElement *geom = config->FirstChildElement("geometry");
	col.geometry = parseGeometry(geom);
	if (!col.geometry)
		return false;
	const char *name_char = config->Attribute("name");
	return true;
}

bool CollisionModel::parseLink(Link &link, TiXmlElement* config)
{
	link.clear();
	const char *name_char = config->Attribute("name");
	if (!name_char)
	{
		ROS_ERROR("No name given for the link.");
		return false;
	}
	link.name = std::string(name_char);
	// Multiple Collisions (optional)
	for (TiXmlElement* col_xml = config->FirstChildElement("self_collision_checking"); col_xml; col_xml = col_xml->NextSiblingElement("self_collision_checking"))
	{
		boost::shared_ptr<Collision> col;
		col.reset(new Collision());
		col->parent_.reset(&link);
		if (parseCollision(*col, col_xml))
		{
			link.collision_array.push_back(col);
		}
		else
		{
			col.reset();
			ROS_ERROR("Could not parse collision element for Link [%s]", link.name.c_str());
			return false;
		}
	}
}

bool CollisionModel::parseDisableCollision(std::string &link1, std::string &link2, TiXmlElement *c)
{
	const char* link1_str = c->Attribute("link1");
	const char* link2_str = c->Attribute("link2");
	if (link1_str != NULL && link2_str != NULL)
	{
		link1 = link1_str;
		link2 = link2_str;
		return true;
	}
	ROS_ERROR("disable_collisions has wrong attributes");

	return false;
}

void CollisionModel::parseSRDF(const std::string &xml_string)
{
	disabled_collisions.clear();

	TiXmlDocument xml_doc;
	xml_doc.Parse(xml_string.c_str());
	if (xml_doc.Error())
	{
		ROS_ERROR("%s", xml_doc.ErrorDesc());
		xml_doc.ClearError();
		return;
	}

	TiXmlElement *robot_xml = xml_doc.FirstChildElement("robot");
	if (!robot_xml)
	{
		ROS_ERROR("Could not find the 'robot' element in the xml file");
		return;
	}
	// Get robot name
	const char *name = robot_xml->Attribute("name");
	if (!name)
	{
		ROS_ERROR("No name given for the robot.");
		return;
	}

	if (name_ != std::string(name))
	{
		ROS_ERROR("Name from SRDF: %s differ from %s", name, name_.c_str());
		return;
	}

	// Get all disable_collisions elements
	for (TiXmlElement* disable_collision_xml = robot_xml->FirstChildElement("disable_collisions"); disable_collision_xml; disable_collision_xml = disable_collision_xml->NextSiblingElement("disable_collisions"))
	{
		std::string link1, link2;
		try {
			parseDisableCollision(link1, link2, disable_collision_xml);
			int link1_id = getLinkIndex(link1);
			int link2_id = getLinkIndex(link2);
			if (link1_id == -1)
			{
				ROS_ERROR("link '%s' does not exist.", link1.c_str());
				return;
			}
			if (link2_id == -1)
			{
				ROS_ERROR("link '%s' does not exist.", link2.c_str());
				return;
			}
			disabled_collisions.push_back(std::make_pair<int, int>(link1_id, link2_id));
		}
		catch (urdf::ParseError &e) {
			ROS_ERROR("disable_collisions xml is not initialized correctly");
			return;
		}
	}


}

boost::shared_ptr<CollisionModel> CollisionModel::parseURDF(const std::string &xml_string)
{
	boost::shared_ptr<CollisionModel> model(new CollisionModel);
	model->root_index_ = -1;
	TiXmlDocument xml_doc;
	xml_doc.Parse(xml_string.c_str());
	if (xml_doc.Error())
	{
		ROS_ERROR("%s", xml_doc.ErrorDesc());
		xml_doc.ClearError();
		model.reset();
		return model;
	}
	TiXmlElement *robot_xml = xml_doc.FirstChildElement("robot");
	if (!robot_xml)
	{
		ROS_ERROR("Could not find the 'robot' element in the xml file");
		model.reset();
		return model;
	}
	// Get robot name
	const char *name = robot_xml->Attribute("name");
	if (!name)
	{
		ROS_ERROR("No name given for the robot.");
		model.reset();
		return model;
	}
	model->name_ = std::string(name);

	int link_count = 0;
	// count links
	for (TiXmlElement* link_xml = robot_xml->FirstChildElement("link"); link_xml; link_xml = link_xml->NextSiblingElement("link"))
	{
		link_count++;
	}

	if (link_count == 0){
		ROS_ERROR("No link elements found in urdf file");
		model.reset();
		return model;
	}

	if (model->links_ != NULL)
	{
		delete[] model->links_;
		model->links_ = NULL;
	}
	model->links_ = new boost::shared_ptr<Link>[link_count];
	model->link_count_ = link_count;

	int link_index=0;
	// Get all Link elements
	for (TiXmlElement* link_xml = robot_xml->FirstChildElement("link"); link_xml; link_xml = link_xml->NextSiblingElement("link"), link_index++)
	{
		model->links_[link_index].reset(new Link);
		model->links_[link_index]->index_ = link_index;
		try {
			parseLink(*(model->links_[link_index]), link_xml);
		}
		catch (urdf::ParseError &e) {
			ROS_ERROR("link xml is not initialized correctly");
			model.reset();
			return model;
		}
	}

	for (int l_i = 0; l_i < model->link_count_; l_i++)
	{
		for (Link::VecPtrCollision::iterator c_it = model->links_[l_i]->collision_array.begin(); c_it != model->links_[l_i]->collision_array.end(); c_it++)
		{
			if ((*c_it)->geometry->type == Geometry::CONVEX)
			{
				Convex *conv = static_cast<Convex*>( (*c_it)->geometry.get() );
				conv->points_id_.clear();
				for (Convex::ConvexPointsStrVector::iterator p_it = conv->points_str_.begin(); p_it != conv->points_str_.end(); p_it++)
				{
					int id = model->getLinkIndex(p_it->first);
					if (id == -1)
					{
						ROS_ERROR("parseURDF: could not find link %s", p_it->first.c_str());
						break;
					}
					conv->points_id_.push_back( std::make_pair<int, KDL::Vector>(id, p_it->second) );
				}
			}
		}
	}

	return model;
}

void CollisionModel::generateCollisionPairs()
{
	enabled_collisions.clear();
//	ROS_INFO("%ld", links_.size());

	for (int l_i = 0; l_i < link_count_; l_i++)
	{
		if (links_[l_i]->collision_array.size() == 0)
		{
			continue;
		}
		for (int l_j = 0; l_j < link_count_; l_j++)
		{
			if (links_[l_j]->collision_array.size() == 0)
			{
				continue;
			}
			if (l_i == l_j)
			{
				continue;
			}
			bool add = true;
			for (CollisionPairs::iterator dc_it = disabled_collisions.begin(); dc_it != disabled_collisions.end(); dc_it++)
			{
				if (	(dc_it->first == l_i && dc_it->second == l_j) ||
					(dc_it->second == l_i && dc_it->first == l_j) )
				{
					add = false;
					break;
				}
			}
			if (add)
			{
				for (CollisionPairs::iterator ec_it = enabled_collisions.begin(); ec_it != enabled_collisions.end(); ec_it++)
				{
					if (	(ec_it->first == l_i && ec_it->second == l_j) ||
						(ec_it->second == l_i && ec_it->first == l_j) )
					{
						add = false;
						break;
					}
				}

				if (add)
				{
					enabled_collisions.push_back(std::make_pair<int, int>(l_i, l_j));
				}
			}
		}
	}
}

double CollisionModel::getDistance(const Geometry &geom1, const KDL::Frame &tf1, const Geometry &geom2, const KDL::Frame &tf2, KDL::Vector &d1_out, KDL::Vector &d2_out, double d0)
{
	if (geom1.type == Geometry::CAPSULE && geom2.type == Geometry::CAPSULE)
	{
//		ROS_INFO("DistanceMeasure::getDistance: CAPSULE,CAPSULE");
		const fcl_2::Capsule* ob1 = static_cast<const fcl_2::Capsule*>(geom1.shape.get());
		const fcl_2::Capsule* ob2 = static_cast<const fcl_2::Capsule*>(geom2.shape.get());

		// calculate narrowphase distance
		if ((tf1.p - tf2.p).Norm() > (ob1->radius + ob1->lz)/2.0 + (ob2->radius + ob2->lz)/2.0 + d0)
		{
			return d0 * 2.0;
		}

		// capsules are shifted by length/2
		double x1,y1,z1,w1, x2, y2, z2, w2;
		KDL::Frame tf1_corrected = tf1 * KDL::Frame(KDL::Vector(0,0,-ob1->lz/2.0));
		tf1_corrected.M.GetQuaternion(x1,y1,z1,w1);
		KDL::Frame tf2_corrected = tf2 * KDL::Frame(KDL::Vector(0,0,-ob2->lz/2.0));
		tf2_corrected.M.GetQuaternion(x2,y2,z2,w2);

		// output variables
		fcl_2::FCL_REAL distance;
		fcl_2::Vec3f p1;
		fcl_2::Vec3f p2;
		gjk_solver.shapeDistance(
			*ob1, fcl_2::Transform3f(fcl_2::Quaternion3f(w1,x1,y1,z1), fcl_2::Vec3f(tf1_corrected.p.x(),tf1_corrected.p.y(),tf1_corrected.p.z())),
			*ob2, fcl_2::Transform3f(fcl_2::Quaternion3f(w2,x2,y2,z2), fcl_2::Vec3f(tf2_corrected.p.x(),tf2_corrected.p.y(),tf2_corrected.p.z())),
			 &distance, &p1, &p2);
		// the output for two capsules is in global coordinates
		d1_out = KDL::Vector(p1[0], p1[1], p1[2]);
		d2_out = KDL::Vector(p2[0], p2[1], p2[2]);
		return distance;
	}
	else if (geom1.type == Geometry::CAPSULE && geom2.type == Geometry::CONVEX)
	{
//		ROS_INFO("DistanceMeasure::getDistance: CAPSULE,CONVEX");
		const fcl_2::Capsule* ob1 = static_cast<const fcl_2::Capsule*>(geom1.shape.get());
		const fcl_2::Convex* ob2 = static_cast<const fcl_2::Convex*>(geom2.shape.get());

		const Convex *conv2 = static_cast<const Convex*>(&geom2);

		// calculate narrowphase distance
		if ((tf1.p - tf2 * conv2->center_).Norm() > (ob1->radius + ob1->lz)/2.0 + conv2->radius_ + d0)
		{
			return d0 * 2.0;
		}


		// capsules are shifted by length/2
		double x1,y1,z1,w1, x2, y2, z2, w2;
		KDL::Frame tf1_corrected = tf1;// * KDL::Frame(KDL::Vector(0,0,-ob1->lz/2.0));
		tf1_corrected.M.GetQuaternion(x1,y1,z1,w1);
		tf2.M.GetQuaternion(x2,y2,z2,w2);

		// output variables
		fcl_2::FCL_REAL distance;
		fcl_2::Vec3f p1;
		fcl_2::Vec3f p2;
		gjk_solver.shapeDistance(
			*ob1, fcl_2::Transform3f(fcl_2::Quaternion3f(w1,x1,y1,z1), fcl_2::Vec3f(tf1_corrected.p.x(),tf1_corrected.p.y(),tf1_corrected.p.z())),
			*ob2, fcl_2::Transform3f(fcl_2::Quaternion3f(w2,x2,y2,z2), fcl_2::Vec3f(tf2.p.x(),tf2.p.y(),tf2.p.z())),
			 &distance, &p1, &p2);
		// the output for two capsules is in wtf coordinates
		d1_out = tf1_corrected*KDL::Vector(p1[0], p1[1], p1[2]);
		d2_out = tf1_corrected*((tf1_corrected.Inverse()*tf2).Inverse()*KDL::Vector(p2[0], p2[1], p2[2]));
		return distance;
	}
	else if (geom1.type == Geometry::CONVEX && geom2.type == Geometry::CAPSULE)
	{
//		ROS_INFO("DistanceMeasure::getDistance: CONVEX, CAPSULE");
		const fcl_2::Convex* ob1 = static_cast<const fcl_2::Convex*>(geom1.shape.get());
		const fcl_2::Capsule* ob2 = static_cast<const fcl_2::Capsule*>(geom2.shape.get());

		const Convex *conv1 = static_cast<const Convex*>(&geom1);

		// calculate narrowphase distance
		if ((tf2.p - tf1 * conv1->center_).Norm() > (ob2->radius + ob2->lz)/2.0 + conv1->radius_ + d0)
		{
			return d0 * 2.0;
		}

		// capsules are shifted by length/2
		double x1,y1,z1,w1, x2, y2, z2, w2;
		tf1.M.GetQuaternion(x1,y1,z1,w1);
		KDL::Frame tf2_corrected = tf2;// * KDL::Frame(KDL::Vector(0,0,-ob2->lz/2.0));
		tf2_corrected.M.GetQuaternion(x2,y2,z2,w2);

		// output variables
		fcl_2::FCL_REAL distance;
		fcl_2::Vec3f p1;
		fcl_2::Vec3f p2;
		gjk_solver.shapeDistance(
			*ob1, fcl_2::Transform3f(fcl_2::Quaternion3f(w1,x1,y1,z1), fcl_2::Vec3f(tf1.p.x(),tf1.p.y(),tf1.p.z())),
			*ob2, fcl_2::Transform3f(fcl_2::Quaternion3f(w2,x2,y2,z2), fcl_2::Vec3f(tf2_corrected.p.x(),tf2_corrected.p.y(),tf2_corrected.p.z())),
			 &distance, &p1, &p2);
		// the output for two capsules is in wtf coordinates
		d1_out = tf1*KDL::Vector(p1[0], p1[1], p1[2]);
		d2_out = tf1*((tf1.Inverse()*tf2_corrected).Inverse()*KDL::Vector(p2[0], p2[1], p2[2]));
		return distance;
	}
	else if (geom1.type == Geometry::CONVEX && geom2.type == Geometry::CONVEX)
	{
//		ROS_INFO("DistanceMeasure::getDistance: CONVEX,CONVEX");
		const fcl_2::Convex* ob1 = static_cast<const fcl_2::Convex*>(geom1.shape.get());
		const fcl_2::Convex* ob2 = static_cast<const fcl_2::Convex*>(geom2.shape.get());

		const Convex *conv1 = static_cast<const Convex*>(&geom1);
		const Convex *conv2 = static_cast<const Convex*>(&geom2);

		// calculate narrowphase distance
		if ((tf1 * conv1->center_ - tf2 * conv2->center_).Norm() > conv1->radius_ + conv2->radius_ + d0)
		{
			return d0 * 2.0;
		}

		double x1,y1,z1,w1, x2, y2, z2, w2;
		tf1.M.GetQuaternion(x1,y1,z1,w1);
		tf2.M.GetQuaternion(x2,y2,z2,w2);

		// output variables
		fcl_2::FCL_REAL distance;
		fcl_2::Vec3f p1;
		fcl_2::Vec3f p2;
		gjk_solver.shapeDistance(
			*ob1, fcl_2::Transform3f(fcl_2::Quaternion3f(w1,x1,y1,z1), fcl_2::Vec3f(tf1.p.x(),tf1.p.y(),tf1.p.z())),
			*ob2, fcl_2::Transform3f(fcl_2::Quaternion3f(w2,x2,y2,z2), fcl_2::Vec3f(tf2.p.x(),tf2.p.y(),tf2.p.z())),
			 &distance, &p1, &p2);
		// the output for two capsules is in wtf coordinates
		d1_out = tf1*KDL::Vector(p1[0], p1[1], p1[2]);
		d2_out = tf1*((tf1.Inverse()*tf2).Inverse()*KDL::Vector(p2[0], p2[1], p2[2]));
		return distance;
	}
	else
	{
		ROS_ERROR("not supported distance measure");
	}
}

}	// namespace self_collision


