#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Point.h"
#include "urdf/model.h"

#include <iostream>

#ifdef __cplusplus
extern "C" {
#include <libqhull/qset.h>
#include <libqhull/libqhull.h>
}
#else
#include <libqhull/qset.h>
#include <libqhull/libqhull.h>
#endif


//#include <qhull/Qhull.h>
//#include <qhull/QhullFacetList.h>
#include <kdl_parser/kdl_parser.hpp>
#include "self_collision_test/urdf_collision_parser.h"
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/treefksolverpos_recursive.hpp>

#include "narrowphase.h"
//#include "fcl/narrowphase/narrowphase.h"
//#include "fcl/distance.h"
//#include "fcl/shape/geometric_shapes.h"

int publishSinglePointMarker(ros::Publisher &pub, int m_id, const KDL::Vector &pos, double r, double g, double b)
{
	visualization_msgs::MarkerArray marker_array;
	visualization_msgs::Marker marker;
	marker.header.frame_id = "world";
	marker.header.stamp = ros::Time();
	marker.ns = "default";
	marker.id = m_id;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = pos.x();
	marker.pose.position.y = pos.y();
	marker.pose.position.z = pos.z();
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.05;
	marker.scale.y = 0.05;
	marker.scale.z = 0.05;
	marker.color.a = 1.0;
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker_array.markers.push_back(marker);
	pub.publish( marker_array );
	return m_id + 1;
}

int publishLineMarker(ros::Publisher &pub, int m_id, const KDL::Vector &pos1, const KDL::Vector &pos2, double r, double g, double b)
{
	visualization_msgs::MarkerArray marker_array;
	visualization_msgs::Marker marker;
	marker.header.frame_id = "world";
	marker.header.stamp = ros::Time();
	marker.ns = "default";
	marker.id = m_id;
	marker.type = visualization_msgs::Marker::LINE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.points.resize(2);
	marker.points[0].x = pos1.x();
	marker.points[0].y = pos1.y();
	marker.points[0].z = pos1.z();
	marker.points[1].x = pos2.x();
	marker.points[1].y = pos2.y();
	marker.points[1].z = pos2.z();
	marker.scale.x = 0.01;
	marker.color.a = 1.0;
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker_array.markers.push_back(marker);
	pub.publish( marker_array );
	return m_id + 1;
}

int publishCapsule(ros::Publisher &pub, int m_id, KDL::Frame fr, double length, double radius)
{
	visualization_msgs::MarkerArray marker_array;

	KDL::Vector zero;
	KDL::Vector v(0,0,length);
	KDL::Vector v2 = (fr * v) - (fr * zero);

	visualization_msgs::Marker marker;
	marker.header.frame_id = "world";//"torso_base";
	marker.header.stamp = ros::Time();
	marker.ns = "default";
	marker.id = m_id;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = fr.p.x() - v2.x()/2.0;
	marker.pose.position.y = fr.p.y() - v2.y()/2.0;
	marker.pose.position.z = fr.p.z() - v2.z()/2.0;
	double qx, qy, qz, qw;
	fr.M.GetQuaternion(qx, qy, qz, qw);
	marker.pose.orientation.x = qx;
	marker.pose.orientation.y = qy;
	marker.pose.orientation.z = qz;
	marker.pose.orientation.w = qw;
	marker.scale.x = radius * 2.0;
	marker.scale.y = radius * 2.0;
	marker.scale.z = radius * 2.0;
	marker.color.a = 0.5;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	marker_array.markers.push_back(marker);

	visualization_msgs::Marker marker2(marker);
	marker2.id = m_id+1;
	marker2.pose.position.x = fr.p.x() + v2.x()/2.0;
	marker2.pose.position.y = fr.p.y() + v2.y()/2.0;
	marker2.pose.position.z = fr.p.z() + v2.z()/2.0;
	marker_array.markers.push_back(marker2);

	visualization_msgs::Marker marker3(marker);
	marker3.id = m_id+2;
	marker3.type = visualization_msgs::Marker::CYLINDER;
	marker3.pose.position.x = fr.p.x();
	marker3.pose.position.y = fr.p.y();
	marker3.pose.position.z = fr.p.z();
	marker3.scale.z = length;
	marker_array.markers.push_back(marker3);

	visualization_msgs::Marker marker4(marker);
	marker4.id = m_id+3;
	marker4.type = visualization_msgs::Marker::SPHERE;
	marker4.pose.position.x = fr.p.x();
	marker4.pose.position.y = fr.p.y();
	marker4.pose.position.z = fr.p.z();
	marker4.scale.x = 0.01;
	marker4.scale.y = 0.01;
	marker4.scale.z = 0.01;
	marker4.color.a = 1.0;
	marker4.color.r = 0.0;
	marker4.color.g = 1.0;
	marker4.color.b = 0.0;
	marker_array.markers.push_back(marker4);

	pub.publish( marker_array );
	return m_id + 4;
}

int publishMeshMarker(ros::Publisher &pub, int m_id, const KDL::Frame &tf, const fcl_2::Vec3f *points, int num_planes, const int *polygons, double r, double g, double b)
{
	visualization_msgs::MarkerArray marker_array;
	// triangles
	visualization_msgs::Marker marker;
	marker.header.frame_id = "world";
	marker.header.stamp = ros::Time();
	marker.ns = "default";
	marker.id = m_id;
	marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = tf.p.x();
	marker.pose.position.y = tf.p.y();
	marker.pose.position.z = tf.p.z();
	double qx, qy, qz, qw;
	tf.M.GetQuaternion(qx, qy, qz, qw);
	marker.pose.orientation.x = qx;
	marker.pose.orientation.y = qy;
	marker.pose.orientation.z = qz;
	marker.pose.orientation.w = qw;
	marker.points.resize(num_planes*3);
	for (int i=0; i<num_planes; i++)
	{
		for (int j=0; j<3; j++)
		{
//			ROS_INFO("num_planes: %d   i: %d   j: %d   polygons[i*4 + 1 + j]: %d", num_planes, i, j, polygons[i*4 + 1 + j]);
			marker.points[i*3+j].x = points[polygons[i*4 + 1 + j]][0];
			marker.points[i*3+j].y = points[polygons[i*4 + 1 + j]][1];
			marker.points[i*3+j].z = points[polygons[i*4 + 1 + j]][2];
		}
	}
	marker.scale.x = 1.0;
	marker.scale.y = 1.0;
	marker.scale.z = 1.0;
	marker.color.a = 0.5;
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker_array.markers.push_back(marker);

	// edges
	visualization_msgs::Marker marker2;
	marker2.header.frame_id = "world";
	marker2.header.stamp = ros::Time();
	marker2.ns = "default";
	marker2.id = m_id+1;
	marker2.type = visualization_msgs::Marker::LINE_LIST;
	marker2.action = visualization_msgs::Marker::ADD;
	marker2.pose.position.x = tf.p.x();
	marker2.pose.position.y = tf.p.y();
	marker2.pose.position.z = tf.p.z();
	tf.M.GetQuaternion(qx, qy, qz, qw);
	marker2.pose.orientation.x = qx;
	marker2.pose.orientation.y = qy;
	marker2.pose.orientation.z = qz;
	marker2.pose.orientation.w = qw;
	marker2.points.resize(num_planes*3*2);
	for (int i=0; i<num_planes; i++)
	{
		for (int j=0; j<3; j++)
		{
			marker2.points[i*6+j*2].x = points[polygons[i*4 + 1 + j]][0];
			marker2.points[i*6+j*2].y = points[polygons[i*4 + 1 + j]][1];
			marker2.points[i*6+j*2].z = points[polygons[i*4 + 1 + j]][2];

			marker2.points[i*6+j*2+1].x = points[polygons[i*4 + 1 + (j+1)%3]][0];
			marker2.points[i*6+j*2+1].y = points[polygons[i*4 + 1 + (j+1)%3]][1];
			marker2.points[i*6+j*2+1].z = points[polygons[i*4 + 1 + (j+1)%3]][2];
		}
	}
	marker2.scale.x = 0.005;
	marker2.color.a = 1;
	marker2.color.r = 1-r;
	marker2.color.g = 1-g;
	marker2.color.b = 1-b;

	marker_array.markers.push_back(marker2);
	pub.publish( marker_array );
	return m_id + 2;
}

typedef std::map<std::string, std::pair<int,double> > JointStatesMap;
JointStatesMap joint_states_map;

void joint_statesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	size_t count = msg->name.size();
	for (size_t i=0; i<count; i++)
	{
		JointStatesMap::iterator q_it = joint_states_map.find(msg->name[i]);
		if (q_it != joint_states_map.end())
		{
			q_it->second.second = msg->position[i];
		}
		
	}
}

class DistanceMeasure
{
private:
	enum ObjectType {CAPSULE, CONVEX};
	typedef std::map<std::string, std::pair<ObjectType, fcl_2::ShapeBase*> > ObjectsMap;
public:
	DistanceMeasure();
	void addCapsule(const std::string &name, double radius, double length);
	void addConvex(std::string name);
	double getDistance(const std::string &name1, const KDL::Frame &tf1, const std::string &name2, const KDL::Frame &tf2, KDL::Vector &d1_out, KDL::Vector &d2_out);
	int publishMarker(ros::Publisher &pub, int m_id, const std::string &name, const KDL::Frame tf);
private:
	fcl_2::GJKSolver_indep gjk_solver;
	ObjectsMap map_;
};

DistanceMeasure::DistanceMeasure()
{
}

void DistanceMeasure::addCapsule(const std::string &name, double radius, double length)
{
	map_[name] = std::make_pair<ObjectType, fcl_2::ShapeBase*>(CAPSULE, static_cast<fcl_2::ShapeBase*>(new fcl_2::Capsule(radius, length)));
}

void DistanceMeasure::addConvex(std::string name)
{
/*
 * The first two arguments (plane_normals_ and plane_dis_) are not used and can be set to NULL.
 * The polygons_ vector has the following structure:
 * polygons_[0]: n = number of points in polygon 0
 * polygons_[1]: point 0 in polygon 0
 * polygons_[n]: point n-1 in polygon 0
 * polygons_[n+1]: n = number of points in polygon 1

Convex(Vec3f* plane_normals_,
         FCL_REAL* plane_dis_,
         int num_planes_,
         Vec3f* points_,
         int num_points_,
         int* polygons_)
*/
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
	map_[name] = std::make_pair<ObjectType, fcl_2::ShapeBase*>(CONVEX, static_cast<fcl_2::ShapeBase*>(new fcl_2::Convex(NULL, NULL, num_planes, points, num_points, polygons)));
}

double DistanceMeasure::getDistance(const std::string &name1, const KDL::Frame &tf1, const std::string &name2, const KDL::Frame &tf2, KDL::Vector &d1_out, KDL::Vector &d2_out)
{
	ObjectsMap::iterator it1 = map_.find(name1);
	ObjectsMap::iterator it2 = map_.find(name2);
	if (it1 != map_.end() && it2 != map_.end() && it1 != it2)
	{
		if (it1->second.first == CAPSULE && it2->second.first == CAPSULE)
		{
			ROS_INFO("DistanceMeasure::getDistance: CAPSULE,CAPSULE");
			const fcl_2::Capsule* ob1 = static_cast<const fcl_2::Capsule*>(it1->second.second);
			const fcl_2::Capsule* ob2 = static_cast<const fcl_2::Capsule*>(it2->second.second);

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
		else if ((it1->second.first == CAPSULE && it2->second.first == CONVEX) || (it1->second.first == CONVEX && it2->second.first == CAPSULE))
		{
			if (it1->second.first == CONVEX && it2->second.first == CAPSULE)
			{
				// swap iterators
				ObjectsMap::iterator tmp = it1;
				it1 = it2;
				it2 = tmp;
			}
			// now it1 points to CAPSULE and it2 points to CONVEX
			ROS_INFO("DistanceMeasure::getDistance: CAPSULE,CONVEX");
			const fcl_2::Capsule* ob1 = static_cast<const fcl_2::Capsule*>(it1->second.second);
			const fcl_2::Convex* ob2 = static_cast<const fcl_2::Convex*>(it2->second.second);

			// capsules are shifted by length/2
			double x1,y1,z1,w1, x2, y2, z2, w2;
			KDL::Frame tf1_corrected = tf1 * KDL::Frame(KDL::Vector(0,0,-ob1->lz/2.0));
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
		else if (it1->second.first == CONVEX && it2->second.first == CONVEX)
		{
			ROS_INFO("DistanceMeasure::getDistance: CONVEX,CONVEX");
			const fcl_2::Convex* ob1 = static_cast<const fcl_2::Convex*>(it1->second.second);
			const fcl_2::Convex* ob2 = static_cast<const fcl_2::Convex*>(it2->second.second);

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
}

int DistanceMeasure::publishMarker(ros::Publisher &pub, int m_id, const std::string &name, const KDL::Frame tf)
{
	ObjectsMap::iterator it = map_.find(name);
	if (it != map_.end())
	{
		if (it->second.first == CAPSULE)
		{
			const fcl_2::Capsule* ob = static_cast<const fcl_2::Capsule*>(it->second.second);
			m_id = publishCapsule(pub, m_id, tf, ob->lz, ob->radius);
		}
		else if (it->second.first == CONVEX)
		{
			const fcl_2::Convex* ob = static_cast<const fcl_2::Convex*>(it->second.second);
			m_id = publishMeshMarker(pub, m_id, tf, ob->points, ob->num_planes, ob->polygons, 0, 0, 1);
		}
		else
		{
			m_id = publishSinglePointMarker(pub, m_id, tf * KDL::Vector(), 1, 0, 0);
		}
	}
	return m_id;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "qhull_test");
	ros::NodeHandle n;

	FILE *null_sink = fopen("/dev/null", "w");
	char qhull_command_const[] = "Qt i";
	char qhull_command[256];
	memcpy(qhull_command, qhull_command_const, strlen(qhull_command_const) + 1);

	// void qh_init_A(FILE *infile, FILE *outfile, FILE *errfile, int argc, char *argv[])
	qh_init_A(NULL, null_sink, null_sink, 0, NULL);
	qh_initflags( qhull_command );
	// void qh_init_B(coordT *points, int numpoints, int dim, boolT ismalloc)
	coordT *points = new coordT[4*3];
	points[0] = 0;
	points[1] = 0;
	points[2] = 0;
	points[3] = 1;
	points[4] = 0;
	points[5] = 0;
	points[6] = 0;
	points[7] = 1;
	points[8] = 0;
	points[9] = 0;
	points[10] = 0;
	points[11] = 1;

	qh_init_B(points, 4, 3, false);

	qh_qhull();
	int faces_count = qh num_facets;
	int vertices_count = qh num_vertices;
	ROS_INFO("qhull test: vertices: %d  faces: %d", vertices_count, faces_count);

	for (facetT *f = qh facet_list; f != qh facet_tail; f = f->next)
	{
		for (int v_idx=0; v_idx < f->vertices->maxsize; v_idx++)
		{
			if (f->vertices->e[v_idx].p == NULL)
				break;
			vertexT *v = static_cast<vertexT *>(f->vertices->e[v_idx].p);
			std::cout << v->id << " ";
		}
		std::cout << std::endl;
	}

//	return 0;
//	qh facet_list (.next)
//	qh facet_tail
//	qh num_facets
//	qh vertex_list (.next)
//	vertex_tail
//	qh num_vertices
//	qh_init_A(NULL, null_sink, null_sink, argc, argv);
////	qh_initqhull_start(infile, outfile, errfile);
//	qh_initstatistics();
//	qh_init_qhull_command(argc, argv);

	// in ops: <component>.rosparam.getAll();
	// in orocos: this->addProperty("robot_description", robot_description_);

	// read KDL::Tree form the robot_description
	KDL::Tree robot_tree;
	ros::NodeHandle node;
	std::string robot_description_;
	node.param("robot_description", robot_description_, std::string());
	if (!kdl_parser::treeFromString(robot_description_, robot_tree))
	{
		ROS_ERROR("Failed to construct kdl tree");
		return -1;
	}

	// read the robot model from the robot_description
	ROS_INFO("parsing robot_description robot kinematic model data");
	urdf::Model robot_model_;
	robot_model_.initString(robot_description_);

	// save robot links
	typedef std::vector< boost::shared_ptr< urdf::Link > > VecPtrLink;
	std::vector< boost::shared_ptr< urdf::Link > > links_;
	robot_model_.getLinks(links_);

	// save robot joints
/*	for (std::map< std::string, boost::shared_ptr< urdf::Joint > >::const_iterator j_it = robot_model_.joints_.begin(); j_it != robot_model_.joints_.end(); j_it++)
	{
		std::cout << j_it->first << std::endl;
		joint_states_map[j_it->first] = 0.0;
	}
*/

	ROS_INFO("parsing robot_description for self-collision data");
	boost::shared_ptr<self_collision::CollisionModel> collision_model = self_collision::CollisionModel::parseURDF(robot_description_);

	for (VecPtrLink::iterator l_it = links_.begin(); l_it != links_.end(); l_it++)
	{
		boost::shared_ptr< const self_collision::Link > link = collision_model->getLink((*l_it)->name);

		std::cout << (*l_it)->name.c_str() << std::endl;
//		ROS_INFO("link: %s  %s, collision geometries: %lu", (*l_it)->name.c_str(), link->name.c_str(), link->collision_array.size());

		for (self_collision::Link::VecPtrCollision::const_iterator c_it = link->collision_array.begin(); c_it != link->collision_array.end(); c_it++)
		{
			//(*c_it)->origin
			switch ((*c_it)->geometry->type)
			{
				case self_collision::Geometry::CAPSULE:
					ROS_INFO("      CAPSULE");
					break;
				default:
					ROS_ERROR("Unknown collision geometry type");
			}
		}
	}

//	Chain torso_chain("torso_base", "torso_link2", robot_tree);
//	Chain left_chain("torso_link2", "left_arm_7_link", robot_tree);
//	Chain right_chain("torso_link2", "right_arm_7_link", robot_tree);

	// subscribe to joint_states
	ros::Subscriber sub = n.subscribe("joint_states", 1000, joint_statesCallback);

	// topic name, queue size
	ros::Publisher vis_pub = n.advertise<visualization_msgs::MarkerArray>( "velma_markers", 1000 );
	ros::Duration(0.5).sleep();

	int joints = 0;
	for (KDL::SegmentMap::const_iterator seg_it = robot_tree.getSegments().begin(); seg_it != robot_tree.getSegments().end(); seg_it++)
	{
		KDL::Joint::JointType type = seg_it->second.segment.getJoint().getType();
		if (	type == KDL::Joint::RotAxis ||
			type == KDL::Joint::RotX ||
			type == KDL::Joint::RotY ||
			type == KDL::Joint::RotZ)
		{
			std::cout << seg_it->second.segment.getJoint().getName() << std::endl;
			joint_states_map[seg_it->second.segment.getJoint().getName()] = std::make_pair<int, double>(seg_it->second.q_nr, 0.0);
			joints++;
		}		
	}

	std::cout << "joints count: " << joints << std::endl;

	KDL::TreeFkSolverPos_recursive fk_solver(robot_tree);
	KDL::JntArray q(joints);


	DistanceMeasure dm;
	dm.addCapsule("caps1", 0.1, 0.2);
	dm.addCapsule("caps2", 0.1, 0.3);

	dm.addConvex("conv1");
	dm.addConvex("conv2");

//	boost::shared_ptr<fcl_2::Capsule> capsule1(new fcl_2::Capsule(0.1, 0.2));
//	boost::shared_ptr<fcl_2::Capsule> capsule2(new fcl_2::Capsule(0.1, 0.3));
//	fcl_2::CollisionObject* o1 = new fcl_2::CollisionObject(capsule1);
//	fcl_2::CollisionObject* o2 = new fcl_2::CollisionObject(capsule2);

	double angle = 0.0;
	while (ros::ok())
	{
		int m_id = 0;

		//
		KDL::Frame T_o1(KDL::Rotation::RotZ(angle), KDL::Vector(-0.5,0,2));
//		double x,y,z,w;
//		T_o1.M.GetQuaternion(x,y,z,w);
//		o1->setTransform(fcl_2::Quaternion3f(w,x,y,z), fcl_2::Vec3f(T_o1.p.x(),T_o1.p.y(),T_o1.p.z()));

		KDL::Frame T_o2(KDL::Rotation::RotY(angle*1.12678891379912), KDL::Vector(0.5,0,2));
//		T_o2.M.GetQuaternion(x,y,z,w);
//		o2->setTransform(fcl_2::Quaternion3f(w,x,y,z), fcl_2::Vec3f(T_o2.p.x(),T_o2.p.y(),T_o2.p.z()));

		KDL::Vector d1, d2;
		double distance = dm.getDistance("conv1", T_o1, "conv2", T_o2, d1, d2);

//		fcl_2::FCL_REAL distance;
//		fcl_2::Vec3f p1;
//		fcl_2::Vec3f p2;
/*		fcl::DistanceRequest request(true, 0.0, 0.0, fcl::GST_LIBCCD);
		fcl::DistanceResult result;
//		distance = fcl::distance(o1, o2, request, result);

		fcl::distance(	(o1->collisionGeometry().get()), o1->getTransform(),
				(o2->collisionGeometry().get()), o2->getTransform(),
				request, result);
		distance = result.min_distance;
		p1 = result.nearest_points[0];
		p2 = result.nearest_points[1];
*/
//		gjk_solver.shapeDistance(*static_cast<const fcl_2::Convex*>(o1->collisionGeometry().get()), o1->getTransform(), *static_cast<const fcl_2::Convex*>(o2->collisionGeometry().get()), o2->getTransform(), &distance, &p1, &p2);

		ROS_INFO("%lf    %lf  %lf  %lf    %lf  %lf  %lf", distance, d1[0], d1[1], d1[2], d2[0], d2[1], d2[2]);

//		m_id = publishCapsule(vis_pub, m_id, T_o1, 0.2, 0.1);
//		m_id = publishCapsule(vis_pub, m_id, T_o2, 0.3, 0.1);
//		KDL::Vector d_1 = KDL::Vector(p1[0], p1[1], p1[2]);
//		KDL::Vector d_2 = KDL::Vector(p2[0], p2[1], p2[2]);
//		KDL::Vector d_1 = T_o1*KDL::Vector(p1[0], p1[1], p1[2]);
//		KDL::Vector d_2 = T_o1*((T_o1.Inverse()*T_o2).Inverse()*KDL::Vector(p2[0], p2[1], p2[2]));
//		m_id = publishSinglePointMarker(vis_pub, m_id, d_1, 1,0,0);
//		m_id = publishSinglePointMarker(vis_pub, m_id, d_2, 0,0,1);

		m_id = dm.publishMarker(vis_pub, m_id, "conv1", T_o1);
		m_id = dm.publishMarker(vis_pub, m_id, "conv2", T_o2);
		m_id = publishLineMarker(vis_pub, m_id, d1, d2, 0,0,1);
		angle += 0.01;
		//


		// rewrite joint positions from joint_states_map to q
		for (JointStatesMap::iterator js_it = joint_states_map.begin(); js_it != joint_states_map.end(); js_it++)
		{
			q(js_it->second.first) = js_it->second.second;
		}

		// iterate through all links
		for (VecPtrLink::iterator l_it = links_.begin(); l_it != links_.end(); l_it++)
		{
			boost::shared_ptr< const self_collision::Link > link = collision_model->getLink((*l_it)->name);
			KDL::Frame T_B_L;
			bool transformationValid = false;

			self_collision::Capsule* capsule = NULL;
			// iterate through collision objects
			for (self_collision::Link::VecPtrCollision::const_iterator c_it = link->collision_array.begin(); c_it != link->collision_array.end(); c_it++)
			{
				switch ((*c_it)->geometry->type)
				{
					case self_collision::Geometry::CAPSULE:
						if (!transformationValid)
						{
							int result = fk_solver.JntToCart(q, T_B_L, (*l_it)->name);
							if (result != 0)
								ROS_ERROR("fk_solver failed");
							transformationValid = true;
						}
						capsule = static_cast<self_collision::Capsule*>((*c_it)->geometry.get());
						
						m_id = publishCapsule(vis_pub, m_id, T_B_L * (*c_it)->origin, capsule->length, capsule->radius);
						break;
					default:
						ROS_ERROR("Unknown collision geometry type");
				}
			}
		}


		ros::spinOnce();

		ros::Duration(0.05).sleep();
	}


  return 0;
}
