#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Point.h"
#include "urdf/model.h"

#include <iostream>

#include <kdl_parser/kdl_parser.hpp>
#include "self_collision_test/urdf_collision_parser.h"
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/treefksolverpos_recursive.hpp>

#include "narrowphase.h"

#include "marker_publisher.h"
#include "qhull_interface.h"

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
	void updateConvex(std::string name, const std::vector<KDL::Vector> &v, const std::vector<Face> &f);
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

void DistanceMeasure::updateConvex(std::string name, const std::vector<KDL::Vector> &v, const std::vector<Face> &f)
{
	ObjectsMap::iterator it = map_.find(name);
	if (it != map_.end() && it->second.first == CONVEX)
	{
		fcl_2::Convex* ob = static_cast<fcl_2::Convex*>(it->second.second);
		int poly_counter = 0;
		for (std::vector<Face>::const_iterator it = f.begin(); it != f.end(); it++)
		{
			ob->polygons[poly_counter++] = it->count;
			for (int i=0; i<it->count; i++)
			{
				ob->polygons[poly_counter++] = it->i[i];
			}
		}

		ob->num_planes = f.size();

		int points_counter = 0;
		for (std::vector<KDL::Vector>::const_iterator it = v.begin(); it != v.end(); it++)
		{
			ob->points[points_counter++] = fcl_2::Vec3f(it->x(), it->y(), it->z());
		}

		ob->num_points = v.size();

//		ROS_INFO("DistanceMeasure::updateConvex: %d  %d", ob->num_planes, ob->num_points);
		ob->fillEdges();
	}
	else
	{
		ROS_ERROR("DistanceMeasure::updateConvex: failure");
	}

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
			m_id = publishSinglePointMarker(pub, m_id, tf * KDL::Vector(), 1, 0, 0, 0.02);
		}
	}
	return m_id;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "qhull_test");
	ros::NodeHandle n;

	initQhull();

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

	// read robot semantic description
	std::string robot_semantic_description_;
	node.param("robot_semantic_description", robot_semantic_description_, std::string());

	// TODO
















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

	DistanceMeasure dm;

	ROS_INFO("parsing robot_description for self-collision data");
	boost::shared_ptr<self_collision::CollisionModel> collision_model = self_collision::CollisionModel::parseURDF(robot_description_);

	for (VecPtrLink::iterator l_it = links_.begin(); l_it != links_.end(); l_it++)
	{
		boost::shared_ptr< const self_collision::Link > link = collision_model->getLink((*l_it)->name);

		std::cout << (*l_it)->name.c_str() << std::endl;
//		ROS_INFO("link: %s  %s, collision geometries: %lu", (*l_it)->name.c_str(), link->name.c_str(), link->collision_array.size());

		int collision_geom_idx = 0;
		for (self_collision::Link::VecPtrCollision::const_iterator c_it = link->collision_array.begin(); c_it != link->collision_array.end(); c_it++)
		{
			char idx_str[20];
			sprintf(idx_str, "_%d", collision_geom_idx);
			//(*c_it)->origin
			switch ((*c_it)->geometry->type)
			{
				case self_collision::Geometry::CAPSULE:
					{
						ROS_INFO("      CAPSULE");
						self_collision::Capsule *capsule = static_cast<self_collision::Capsule*>((*c_it)->geometry.get());
						dm.addCapsule(link->name + idx_str, capsule->radius, capsule->length);
						break;
					}
				case self_collision::Geometry::CONVEX:
					{
						ROS_INFO("      CONVEX");
						self_collision::Convex *convex = static_cast<self_collision::Convex*>((*c_it)->geometry.get());
						dm.addConvex(link->name + idx_str);
						break;
					}
				default:
					ROS_ERROR("Unknown collision geometry type");
			}
			collision_geom_idx++;
		}
	}

	// subscribe to joint_states
	ros::Subscriber sub = n.subscribe("joint_states", 1000, joint_statesCallback);

	// topic name, queue size
	ros::Publisher vis_pub = n.advertise<visualization_msgs::MarkerArray>( "velma_markers", 1000 );


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

	KDL::TreeFkSolverPos_recursive fk_solver(robot_tree);

	KDL::JntArray q(joints);

	// rewrite joint positions from joint_states_map to q
	for (JointStatesMap::iterator js_it = joint_states_map.begin(); js_it != joint_states_map.end(); js_it++)
	{
		q(js_it->second.first) = js_it->second.second;
	}

	std::cout << "joints count: " << joints << std::endl;


	double angle = 0.0;
	while (ros::ok())
	{
		int m_id = 0;
		// rewrite joint positions from joint_states_map to q
		for (JointStatesMap::iterator js_it = joint_states_map.begin(); js_it != joint_states_map.end(); js_it++)
		{
			q(js_it->second.first) = js_it->second.second;
		}

		//
/*		KDL::Frame T_B_E;
		fk_solver.JntToCart(q, T_B_E, "left_HandPalmLink");
		KDL::Frame T_o1 = T_B_E;

		KDL::Frame T_o2(KDL::Rotation::RotY(angle*1.12678891379912), KDL::Vector(0.5,0,2));

		KDL::Vector d1, d2;
		double distance = dm.getDistance("conv1", T_o1, "conv2", T_o2, d1, d2);

		ROS_INFO("%lf    %lf  %lf  %lf    %lf  %lf  %lf", distance, d1[0], d1[1], d1[2], d2[0], d2[1], d2[2]);

		angle += 0.01;
*/		//


/*	KDL::Frame T_B_E;
	int result = fk_solver.JntToCart(q, T_B_E, "right_HandPalmLink");
	if (result != 0)
		ROS_ERROR("fk_solver failed");
	KDL::Frame fr2(KDL::Rotation::RotY(90.0/180.0*3.1415), KDL::Vector(0.075,0,0.2));
	KDL::Frame fr = T_B_E * fr2;

//	m_id = publishCapsule(vis_pub, m_id, fr, 0.25, 0.04);
	m_id = publishCylinder(vis_pub, m_id, fr, 0.15, 0.04);

		ros::spinOnce();

		ros::Duration(0.05).sleep();
		continue;
*/
		// iterate through all links
		for (VecPtrLink::iterator l_it = links_.begin(); l_it != links_.end(); l_it++)
		{
			boost::shared_ptr< const self_collision::Link > link = collision_model->getLink((*l_it)->name);
			KDL::Frame T_B_L;
			bool transformationValid = false;

			int collision_geom_idx = 0;
			// iterate through collision objects
			for (self_collision::Link::VecPtrCollision::const_iterator c_it = link->collision_array.begin(); c_it != link->collision_array.end(); c_it++)
			{
				char idx_str[20];
				sprintf(idx_str, "_%d", collision_geom_idx);
				switch ((*c_it)->geometry->type)
				{
					case self_collision::Geometry::CAPSULE:
						{
							if (!transformationValid)
							{
								int result = fk_solver.JntToCart(q, T_B_L, (*l_it)->name);
								if (result != 0)
									ROS_ERROR("fk_solver failed");
								transformationValid = true;
							}
							self_collision::Capsule* capsule = static_cast<self_collision::Capsule*>((*c_it)->geometry.get());
						
							m_id = dm.publishMarker(vis_pub, m_id, link->name + idx_str, T_B_L * (*c_it)->origin);
							break;
						}
					case self_collision::Geometry::CONVEX:
						{
							if (!transformationValid)
							{
								int result = fk_solver.JntToCart(q, T_B_L, (*l_it)->name);
								if (result != 0)
									ROS_ERROR("fk_solver failed");
								transformationValid = true;
							}
							self_collision::Convex* convex = static_cast<self_collision::Convex*>((*c_it)->geometry.get());
							std::vector<KDL::Vector> points;

							for (self_collision::Convex::ConvexPointsVector::iterator it = convex->points.begin(); it != convex->points.end(); it++)
							{
								KDL::Frame T_B_F;
								fk_solver.JntToCart(q, T_B_F, it->first);
								KDL::Frame T_E_F = T_B_L.Inverse() * T_B_F;
								points.push_back(T_E_F * it->second);
							}
							std::vector<KDL::Vector> v_out;
							std::vector<Face> f_out;

							calculateQhull(points, v_out, f_out);
							initQhull();
							dm.updateConvex(link->name + idx_str, v_out, f_out);
							m_id = dm.publishMarker(vis_pub, m_id, link->name + idx_str, T_B_L);
							break;
						}
					default:
						ROS_ERROR("Unknown collision geometry type");
				}
				collision_geom_idx++;
			}
		}

		ros::spinOnce();

		ros::Duration(0.05).sleep();
	}


  return 0;
}


