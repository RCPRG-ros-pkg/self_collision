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
#include "distance_measure.h"


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

	// read robot semantic description
	ROS_INFO("parsing robot_semantic_description for self-collision data");
	std::string robot_semantic_description_;
	node.param("robot_semantic_description", robot_semantic_description_, std::string());

	collision_model->parseSRDF(robot_semantic_description_);
	collision_model->generateCollisionPairs();

//	ROS_INFO("%ld", collision_model->disabled_collisions.size());
//	ROS_INFO("%ld", collision_model->enabled_collisions.size());
//	return 0;

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


