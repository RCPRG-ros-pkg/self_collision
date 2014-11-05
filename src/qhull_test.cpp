#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Point.h"
#include "urdf/model.h"

#include <iostream>

#include <kdl_parser/kdl_parser.hpp>
#include "urdf_collision_parser.h"
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

	ROS_INFO("parsing robot_description for self-collision data");
	boost::shared_ptr<self_collision::CollisionModel> collision_model = self_collision::CollisionModel::parseURDF(robot_description_);
/*
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
			switch ((*c_it)->geometry->type)
			{
				case self_collision::Geometry::CAPSULE:
					{
						ROS_INFO("      CAPSULE");
						break;
					}
				case self_collision::Geometry::CONVEX:
					{
						ROS_INFO("      CONVEX");
						break;
					}
				default:
					ROS_ERROR("Unknown collision geometry type");
			}
			collision_geom_idx++;
		}
	}
*/
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

	// create map of transformations
	std::map<std::string, KDL::Frame> transformation_map;
	for (VecPtrLink::iterator l_it = links_.begin(); l_it != links_.end(); l_it++)
	{
		transformation_map[(*l_it)->name] = KDL::Frame();
	}

	clearMarkers(vis_pub, 0, 1000);
	int m_id = 0;
	int last_m_id = 0;
	double angle = 0.0;
	while (ros::ok())
	{
		if (m_id < last_m_id)
		{
			clearMarkers(vis_pub, m_id, last_m_id+1);
		}
		last_m_id = m_id;
		m_id = 0;
		// rewrite joint positions from joint_states_map to q
		for (JointStatesMap::iterator js_it = joint_states_map.begin(); js_it != joint_states_map.end(); js_it++)
		{
			q(js_it->second.first) = js_it->second.second;
		}

		// get current transformations
		for (VecPtrLink::iterator l_it = links_.begin(); l_it != links_.end(); l_it++)
		{
			KDL::Frame &T_B_L = transformation_map[(*l_it)->name];
			int result = fk_solver.JntToCart(q, T_B_L, (*l_it)->name);
			if (result != 0)
			{
				ROS_ERROR("fk error for link %s   error code: %d", (*l_it)->name.c_str(), result);
			}
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

			// iterate through collision objects
			for (self_collision::Link::VecPtrCollision::const_iterator c_it = link->collision_array.begin(); c_it != link->collision_array.end(); c_it++)
			{
				char idx_str[20];
				switch ((*c_it)->geometry->type)
				{
					case self_collision::Geometry::CAPSULE:
						{
							self_collision::Capsule* capsule = static_cast<self_collision::Capsule*>((*c_it)->geometry.get());
							KDL::Frame &T_B_L = transformation_map[(*l_it)->name];
							m_id = (*c_it)->geometry->publishMarker(vis_pub, m_id, T_B_L * (*c_it)->origin);
							break;
						}
					case self_collision::Geometry::CONVEX:
						{
							self_collision::Convex* convex = static_cast<self_collision::Convex*>((*c_it)->geometry.get());
							KDL::Frame &T_B_L = transformation_map[(*l_it)->name];

							std::vector<KDL::Vector> points;

							for (self_collision::Convex::ConvexPointsVector::iterator it = convex->points.begin(); it != convex->points.end(); it++)
							{
								KDL::Frame &T_B_F = transformation_map[it->first];
								KDL::Frame T_E_F = (T_B_L * (*c_it)->origin).Inverse() * T_B_F;
								points.push_back(T_E_F * it->second);
							}
							std::vector<KDL::Vector> v_out;
							std::vector<Face> f_out;

							calculateQhull(points, v_out, f_out);
							initQhull();
							convex->updateConvex(v_out, f_out);
							m_id = (*c_it)->geometry->publishMarker(vis_pub, m_id, T_B_L * (*c_it)->origin);
							break;
						}
					default:
						ROS_ERROR("Unknown collision geometry type");
				}
			}
		}

		if (false)
		{
			self_collision::Capsule caps;
			caps.radius = 0.1;
			caps.length = 0.1;
			caps.shape.reset( static_cast<fcl_2::ShapeBase*>(new fcl_2::Capsule(caps.radius, caps.length)) );

			self_collision::Capsule caps2;
			caps2.radius = 0.1;
			caps2.length = 0.1;
			caps2.shape.reset( static_cast<fcl_2::ShapeBase*>(new fcl_2::Capsule(caps2.radius, caps2.length)) );

			self_collision::Convex conv;
			self_collision::Convex conv2;

			KDL::Vector d1, d2;
			KDL::Frame T_l1(KDL::Rotation::RotY(30.0/180.0*3.1415), KDL::Vector(0.175,0,2.2)), T_l2(KDL::Vector(-0.2,0,2.2));

			double distance;
			distance = self_collision::CollisionModel::getDistance(caps, KDL::Frame(KDL::Vector(0,1.0,0)) * T_l1, conv, KDL::Frame(KDL::Vector(0,1.0,0)) * T_l2, d1, d2);
			m_id = caps.publishMarker(vis_pub, m_id, KDL::Frame(KDL::Vector(0,1.0,0)) * T_l1);
			m_id = conv.publishMarker(vis_pub, m_id, KDL::Frame(KDL::Vector(0,1.0,0)) * T_l2);
			m_id = publishLineMarker(vis_pub, m_id, d1, d2, 1, 0, 0);

			distance = self_collision::CollisionModel::getDistance(conv, KDL::Frame(KDL::Vector(0,0.7,0)) * T_l1, caps, KDL::Frame(KDL::Vector(0,0.7,0)) * T_l2, d1, d2);
			m_id = conv.publishMarker(vis_pub, m_id, KDL::Frame(KDL::Vector(0,0.7,0)) * T_l1);
			m_id = caps.publishMarker(vis_pub, m_id, KDL::Frame(KDL::Vector(0,0.7,0)) * T_l2);
			m_id = publishLineMarker(vis_pub, m_id, d1, d2, 1, 0, 0);

			distance = self_collision::CollisionModel::getDistance(caps2, KDL::Frame(KDL::Vector(0,0.4,0)) * T_l1, caps, KDL::Frame(KDL::Vector(0,0.4,0)) * T_l2, d1, d2);
			m_id = caps2.publishMarker(vis_pub, m_id, KDL::Frame(KDL::Vector(0,0.4,0)) * T_l1);
			m_id = caps.publishMarker(vis_pub, m_id, KDL::Frame(KDL::Vector(0,0.4,0)) * T_l2);
			m_id = publishLineMarker(vis_pub, m_id, d1, d2, 1, 0, 0);

			distance = self_collision::CollisionModel::getDistance(conv2, T_l1, conv, T_l2, d1, d2);
			m_id = conv2.publishMarker(vis_pub, m_id, T_l1);
			m_id = conv.publishMarker(vis_pub, m_id, T_l2);
			m_id = publishLineMarker(vis_pub, m_id, d1, d2, 1, 0, 0);
		}

		int low_distance_count = 0;
		for (self_collision::CollisionModel::CollisionPairs::iterator it = collision_model->enabled_collisions.begin(); it != collision_model->enabled_collisions.end(); it++)
		{
			KDL::Vector d1, d2;
			KDL::Frame T_l1, T_l2;
			KDL::Frame &T_B_L1 = transformation_map[it->first];
			KDL::Frame &T_B_L2 = transformation_map[it->second];

			boost::shared_ptr< const self_collision::Link > link1 = collision_model->getLink(it->first);
			boost::shared_ptr< const self_collision::Link > link2 = collision_model->getLink(it->second);

			// iterate through collision objects of link1
			for (self_collision::Link::VecPtrCollision::const_iterator c_it1 = link1->collision_array.begin(); c_it1 != link1->collision_array.end(); c_it1++)
			{
				// iterate through collision objects of link2
				for (self_collision::Link::VecPtrCollision::const_iterator c_it2 = link2->collision_array.begin(); c_it2 != link2->collision_array.end(); c_it2++)
				{
					double distance = self_collision::CollisionModel::getDistance(*((*c_it1)->geometry.get()), T_B_L1 * (*c_it1)->origin, *((*c_it2)->geometry.get()), T_B_L2 * (*c_it2)->origin, d1, d2);
					double dist2 = (d1-d2).Norm();
					if (distance-dist2 > 0.01 || distance-dist2 < -0.01)
					{
						ROS_ERROR("%s  %s   %d  %d  %lf", it->first.c_str(), it->second.c_str(), (*c_it1)->geometry->type, (*c_it2)->geometry->type, distance);
					}
					if (distance < 0.0)
					{
						ROS_INFO("%s  %s   %d  %d   %lf", it->first.c_str(), it->second.c_str(), (*c_it1)->geometry->type, (*c_it2)->geometry->type, distance);
					}
					else if (distance < 0.15)
					{
						m_id = publishLineMarker(vis_pub, m_id, d1, d2, 1, 0, 0);
						low_distance_count++;
					}
				}
			}
		}

		ROS_INFO("low distances: %d", low_distance_count);
		ros::spinOnce();

		ros::Duration(0.05).sleep();
	}

	return 0;
}


