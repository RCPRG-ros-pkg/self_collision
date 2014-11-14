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

// for memory allocation tests
#include <malloc.h>
#include <pthread.h>

// for time measurment tests
#include <sys/time.h>

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

void *(*old_malloc_hook) (size_t size, const void *caller);
//void (*old_free_hook) (void *ptr, const void *caller);

int malloc_count = 0;
int free_count = 0;

static void *my_malloc_hook (size_t size, const void *caller);
//static void my_free_hook (void *ptr, const void *caller);

pthread_mutex_t malloc_mutex;

static void *my_malloc_hook (size_t size, const void *caller)
{
	pthread_mutex_lock(&malloc_mutex);
	void *result;
	/* Restore all old hooks */
	__malloc_hook = old_malloc_hook;
//	__free_hook = old_free_hook;
	/* Call recursively */
	result = malloc (size);
	/* Save underlying hooks */
//	old_malloc_hook = __malloc_hook;
//	old_free_hook = __free_hook;

	malloc_count++;
	/* printf might call malloc, so protect it too. */
//	printf ("malloc (%u) returns %p\n", (unsigned int) size, result);
	/* Restore our own hooks */
	__malloc_hook = my_malloc_hook;
//	__free_hook = my_free_hook;
	pthread_mutex_unlock(&malloc_mutex);
	return result;
}
/*
static void my_free_hook (void *ptr, const void *caller)
{
	pthread_mutex_lock(&malloc_mutex);
	__malloc_hook = old_malloc_hook;
	__free_hook = old_free_hook;
	free (ptr);

	free_count++;
//	printf ("freed pointer %p\n", ptr);
	__malloc_hook = my_malloc_hook;
	__free_hook = my_free_hook;
	pthread_mutex_unlock(&malloc_mutex);
}
*/
void enableMallocHook()
{
	pthread_mutex_lock(&malloc_mutex);
	old_malloc_hook = __malloc_hook;
//	old_free_hook = __free_hook;
	__malloc_hook = my_malloc_hook;
//	__free_hook = my_free_hook;
	pthread_mutex_unlock(&malloc_mutex);
}

void disableMallocHook()
{
	pthread_mutex_lock(&malloc_mutex);
	__malloc_hook = old_malloc_hook;
//	__free_hook = old_free_hook;
	pthread_mutex_unlock(&malloc_mutex);
}

class Distance
{
public:
	int i_;
	int j_;

	// distance
	double d_;

	// position of point at link i in frame i
	KDL::Vector xi_;

	// position of point at link j in frame i
	KDL::Vector xj_;
};

timespec diff(timespec start, timespec end)
{
	timespec temp;
	if ((end.tv_nsec-start.tv_nsec)<0) {
		temp.tv_sec = end.tv_sec-start.tv_sec-1;
		temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
	} else {
		temp.tv_sec = end.tv_sec-start.tv_sec;
		temp.tv_nsec = end.tv_nsec-start.tv_nsec;
	}
	return temp;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "qhull_test");
	ros::NodeHandle n;

	initQhull();

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

	ROS_INFO("parsing robot_description for self-collision data");
	boost::shared_ptr<self_collision::CollisionModel> collision_model = self_collision::CollisionModel::parseURDF(robot_description_);

	// read robot semantic description
	ROS_INFO("parsing robot_semantic_description for self-collision data");
	std::string robot_semantic_description_;
	node.param("robot_semantic_description", robot_semantic_description_, std::string());

	collision_model->parseSRDF(robot_semantic_description_);
	collision_model->generateCollisionPairs();

	ROS_INFO("%ld", collision_model->disabled_collisions.size());
	ROS_INFO("%ld", collision_model->enabled_collisions.size());

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

	// create vector of transformations
	KDL::Frame *transformation_map = new KDL::Frame[collision_model->link_count_];

	// create vector of convex hulls for quick update
	self_collision::Link::VecPtrCollision convex_hull_vector;
	// iterate through all links
	for (int l_i = 0; l_i < collision_model->link_count_; l_i++)
	{
		// iterate through collision objects
		for (self_collision::Link::VecPtrCollision::const_iterator c_it = collision_model->links_[l_i]->collision_array.begin(); c_it != collision_model->links_[l_i]->collision_array.end(); c_it++)
		{
			if ((*c_it)->geometry->type == self_collision::Geometry::CONVEX)
			{
				convex_hull_vector.push_back(*c_it);
			}
		}
	}

	// create vector of distances
	int distances_max_count = 100;
	int distances_count = 0;
	Distance *distances = new Distance[distances_max_count];

	for (int l_i = 0; l_i < collision_model->link_count_; l_i++)
	{
		collision_model->links_[l_i]->kdl_segment_ = &(robot_tree.getSegment(collision_model->links_[l_i]->name)->second);
	}

	// create vector of proper fk calculation link sequence
	int *fk_seq = new int[collision_model->link_count_];
	// update parent link information
	for (int l_i = 0; l_i < collision_model->link_count_; l_i++)
	{
		urdf::Link *parent_link = robot_model_.getLink( collision_model->links_[l_i]->name )->getParent().get();
		if (parent_link != NULL)
		{
			collision_model->links_[l_i]->parent_id_ = collision_model->getLinkId( parent_link->name );
		}
		else
		{
			collision_model->links_[l_i]->parent_id_ = -1;
			fk_seq[0] = l_i;
		}
	}
	int fk_seq_i = 1;

	while (true)
	{
		bool added = false;
		int fk_seq_i_saved = fk_seq_i;
		for (int i=0; i<fk_seq_i_saved; i++)
		{
			for (int l_i = 0; l_i < collision_model->link_count_; l_i++)
			{
				// found child of already added link
				if (collision_model->links_[l_i]->parent_id_ == fk_seq[i])
				{
					// chack if the child was added
					bool child_already_added = false;
					for (int j=0; j<fk_seq_i; j++)
					{
						if (fk_seq[j] == l_i)
						{
							child_already_added = true;
							break;
						}
					}
					// add child
					if (!child_already_added)
					{
						added = true;
						fk_seq[fk_seq_i] = l_i;
						fk_seq_i++;
					}
				}
			}
		}
		if (!added)
		{
			break;
		}
	}

	pthread_mutex_init(&malloc_mutex, NULL);

	double mean21_time = 0.0;
	int count_21 = 0;
	double mean32_time = 0.0;
	int count_32 = 0;

	const double param_d0 = 0.15;
	clearMarkers(vis_pub, 0, 1000);
	int m_id = 0;
	int last_m_id = 0;
	double angle = 0.0;
	while (ros::ok())
	{
		// clear markers
		if (m_id < last_m_id)
		{
			clearMarkers(vis_pub, m_id, last_m_id+1);
		}
		last_m_id = m_id;
		m_id = 0;

		// start time measurement
		timespec ts1, ts2, ts3;
		clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts1);

		// enable malloc hook
//		enableMallocHook();

		int malloc_count_0 = malloc_count;

		// rewrite joint positions from joint_states_map to q
		for (JointStatesMap::iterator js_it = joint_states_map.begin(); js_it != joint_states_map.end(); js_it++)
		{
			q(js_it->second.first) = js_it->second.second;
		}

		int malloc_count_1 = malloc_count;
		for (int fk_i = 0; fk_i < collision_model->link_count_; fk_i++)
		{
			int l_i = fk_seq[fk_i];
			int parent_i = collision_model->links_[l_i]->parent_id_;
			double q_i = q(collision_model->links_[l_i]->kdl_segment_->q_nr);
			if (parent_i == -1)
			{
				transformation_map[l_i] = collision_model->links_[l_i]->kdl_segment_->segment.pose(q_i);
			}
			else
			{
				transformation_map[l_i] = transformation_map[parent_i] * collision_model->links_[l_i]->kdl_segment_->segment.pose(q_i);
			}
			
		}

		int malloc_count_2 = malloc_count;

		clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts2);

		// tests
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
			distance = self_collision::CollisionModel::getDistance(caps, KDL::Frame(KDL::Vector(0,1.0,0)) * T_l1, conv, KDL::Frame(KDL::Vector(0,1.0,0)) * T_l2, d1, d2, param_d0);
			m_id = caps.publishMarker(vis_pub, m_id, KDL::Frame(KDL::Vector(0,1.0,0)) * T_l1);
			m_id = conv.publishMarker(vis_pub, m_id, KDL::Frame(KDL::Vector(0,1.0,0)) * T_l2);
			m_id = publishLineMarker(vis_pub, m_id, d1, d2, 1, 0, 0);

			distance = self_collision::CollisionModel::getDistance(conv, KDL::Frame(KDL::Vector(0,0.7,0)) * T_l1, caps, KDL::Frame(KDL::Vector(0,0.7,0)) * T_l2, d1, d2, param_d0);
			m_id = conv.publishMarker(vis_pub, m_id, KDL::Frame(KDL::Vector(0,0.7,0)) * T_l1);
			m_id = caps.publishMarker(vis_pub, m_id, KDL::Frame(KDL::Vector(0,0.7,0)) * T_l2);
			m_id = publishLineMarker(vis_pub, m_id, d1, d2, 1, 0, 0);

			distance = self_collision::CollisionModel::getDistance(caps2, KDL::Frame(KDL::Vector(0,0.4,0)) * T_l1, caps, KDL::Frame(KDL::Vector(0,0.4,0)) * T_l2, d1, d2, param_d0);
			m_id = caps2.publishMarker(vis_pub, m_id, KDL::Frame(KDL::Vector(0,0.4,0)) * T_l1);
			m_id = caps.publishMarker(vis_pub, m_id, KDL::Frame(KDL::Vector(0,0.4,0)) * T_l2);
			m_id = publishLineMarker(vis_pub, m_id, d1, d2, 1, 0, 0);

			distance = self_collision::CollisionModel::getDistance(conv2, T_l1, conv, T_l2, d1, d2, param_d0);
			m_id = conv2.publishMarker(vis_pub, m_id, T_l1);
			m_id = conv.publishMarker(vis_pub, m_id, T_l2);
			m_id = publishLineMarker(vis_pub, m_id, d1, d2, 1, 0, 0);
		}

		// check collisions between links
		distances_count = 0;
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
					double dist = self_collision::CollisionModel::getDistance(*((*c_it1)->geometry.get()), T_B_L1 * (*c_it1)->origin, *((*c_it2)->geometry.get()), T_B_L2 * (*c_it2)->origin, d1, d2, param_d0);
					double dist2 = (d1-d2).Norm();
					if (dist-dist2 > 0.01 || dist-dist2 < -0.01)
					{
//						ROS_ERROR("%s  %s   %d  %d  %lf", it->first.c_str(), it->second.c_str(), (*c_it1)->geometry->type, (*c_it2)->geometry->type, distance);
					}
					if (dist < 0.0)
					{
						ROS_INFO("collision: %d  %d   %d  %d   %lf", it->first, it->second, (*c_it1)->geometry->type, (*c_it2)->geometry->type, dist);
					}
					else if (dist < param_d0)
					{
						if (distances_count >= distances_max_count)
						{
							ROS_ERROR("too many low distances");
						}
						else
						{
							distances[distances_count].i_ = it->first;
							distances[distances_count].j_ = it->second;
							distances[distances_count].d_ = dist;
							distances[distances_count].xi_ = T_B_L1.Inverse() * d1;
							distances[distances_count].xj_ = T_B_L1.Inverse() * d2;
							distances_count++;
						}
					}
				}
			}
		}

		int malloc_count_3 = malloc_count;

//		disableMallocHook();
		clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts3);

		timespec time_diff21 = diff(ts1, ts2);
		timespec time_diff32 = diff(ts2, ts3);
		double time_diff21_s = (double)time_diff21.tv_sec + (double)time_diff21.tv_nsec/1000000000.0;
		double time_diff32_s = (double)time_diff32.tv_sec + (double)time_diff32.tv_nsec/1000000000.0;

//		ROS_INFO("time: %lf + %lf = %lf  distances: %d   %d  %d", time_diff21_s, time_diff32_s, time_diff21_s + time_diff32_s, distances_count, malloc_count, free_count);

		mean21_time += time_diff21_s;
		count_21++;
		mean32_time += time_diff32_s;
		count_32++;
		if (count_21 == 20)
		{
			ROS_INFO("time: %lf + %lf = %lf  distances: %d   %d %d %d %d", mean21_time/(double)count_21, mean32_time/(double)count_32, (mean21_time+mean32_time)/(double)count_21, distances_count, malloc_count_0, malloc_count_1, malloc_count_2, malloc_count_3);
			mean21_time = 0.0;
			count_21 = 0;
			mean32_time = 0.0;
			count_32 = 0;
		}

		//
		// convex hull computation
		//

		for (self_collision::Link::VecPtrCollision::iterator it = convex_hull_vector.begin(); it != convex_hull_vector.end(); it++)
		{
			self_collision::Convex* convex = static_cast<self_collision::Convex*>((*it)->geometry.get());
			KDL::Frame &T_B_L = transformation_map[(*it)->parent_->id_];

			std::vector<KDL::Vector> points;

			for (self_collision::Convex::ConvexPointsIdVector::iterator pt_it = convex->points_id_.begin(); pt_it != convex->points_id_.end(); pt_it++)
			{
				KDL::Frame &T_B_F = transformation_map[pt_it->first];
				KDL::Frame T_E_F = (T_B_L * (*it)->origin).Inverse() * T_B_F;
				points.push_back(T_E_F * pt_it->second);
			}
			std::vector<KDL::Vector> v_out;
			std::vector<Face> f_out;

			calculateQhull(points, v_out, f_out);
			initQhull();
			convex->updateConvex(v_out, f_out);
			KDL::Vector center;
			for (std::vector<KDL::Vector>::iterator v_it = v_out.begin(); v_it != v_out.end(); v_it++)
			{
				center += *v_it;
			}
			center = 1.0/(double)v_out.size() * center;

			double radius = 0.0;
			for (std::vector<KDL::Vector>::iterator v_it = v_out.begin(); v_it != v_out.end(); v_it++)
			{
				double d = ((*v_it)-center).Norm();
				if (d > radius)
				{
					radius = d;
				}
			}
			convex->center_ = center;
			convex->radius_ = radius;
		}

		//
		// visualization
		//

		// draw all links' collision objects
		// iterate through all links
		for (int l_i = 0; l_i < collision_model->link_count_; l_i++)
		{
			// iterate through collision objects
			for (self_collision::Link::VecPtrCollision::const_iterator c_it = collision_model->links_[l_i]->collision_array.begin(); c_it != collision_model->links_[l_i]->collision_array.end(); c_it++)
			{
				m_id = (*c_it)->geometry->publishMarker(vis_pub, m_id, transformation_map[l_i] * (*c_it)->origin);
			}
		}

		// draw all low distances
		for (int l_d = 0; l_d < distances_count; l_d++)
		{
			KDL::Frame &T_B_L1 = transformation_map[distances[l_d].i_];
			m_id = publishLineMarker(vis_pub, m_id, T_B_L1 * distances[l_d].xi_, T_B_L1 * distances[l_d].xj_, 1, 0, 0);
		}

		ros::spinOnce();

		ros::Duration(0.05).sleep();
	}

	pthread_mutex_destroy(&malloc_mutex);

	return 0;
}


