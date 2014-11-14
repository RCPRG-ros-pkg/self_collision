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

#ifndef SELF_COLLISION_AVOIDANCE_H_
#define SELF_COLLISION_AVOIDANCE_H_

#include <string>

#include "rtt/TaskContext.hpp"
#include "rtt/Port.hpp"

#include "sensor_msgs/JointState.h"
#include "visualization_msgs/MarkerArray.h"
#include "self_collision/urdf_collision_parser.h"

#include "qhull_msgs/PointLists.h"
#include "qhull_msgs/QhullList.h"

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

	int marker_id_;

	void addMarkers(visualization_msgs::MarkerArray &marker_array);
	void updateMarkers(visualization_msgs::MarkerArray &marker_array, const KDL::Frame &T_B_i);

};

class SelfCollisionAvoidance: public RTT::TaskContext {
public:
	explicit SelfCollisionAvoidance(const std::string& name);
	virtual ~SelfCollisionAvoidance();
	virtual bool configureHook();
	virtual bool startHook();
	virtual void stopHook();
	virtual void updateHook();

private:
	// ports and buffers
	RTT::InputPort<sensor_msgs::JointState> joint_in_;
	sensor_msgs::JointState joint_states_;

	RTT::OutputPort<visualization_msgs::MarkerArray> markers_out_;
	visualization_msgs::MarkerArray markers_;

	RTT::OutputPort<qhull_msgs::PointLists> qhull_points_out_2_;
	qhull_msgs::PointLists qhull_points_2_;

	RTT::InputPort<qhull_msgs::QhullList> qhull_data_in_2_;
	qhull_msgs::QhullList qhull_data_2_;

	int joints_count_;
	KDL::Tree robot_tree_;
	boost::shared_ptr<self_collision::CollisionModel> collision_model_;
	std::map<std::string,int> joint_name_2_index_map_;
	std::vector<double> joint_positions_by_index_;
	std::vector<KDL::Frame> transformations_by_index_;
	std::vector<int> fk_seq_;
	std::vector<Distance> distances;
	self_collision::Link::VecPtrCollision convex_hull_vector_;

	// properties
	std::string prop_robot_description_;
	std::string prop_robot_semantic_description_;
	int prop_distances_count_;
	double prop_d0_;

	std::vector<bool> calculated_fk_;

/*  RTT::InputPort<cartesian_trajectory_msgs::CartesianTrajectoryConstPtr > port_trajectory_;
  RTT::InputPort<geometry_msgs::Pose > port_cartesian_position_;
  RTT::OutputPort<geometry_msgs::Pose > port_cartesian_command_;

  cartesian_trajectory_msgs::CartesianTrajectoryConstPtr trajectory_;
  geometry_msgs::Pose setpoint_;
  geometry_msgs::Pose old_point_;

  size_t trajectory_ptr_;

  bool activate_pose_init_property_;
  geometry_msgs::Pose init_setpoint_property_;

  bool last_point_not_set_;
  bool trajectory_active_;*/
};

#endif	// SELF_COLLISION_AVOIDANCE_H_
