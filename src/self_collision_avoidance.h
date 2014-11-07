// Copyright 2014 WUT
/*
 * self_collision_avoidance.h
 *
 *  Created on: 06 nov 2014
 *      Author: dseredyn
 */

#ifndef SELF_COLLISION_AVOIDANCE_H_
#define SELF_COLLISION_AVOIDANCE_H_

#include <string>

#include "rtt/TaskContext.hpp"
#include "rtt/Port.hpp"

#include "sensor_msgs/JointState.h"
#include "visualization_msgs/MarkerArray.h"
#include "urdf_collision_parser.h"

//#include "cartesian_trajectory_msgs/CartesianTrajectory.h"
//#include "geometry_msgs/Pose.h"

//#include "Eigen/Dense"

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

class SelfCollisionAvoidance: public RTT::TaskContext {
public:
	explicit SelfCollisionAvoidance(const std::string& name);
	virtual ~SelfCollisionAvoidance();
	virtual bool configureHook();
	virtual bool startHook();
	virtual void updateHook();

private:
	RTT::InputPort<sensor_msgs::JointState> joint_in_;
	sensor_msgs::JointState joint_states_;

	RTT::OutputPort<visualization_msgs::MarkerArray> markers_out_;
	visualization_msgs::MarkerArray markers_;

	int joints_count_;
	boost::shared_ptr<self_collision::CollisionModel> collision_model_;
	std::map<std::string,int> joint_name_2_id_map_;
	std::vector<double> joint_positions_by_id_;
	std::vector<KDL::Frame> transformations_by_id_;
	std::vector<int> fk_seq_;
	std::vector<Distance> distances;

	// properties
	std::string prop_robot_description_;
	std::string prop_robot_semantic_description_;
	int prop_distances_count_;
	int prop_d0_;

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
