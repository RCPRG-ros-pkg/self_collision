// Copyright 2014 WUT
/*
 * self_collision_avoidance.cpp
 *
 *  Created on: 06 nov 2014
 *      Author: dseredyn
 */

#include "self_collision_avoidance.h"

//#include <string>

//#include "rtt_rosclock/rtt_rosclock.h"
//#include "Eigen/Geometry"
#include "rtt/Component.hpp"

#include "urdf/model.h"
#include <kdl_parser/kdl_parser.hpp>
#include "urdf_collision_parser.h"

SelfCollisionAvoidance::SelfCollisionAvoidance(const std::string& name) :
	RTT::TaskContext(name),
	joints_count_(0)
{
	this->addProperty("robot_description", prop_robot_description_);
	this->addProperty("robot_semantic_description", prop_robot_semantic_description_);
	this->addProperty("distances_count", prop_distances_count_);
	this->addProperty("d0", prop_d0_);
	this->ports()->addPort("dbg_joint_states", joint_in_);
	this->ports()->addPort("dbg_markers", markers_out_);
}

SelfCollisionAvoidance::~SelfCollisionAvoidance() {
}

bool SelfCollisionAvoidance::configureHook() {

	// read KDL::Tree form the robot_description
	KDL::Tree robot_tree;
	if (!kdl_parser::treeFromString(prop_robot_description_, robot_tree))
	{
		RTT::log(RTT::Error) << "could not read robot_description parameter" << std::endl;
		return false;
	}

	// read the robot model from the robot_description
	urdf::Model robot_model_;
	robot_model_.initString(prop_robot_description_);

	collision_model_ = self_collision::CollisionModel::parseURDF(prop_robot_description_);
	collision_model_->parseSRDF(prop_robot_semantic_description_);
	collision_model_->generateCollisionPairs();

	// prepare the map of joint states
	for (KDL::SegmentMap::const_iterator seg_it = robot_tree.getSegments().begin(); seg_it != robot_tree.getSegments().end(); seg_it++)
	{
		KDL::Joint::JointType type = seg_it->second.segment.getJoint().getType();
		if (	type == KDL::Joint::RotAxis ||
			type == KDL::Joint::RotX ||
			type == KDL::Joint::RotY ||
			type == KDL::Joint::RotZ)
		{
			joint_name_2_id_map_.insert(std::make_pair<std::string,int>(seg_it->second.segment.getJoint().getName(),seg_it->second.q_nr));
		}		
	}

	joints_count_ = joint_name_2_id_map_.size();
	RTT::log(RTT::Info) << "number of joints:" << joints_count_ << std::endl;

	joint_positions_by_id_.resize(joints_count_);

	transformations_by_id_.resize(collision_model_->link_count_);

	// update the links graph with KDL data
	for (int l_i = 0; l_i < collision_model_->link_count_; l_i++)
	{
		collision_model_->links_[l_i]->kdl_segment_ = &(robot_tree.getSegment(collision_model_->links_[l_i]->name)->second);
	}

	//
	// create vector of proper fk calculation link sequence (sort the links graph)
	//
	fk_seq_.resize(collision_model_->link_count_);
	// update parent link information
	for (int l_i = 0; l_i < collision_model_->link_count_; l_i++)
	{
		urdf::Link *parent_link = robot_model_.getLink( collision_model_->links_[l_i]->name )->getParent().get();
		if (parent_link != NULL)
		{
			collision_model_->links_[l_i]->parent_id_ = collision_model_->getLinkId( parent_link->name );
		}
		else
		{
			collision_model_->links_[l_i]->parent_id_ = -1;
			fk_seq_[0] = l_i;
		}
	}
	// sort the graph
	int fk_seq_i = 1;
	while (true)
	{
		bool added = false;
		int fk_seq_i_saved = fk_seq_i;
		for (int i=0; i<fk_seq_i_saved; i++)
		{
			for (int l_i = 0; l_i < collision_model_->link_count_; l_i++)
			{
				// found child of already added link
				if (collision_model_->links_[l_i]->parent_id_ == fk_seq_[i])
				{
					// check if the child was added
					bool child_already_added = false;
					for (int j=0; j<fk_seq_i; j++)
					{
						if (fk_seq_[j] == l_i)
						{
							child_already_added = true;
							break;
						}
					}
					// add child
					if (!child_already_added)
					{
						added = true;
						fk_seq_[fk_seq_i] = l_i;
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

	// create vector of distances
	int distances_max_count = 100;
	int distances_count = 0;
	distances.resize(prop_distances_count_);



	// prepare communication buffers
	joint_states_.name.resize(joints_count_);
	joint_states_.position.resize(joints_count_);
	joint_states_.velocity.resize(joints_count_);
	joint_states_.effort.resize(joints_count_);


//	port_JointPosition.setDataSample(jnt_pos_);
	return true;
}

bool SelfCollisionAvoidance::startHook() {
	return true;
/*
  if (activate_pose_init_property_) {
    setpoint_ = init_setpoint_property_;
  } else {
    if (port_cartesian_position_.read(setpoint_) == RTT::NoData) {
      return false;
    }
  }
  last_point_not_set_ = false;
  trajectory_active_ = false;
  return true;*/
}

void SelfCollisionAvoidance::updateHook() {
	joint_in_.read(joint_states_);

	if (joint_states_.name.size() != joints_count_)
	{
		RTT::log(RTT::Error) << "wrong number of joints: " << joint_states_.name.size() << ", should be: " << joints_count_ << std::endl;
		return;
	}

	// arrange the joint positions to their ids in the KDL tree
	for (int i=0; i<joint_states_.name.size(); i++)
	{
		joint_positions_by_id_[ joint_name_2_id_map_[joint_states_.name[i]] ] = joint_states_.position[i];
	}

	// calculate the forward kinematics for all links
	for (int fk_i = 0; fk_i < collision_model_->link_count_; fk_i++)
	{
		int l_i = fk_seq_[fk_i];
		int parent_i = collision_model_->links_[l_i]->parent_id_;
		double q_i = joint_positions_by_id_[l_i];
		if (parent_i == -1)
		{
			transformations_by_id_[l_i] = collision_model_->links_[l_i]->kdl_segment_->segment.pose(q_i);
		}
		else
		{
			transformations_by_id_[l_i] = transformations_by_id_[parent_i] * collision_model_->links_[l_i]->kdl_segment_->segment.pose(q_i);
		}
	}

	// check collisions between links
	int distances_count = 0;
	for (self_collision::CollisionModel::CollisionPairs::iterator it = collision_model_->enabled_collisions.begin(); it != collision_model_->enabled_collisions.end(); it++)
	{
		KDL::Vector d1, d2;
		KDL::Frame T_l1, T_l2;
		KDL::Frame &T_B_L1 = transformations_by_id_[it->first];
		KDL::Frame &T_B_L2 = transformations_by_id_[it->second];

		boost::shared_ptr< const self_collision::Link > link1 = collision_model_->getLink(it->first);
		boost::shared_ptr< const self_collision::Link > link2 = collision_model_->getLink(it->second);

		// iterate through collision objects of link1
		for (self_collision::Link::VecPtrCollision::const_iterator c_it1 = link1->collision_array.begin(); c_it1 != link1->collision_array.end(); c_it1++)
		{
			// iterate through collision objects of link2
			for (self_collision::Link::VecPtrCollision::const_iterator c_it2 = link2->collision_array.begin(); c_it2 != link2->collision_array.end(); c_it2++)
			{
				double dist = self_collision::CollisionModel::getDistance(*((*c_it1)->geometry.get()), T_B_L1 * (*c_it1)->origin, *((*c_it2)->geometry.get()), T_B_L2 * (*c_it2)->origin, d1, d2, prop_d0_);
				double dist2 = (d1-d2).Norm();
				if (dist-dist2 > 0.01 || dist-dist2 < -0.01)
				{
					// it is ok
				}

				if (dist < 0.0)
				{
					RTT::log(RTT::Error) << "collision: " << it->first << " " << it->second << " " << (*c_it1)->geometry->type << " " << (*c_it2)->geometry->type << " " << dist << std::endl;
				}
				else if (dist < prop_d0_)
				{
					if (distances_count >= prop_distances_count_)
					{
						RTT::log(RTT::Error) << "too many low distances" << std::endl;
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






/*  if (port_trajectory_.read(trajectory_) == RTT::NewData) {
    trajectory_ptr_ = 0;
    old_point_ = setpoint_;
    last_point_not_set_ = true;
    trajectory_active_ = true;
  //  std::cout << std::endl<< "CartesianInterpolator new trj" << std::endl<< std::endl<< std::endl;
  }
  //std::cout << "CartesianInterpolator" << std::endl;
  ros::Time now = rtt_rosclock::host_now();
  if (trajectory_active_ && trajectory_ && (trajectory_->header.stamp < now)) {
    for (; trajectory_ptr_ < trajectory_->points.size(); trajectory_ptr_++) {
      ros::Time trj_time = trajectory_->header.stamp
          + trajectory_->points[trajectory_ptr_].time_from_start;
      if (trj_time > now) {
        break;
      }
    }

    if (trajectory_ptr_ < trajectory_->points.size()) {
      if (trajectory_ptr_ == 0) {
        cartesian_trajectory_msgs::CartesianTrajectoryPoint p0;
        p0.time_from_start.fromSec(0.0);
        p0.pose = old_point_;
        setpoint_ = interpolate(p0, trajectory_->points[trajectory_ptr_], now);
      } else {
        setpoint_ = interpolate(trajectory_->points[trajectory_ptr_ - 1],
                                trajectory_->points[trajectory_ptr_], now);
      }
    } else if (last_point_not_set_) {
      setpoint_ = trajectory_->points[trajectory_->points.size() - 1].pose;
      last_point_not_set_ = false;
    }
  }
  port_cartesian_command_.write(setpoint_);
*/
}

ORO_CREATE_COMPONENT(SelfCollisionAvoidance)

